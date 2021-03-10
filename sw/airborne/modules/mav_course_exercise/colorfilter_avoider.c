/*
 * Copyright (C) Daniel Toth <d.toth@student.tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/mav_course_exercise/colorfilter_avoider.c"
 * @author Daniel Toth <d.toth@student.tudelft.nl>
 * Experimental obstacle avoidance based on color filtering.
 */

#include "modules/mav_course_exercise/colorfilter_avoider.h"
#include "modules/computer_vision/opencv_contour.h"
#include "modules/computer_vision/lib/vision/image.h"

#ifndef COLORFILTER_AVOIDER_FPS
#define COLORFILTER_AVOIDER_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND,
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS,
    REENTER_ARENA
};


// define color ranges
struct HSI_color orange_low = {75, 0, 150};
struct HSI_color orange_high = {115, 255, 255};

// define settings
float oag_max_speed = 0.5f;                 // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);    // heading change setpoint for avoidance [rad/s]
float threatening_obstacle_distance = 1.;   // The obstacle distance at which point we must start worrying.

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine

float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free



extern struct contour_estimation cont_est;
extern struct contour_threshold cont_thres;

struct contour_estimation threat_obstacle;

static void colorfilter_avoider_cb(struct image_t *img){
    //TODO: Do I need to make a copy of the img, so that we don't overwright the original one? If so, can I use image_copy from image.c? How do I allocate memory for the output->buf?

    //TODO: This colorfilt might be redundant, as we might be able to do this within find_contour by defining cont_thres
//    int count;
//    count = image_yuv422_colorfilt(img, img,
//                                   orange_low->y, orange_high->y,
//                                   orange_low->u, orange_high->u,
//                                   orange_low->v, orange_high->v);

    // Set the cont_thres for find_contour
    cont_thres->lower_y = orange_low->y;
    cont_thres->upper_y = orange_high->y;
    cont_thres->lower_u = orange_low->u;
    cont_thres->upper_u = orange_high->u;
    cont_thres->lower_v = orange_low->v;
    cont_thres->upper_v = orange_high->v;
    find_contour((char *) img->buf, img->w, img->h);
    //The output of this is cont_est

    //Make a copy of cont_est, just to make sure it doesn't change in the background due to parallell processes
    memcpy(threat_obstacle, cont_est);

    //TODO: this only gets the LARGEST contour, but not all. Can I modify the find_contour in opencv_contour for my purposes? I want more details i.e.: cont_est for the N largest contour and their area


    //TODO: For each contour: filter them based on OpenCV moments['M00']

}


void colorfilter_avoider_init(void)
{
    cv_add_to_device(&COLORFILTER_AVOIDER_CAMERA, colorfilter_avoider_cb, COLORFILTER_AVOIDER_FPS);
}

void colorfilter_avoider_periodic(void)
{
    // update our safe confidence using color threshold
    if(threat_obstacle->contour_d_x > threatening_obstacle_distance){
        obstacle_free_confidence++;
    } else {
        obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
    }
    // bound obstacle_free_confidence
    Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

    //TODO: I changed the obstacle_free_confidance speed factor from 0.2 to 0.1 for more gradual and more conservative slow down. I might want to change it back later
    float speed_sp = fminf(oag_max_speed, 0.1f * obstacle_free_confidence);

    switch (navigation_state){
        case SAFE:
            if (obstacle_free_confidence == 0){
                navigation_state = OBSTACLE_FOUND;
            }
//            else if(//TODO: OUT_OF_BOUNDS condition){
//                navigation_state = OUT_OF_BOUNDS;
//            }
            else {
                guidance_h_set_guided_body_vel(speed_sp, 0);
            }
            break;

        case OBSTACLE_FOUND:
            // stop
            guidance_h_set_guided_body_vel(0, 0);

            //turn away from the obstacle
            if (threat_obstacle->contour_d_y > 0.0){
                // positive y means the obstacle is to the left of the drone
                // so turn right
                chooseIncrementAvoidance(true);
            }
            else {
                // otherwise turn left
                chooseIncrementAvoidance(false);
            }

            navigation_state = SEARCH_FOR_SAFE_HEADING;
            break;

        case SEARCH_FOR_SAFE_HEADING:
            guidance_h_set_guided_heading_rate(avoidance_heading_direction * oag_heading_rate);

            // make sure we have a couple of good readings before declaring the way safe
            if (obstacle_free_confidence >= 2){
                guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
                navigation_state = SAFE;
            }
            break;

        case OUT_OF_BOUNDS:
            // stop
            guidance_h_set_guided_body_vel(0, 0);

            // start turn back into arena
            guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(15));

            navigation_state = REENTER_ARENA;
            break;

        case REENTER_ARENA:
            // TODO: REENTER_ARENA
//            // force floor center to opposite side of turn to head back into arena
//            if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
//                // return to heading mode
//                guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
//
//                // reset safe counter
//                obstacle_free_confidence = 0;
//
//                // ensure direction is safe before continuing
//                navigation_state = SAFE;
//            }
            break;
        default:
            break;
    }

    return;
}

/*
 * Sets the variable 'incrementForAvoidance' positive/negative
 */
uint8_t chooseIncrementAvoidance(void, uint8_t cw)
{
    if (cw) {
        avoidance_heading_direction = 1.f;
        VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
    }
    else {
        avoidance_heading_direction = -1.f;
        VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
    }

    return false;
}


