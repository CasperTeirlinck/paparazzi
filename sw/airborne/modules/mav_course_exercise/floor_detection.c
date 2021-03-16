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

/** @file "modules/mav_course_exercise/floor_detection.c"
 * @author Daniel Toth <d.toth@student.tudelft.nl>
 * Obstacle detection based on floor colorfiltering.
 */

#include "modules/mav_course_exercise/floor_detection.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/mav_course_exercise/ground_obstacle_detect.h"
#include "subsystems/abi.h"

#ifndef FLOOR_DETECT_FPS
#define FLOOR_DETECT_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

#ifndef FLOOR_DETECT_TYPE
#define FLOOR_DETECT_TYPE 0 ///< 0 is simulation, 1 is real flight
#endif

//TODO: Add the camera, fps, detect_type macros to the module xml - Daniel      * Done. -S.

//TODO: Define green floor color range in YUV for simulation, use the sim_poles_panels_mats - Daniel
// I found these HSV limits to be working really well, but still need to be converted to YUV!!!
// low_hsv = Scalar(35, 100, 90)
// high_hsv = Scalar(45, 255, 255)
struct YUV_color floor_simu_min = {0, 0, 0};
struct YUV_color floor_simu_max = {0, 0, 0};

//TODO: Define green floor color range in YUV for real flight, use the cyberzoo_poles_panels_mats - Daniel
struct YUV_color floor_real_min = {0, 0, 0};
struct YUV_color floor_real_max = {0, 0, 0};

struct YUV_color floor_min;
struct YUV_color floor_max;

enum color_set {
    SIMU,
    REAL
};




/// <summary>
/// Bounds num between min and max.
/// </summary>
/// <param name="num"></param>
/// <param name="min"></param>
/// <param name="max"></param>
/// <returns></returns>
int c_bound_int(int num, int min, int max) {
    int out;
    if (num < min) {
        out = min;
    }
    else if (num > max) {
        out = max;
    }
    else {
        out = num;
    }
    return out;
}


//TODO: Trying to change the C++ openCV function from ground_obstacle_detect into a image_t friendly openCV free function.
//      ...Still work in progress, see the TODOs below.
/// <summary>
/// Checks the bottom_count of pixels at the bottom of each column if they are white.
/// If more than certainty many black pixels are found at the bottom of the column, it is considered unsafe.
/// </summary>
/// <param name="img"> BW colorfiltered image in openCV::Mat </param>
/// <param name="bottom_count"> The width of the band on the bottom to scan for black </param>
/// <param name="certainty"> The number of black pixels in a column of the bottom_count band, that makes that direction unsafe. </param>
/// <returns> Array with indexes representing the column index, and values: 1 is safe, 2 is obstacle, 0 is outside of frame </returns>
int* c_ground_obstacle_detect(struct image_t *input, int safe_vector[], int invert_color = 0, int bottom_count = 10, int certainty = 1) {
    // (0, 0) is top left corner
    int threat, obstacle_color, safe_color;

    if (invert_color == 0){
        obstacle_color = 0;
        safe_color = 255;
    } else{
        obstacle_color = 255;
        safe_color = 0;
    }


    if (inverted_color == 1) {
        uint8_t y_obstacle = 0, u_obstacle = 0, v_obstacle = 0;         //black
        uint8_t y_safe = 255, u_safe = 0, v_safe = 0;                   //white
    } else{
        uint8_t y_obstacle = 255, u_obstacle = 0, v_obstacle = 0;       //white
        uint8_t y_safe = 0, u_safe = 0, v_safe = 0;                     //black
    }

    uint8_t *source = (uint8_t *)input->buf;


    // Go trough all the pixels
    //TODO: The loops here are totally wrong. This starts from the top left, but we want to start at the bottom left,
    //      ...also we want to go through the rows of each column.
    for (uint16_t y = 0; y < input->h; y++) {
        threat = 0;
        safe_vector[x] == 0;
        for (uint16_t x = 0; x < input->w; x += 2) {
            if (
                    (source[1] == y_obstacle)
                    && (source[0] == u_obstacle)
                    && (source[2] == v_obstacle)
                    ) {
                threat ++;
            } else if (
                    (source[1] == y_safe)
                    && (source[0] == u_safe)
                    && (source[2] == v_safe)
                    ) {
                threat --;
            } else {
                    //TODO: it shouldn't come to this, raise some error or something
                    return safe_vector;
            }

            threat = bound_int(threat, 0, certainty);

            if (threat == certainity && sfae_vector[x] == 0){
                safe_vector[x] = input->h - y - certainty;
            } else if (threat == 0) {
                safe_vector[x] == 0;
            }

            // Go to the next 2 pixels
            source += 4;
        }
    }

    return safe_vector;
}




static struct image_t *floor_detect_cb(struct image_t *img){
    //Create a copy of the input img, so we don't overwrite it
    struct image_t floor_img;
    image_copy(img, floor_img);


    //TODO: Use the normalised box filter function from OpenCV: blur(Mat src, Mat out, Size(krnl, krnl)
    //      ...with kernel size of 5.
    //      Hang on, it might not be necessary.


    //Apply the colorfilter for the range of greens
    uint32_t count = image_yuv422_colorfilt(floor_img, floor_img,
                                            floor_min->y, floor_max->y,
                                            floor_min->u, floor_max->u,
                                            floor_min->v, floor_max->v);


    //TODO: Make sure the array length is equal to the width of the front camera image
    int safe_array[520] = { };

    //TODO: Measure how many pixels from the bottom the bottom of the obstacle is,
    //      ...if it is in the safe distance away from the drone, hovering at the standard altitude.
    //      ...Use this pixel count for bottom_count in ground_obstacle_detect
    //TODO:Alternatively measure how far the obstacle must be, so that it's bottom starts clipping in the image,
    //      ...and maybe take that distance as the safe distance in the periodic?

    int *safe_array_pt = ground_obstacle_detect(src, safe_array, 0, 50, 5);

    //TODO: Divide the safe_array - Daniel

    uint8_t test_val = 8;
    AbiSendMsgFLOOR_DETECTION(ABI_FLOOR_DETECTION_ID, test_val);    // placeholder; send the result via Abi.
    // TODO: what variables do you want to publish via Abi? -S.

    return img;
}

void floor_detection_init(void)
{
    //TODO: Undistort the camera image if needed, see computer_vision/undistort_image.c maybe


    if (FLOOR_DETECT_TYPE == SIMU){
        floor_min = floor_simu_min;
        floor_max = floor_simu_max;
    } else if(FLOOR_DETECT_TYPE == REAL){
        floor_min = floor_real_min;
        floor_max = floor_real_max;
    }

    cv_add_to_device(&FLOOR_DETECT_CAMERA, floor_detect_cb, FLOOR_DETECT_FPS);

}

void floor_detection_periodic(void)
{
    //TODO: For most of this, check the periodic in orange_avoider_guided

    //TODO: Decide on a safe distance from the drone, i.e. 1 meter
    //TODO: Measure how many pixel wide (+ safety margin) the image of the drone from the safe distance.
    // ...i.e. take a picture with a drone of an other drone located 1 meter away in front of it
    //TODO: take this width in the center of the image, I'll call it safety band from now on.

    /// Front camera horizontal FOV 1.8rad~=103deg (bebop2.sdf #233) -S.

    //TODO: switch case logic, like in orange_avoider_guided:
    // ...if obstacle inside the safety band on one side, then turn away one increment
    // ...if obstacle inside the safety band on BOTH side, then do some evasive manoeuvre, like rotate 90deg
    // ...if no obstacle inside the safety band, keep flying forward

    return;
}


