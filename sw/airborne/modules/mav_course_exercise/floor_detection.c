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

#ifndef FLOOR_DETECT_FPS
#define FLOOR_DETECT_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

#ifndef FLOOR_DETECT_TYPE
#define FLOOR_DETECT_TYPE 0 ///< 0 is simulation, 1 is real flight
#endif

//TODO: Add the camera, fps, detect_type macros to the module xml - Daniel

//TODO: Make a new airframe xml with this module - Sunyou

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

//TODO: does this need to be static?
struct image_t floor_img;


static struct image_t *floor_detect_cb(struct image_t *img){
    //Create a copy of the input img, so we don't overwrite it
    //TODO: do we need to free this image via image_free() at some point?
    image_create(struct image_t *floor_img, img->w, img->h, img->type);


    //TODO: Use the normalised box filter function from OpenCV: blur(Mat src, Mat out, Size(krnl, krnl)
    //      ...with kernel size of 5.


    //TODO: Apply the colorfilter below:
    //uint32_t count = image_yuv422_colorfilt(src, out,
    //                                        floor_min->y, floor_max->y,
    //                                        floor_min->u, floor_max->u,
    //                                        floor_min->v, floor_max->v);


    //TODO: Make sure the array length is equal to the width of the front camera image
    //int safe_array[520] = { };

    //TODO: Measure how many pixels from the bottom the bottom of the obstacle is,
    // ...if it is in the safe distance away from the drone, hovering at the standard altitude.
    // ...Use this pixel count for bottom_count in ground_obstacle_detect
    //TODO:Alternatively measure how far the obstacle must be, so that it's bottom starts clipping in the image,
    // ...and maybe take that distance as the safe distance in the periodic?

    //int *safe_array_pt = ground_obstacle_detect(src, safe_array);

    //TODO: Divide the safe_array - Daniel


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

    //TODO: Is this the right way to do it, or do we need to do some kind of ABI magic?
    cv_add_to_device(&FLOOR_DETECT_CAMERA, floor_detect_cb, FLOOR_DETECT_FPS);

}

void floor_detection_periodic(void)
{
    //TODO: For most of this, check the periodic in orange_avoider_guided

    //TODO: Decide on a safe distance from the drone, i.e. 1 meter
    //TODO: Measure how many pixel wide (+ safety margin) the image of the drone from the safe distance.
    // ...i.e. take a picture with a drone of an other drone located 1 meter away in front of it
    //TODO: take this width in the center of the image, I'll call it safety band from now on.


    //TODO: switch case logic, like in orange_avoider_guided:
    // ...if obstacle inside the safety band on one side, then turn away one increment
    // ...if obstacle inside the safety band on BOTH side, then do some evasive manoeuvre, like rotate 90deg
    // ...if no obstacle inside the safety band, keep flying forward

    return;
}


