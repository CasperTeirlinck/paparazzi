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


// define color ranges
struct HSI_color{
    uint8 y;
    uint8 u;
    uint8 v;
};
struct HSI_color orange_low = {75, 0, 150};
struct HSI_color orange_high = {115, 255, 255};

struct image_t *colorfilter_avoider_cb(struct image_t *img){
    int count = image_yuv422_colorfilt(img, img,
                                       orange_low->y, orange_high->y,
                                       orange_low->u, orange_high->u,
                                       orange_low->v, orange_high->v);

    //TODO: How to actually use this function?
    find_contour((char *) img->buf, img->w, img->h);

    //TODO: For each contour: filter them based on OpenCV moments['M00']

    return img
}


void colorfilter_avoider_init(void)
{
    cv_add_to_device(&COLORFILTER_AVOIDER_CAMERA, object_detector1, COLORFILTER_AVOIDER_FPS);
    return;
}

void colorfilter_avoider_periodic(void)
{
    return;
}


