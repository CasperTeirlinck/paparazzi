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
#include "subsystems/abi.h"
#include <stdio.h>

#ifndef FLOOR_DETECT_FPS
#define FLOOR_DETECT_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

#ifndef FLOOR_DETECT_TYPE
#define FLOOR_DETECT_TYPE 0 ///< 0 is simulation, 1 is real flight
#endif

//TODO: If I remember correctly, teh camera FPS was decreased for testing puropses by Sunyou - Dani

// Define the YUV range for the green floow for Gazebo
struct YUV_color floor_simu_min = {80, 0, 0};
struct YUV_color floor_simu_max = {105, 105, 135};

// Define the YUV range for the green floow for Cyberzoo
//TODO: Test this range on cyberzoo_poles_panels_mats - Daniel
struct YUV_color floor_real_min = {69, 10, 75};
struct YUV_color floor_real_max = {145, 133, 135};

struct YUV_color floor_min;
struct YUV_color floor_max;

enum color_set {
    SIMU,
    REAL
};



///// I leave this for navigation guys!
//static abi_event floor_detection_ev;
//static void floor_detection_test_cb(uint8_t __attribute__((unused)) sender_id, struct FloorDetectionOutput test1)
//{
//    /// I checked it working well!
//    for (int i=0;i<520;i++){
//        printf("%d, %d, \n", i, test1.obstacle_vector[i]);
//    }
//}



/// Bounds num between min and max.
/// \param num
/// \param min
/// \param max
/// \return
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


/// OpenCV is for the weak.
/// The input frame is divided into decades, aka sectors.
/// Checks the bottom_count of rows at the bottom of each column if they are within the given YUV color range.
/// If a block of pixels are out of the given range, it will be considered as an obstacle,
/// and lowest row where the obstacle started is recorded in the sector corresponding to the column in question.
/// A column is safe, when a block of pixels within the range are detected above the the block of pixels out of range.
/// This way mats, and objects far below the camera are not detected.
/// \param input image_t from the front camera
/// \param bottom_count The width of the band on the bottom to scan for black
/// \param certainty The number of pixels detected, in order to change a column from safe to unsafe, and vica versa.
/// \param y_m Y range lower limit
/// \param y_M Y range upper limit
/// \param u_m U range lower limit
/// \param u_M U range upper limit
/// \param v_m V range lower limit
/// \param v_M V range upper limit
void c_ground_obstacle_detect(struct image_t *input, int bottom_count, int certainty, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M) {

    int threat;
//    int obstacle_sector_array[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    /// you can change the variable name if you want!
    /// usage is something like this... detection_output.obstacle_vector[i] = 0;
    struct FloorDetectionOutput obstacle_sector_array;

    //Go through all pixels in pairs. Note that by default, the frame is rotated 90deg CW.
    uint8_t *pixel = (uint8_t *) input->buf;    //First pixel, top left
    for (uint16_t i = 0; i < input->h; i++) {

        //Reset the counter for the pixels out of the range.
        threat = 0;
        obstacle_sector_array.obstacle_vector[i] = 0;

        for (uint16_t j = 0; j < input->w; j += 2) {
            //Check if the pixel pair is within the range
            if (
                    (pixel[1] >= y_m)
                    && (pixel[1] <= y_M)
                    && (pixel[0] >= u_m)
                    && (pixel[0] <= u_M)
                    && (pixel[2] >= v_m)
                    && (pixel[2] <= v_M)
                    ) {

                threat-=2;

            } else {
                if (j <= bottom_count) {
                    threat+=2;
                }
            }

            threat = c_bound_int(threat, 0, certainty);

            //If the number of pixels out of the range equal or exceed certainty, the column is unsafe.
//	        int idx = i * 10 / 520;
            if (threat == certainty) {
                //In each sector keep track of only the closest obstacle.
                // Obstacles located lower on the frame are assumed to be closer in space.
                if (obstacle_sector_array.obstacle_vector[i] == 0 || obstacle_sector_array.obstacle_vector[i] > j - certainty + 3){
                    obstacle_sector_array.obstacle_vector[i] = j - certainty + 3;     // +3 is necessary, in case obstacle at j=0: threat at j=0 will be 2, so certainty will be reached at j = certainty-2. We don't write 0 in the array (as 0 represents no obstacle), so instead write 1, so +3
                }
            } else if (threat == 0) {
                obstacle_sector_array.obstacle_vector[i] = 0;
            }

            pixel += 4; //Go to the next pixel pair.
        }
    }

    //TODO: This is here for testing purposes, deletet it for the flight version
//    for (int a=0; a<10; a++){
//        printf("%d ", obstacle_sector_array[a]);
//    }
//    printf("\n");

    // publish the result via Abi.
    AbiSendMsgFLOOR_DETECTION(ABI_FLOOR_DETECTION_ID, obstacle_sector_array);


    //publish the result via Abi. Passing 10 int is a stupid way but safe haha
//    AbiSendMsgFLOOR_DETECTION(ABI_FLOOR_DETECTION_ID, obstacle_sector_array[0], obstacle_sector_array[1],
//                              obstacle_sector_array[2], obstacle_sector_array[3], obstacle_sector_array[4],
//                              obstacle_sector_array[5], obstacle_sector_array[6], obstacle_sector_array[7],
//                              obstacle_sector_array[8], obstacle_sector_array[9]);
}


static struct image_t *floor_detect_cb(struct image_t *img){

    //TODO: Measure how many pixels from the bottom the bottom of the obstacle is,
    //      ...if it is in the safe distance away from the drone, hovering at the standard altitude.
    //      ...Use this pixel count for bottom_count in ground_obstacle_detect
    //TODO:Alternatively measure how far the obstacle must be, so that it's bottom starts clipping in the image,
    //      ...and maybe take that distance as the safe distance in the periodic?

    c_ground_obstacle_detect(img, 50, 6, floor_min.y, floor_max.y, floor_min.u, floor_max.u, floor_min.v, floor_max.v);

    return img;
}


void floor_detection_init(void)
{
    //Select the correct color range.
    if (FLOOR_DETECT_TYPE == SIMU){
        floor_min = floor_simu_min;
        floor_max = floor_simu_max;
        printf("SIMU");
    } else if(FLOOR_DETECT_TYPE == REAL){
        floor_min = floor_real_min;
        floor_max = floor_real_max;
        printf("REAL");
    }

    // Subscribe the camera image
    cv_add_to_device(&FLOOR_DETECT_CAMERA, floor_detect_cb, FLOOR_DETECT_FPS);
//    AbiBindMsgFLOOR_DETECTION(ABI_FLOOR_DETECTION_ID, &floor_detection_ev, floor_detection_test_cb);

}


