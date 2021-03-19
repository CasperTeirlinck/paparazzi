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
//#include "modules/mav_course_exercise/ground_obstacle_detect.h"
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
/// OpenCV is for the weak.
/// Checks the bottom_count of pixels at the bottom of each column if they are white.
/// If more than certainty many black pixels are found at the bottom of the column, it is considered unsafe.
/// </summary>
/// <param name="img"> colorfiltered image_t </param>
/// <param name="invert_color"> Flase/0 -> input is colorfiltered such that obstacle is marked with white, and safe is maerked with black. You can invert this by setting to 1.
/// <param name="bottom_count"> The width of the band on the bottom to scan for black </param>
/// <param name="certainty"> The number of black pixels in a column of the bottom_count band, that makes that direction unsafe. </param>
/// <returns> Array with indexes representing the column index, and values: 1 is safe, 2 is obstacle, 0 is outside of frame </returns>
//int* c_ground_obstacle_detect(struct image_t *input, int safe_vector[], int invert_color=0, int bottom_count=10, int certainty=1) {
int* c_ground_obstacle_detect(struct image_t *input, int safe_vector[], int invert_color, int bottom_count0, int certainty, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M) {
    // (0, 0) is top left corner
    int threat, obstacle_color, safe_color;

    uint8_t y_obstacle = 255, u_obstacle = 0, v_obstacle = 0;       //white
    uint8_t y_safe = 0, u_safe = 0, v_safe = 0;                     //black
    if (invert_color == 1) {
        uint8_t y_obstacle = 0, u_obstacle = 0, v_obstacle = 0;         //black
        uint8_t y_safe = 255, u_safe = 0, v_safe = 0;                   //white
    }
//    else{
//        uint8_t y_obstacle = 255, u_obstacle = 0, v_obstacle = 0;       //white
//        uint8_t y_safe = 0, u_safe = 0, v_safe = 0;                     //black
//    }


    // Go trough all the pixels, starting from bottom left
    uint8_t *zero = (uint8_t *)input->buf;
    //TODO: Maybe we have to go through the width in steps of to, i.e. y+=2
    for (uint8_t i = 0; i < input->w; i+=2) {
        // Go to the bottom of the next column
        uint8_t *column = zero + i + (input->h - 1)*input->w;

        threat = 0;
        safe_vector[i] == 0;
        for (uint8_t j = 0; j < input->h; j++) {
            // Go up one row in this column
            uint8_t *pixel = column - j*input->w;

            if (
                    (pixel[1] >= y_m)
                    && (pixel[1] <= y_M)
                    && (pixel[0] >= u_m)
                    && (pixel[0] <= u_M)
                    && (pixel[2] >= v_m)
                    && (pixel[2] <= v_M)
                    )  {
                threat ++;
            } else{
                threat --;
            }

            threat = c_bound_int(threat, 0, certainty);

            if (threat == certainty && safe_vector[i] == 0){
                safe_vector[i] = input->h - j - certainty;
            } else if (threat == 0) {
                safe_vector[i] == 0;
            }
        }
    }
    return safe_vector;
}

/// <summary>
/// From binary array of length 5, to single decimal
/// </summary>
/// <param name="vector"> </param>
/// <returns> </returns>
uint8_t binary_encoder(int *vector){
    int factor = 1;
    uint8_t out = 0;

    for (int i=4; i >= 0; i--){
        out += vector[i] * factor;
        factor * 2;
    }
    return out;
}

/// <summary>
/// From decimal int, to binary array with length 5
/// </summary>
/// <param name="code"> </param>
/// <returns> </returns>
int *binary_decoder(uint8_t code){
    int n = code;
    int factor = 1;
    int array[5];
    int i = 4;

    while(n > 0){
        array[i] = n % 2;
        n = n / 2;
        i--;
    }
    return array;
}


void floor_detect_cb(struct image_t *img){
    //Create a copy of the input img, so we don't overwrite it
    struct image_t floor_img;
    image_copy(img, &floor_img);


    //TODO: Use the normalised box filter function from OpenCV: blur(Mat src, Mat out, Size(krnl, krnl)
    //      ...with kernel size of 5.
    //      Hang on, it might not be necessary.


    //Apply the colorfilter for the range of greens
//    uint16_t count = image_yuv422_colorfilt(&img, &floor_img,
//                                            floor_min.y, floor_max.y,
//                                            floor_min.u, floor_max.u,
//                                            floor_min.v, floor_max.v);


    //TODO: Make sure the array length is equal to the width of the front camera image
    int safe_array[260] = { };

    //TODO: Measure how many pixels from the bottom the bottom of the obstacle is,
    //      ...if it is in the safe distance away from the drone, hovering at the standard altitude.
    //      ...Use this pixel count for bottom_count in ground_obstacle_detect
    //TODO:Alternatively measure how far the obstacle must be, so that it's bottom starts clipping in the image,
    //      ...and maybe take that distance as the safe distance in the periodic?

    int *safe_array_pt = c_ground_obstacle_detect(&floor_img, safe_array, 0, 50, 5,
                                                  floor_min.y, floor_max.y,
                                                  floor_min.u, floor_max.u,
                                                  floor_min.v, floor_max.v);

    //TODO: Divide the safe_array - Daniel
    //Take the closest obstacle in the given sector
    int obstacle_sector_array[5] = {};
    int range_low, range_high, range_increment;
    for (int i = 0; i < 5; i++){
        range_increment = 260/5;
        range_low = i*range_increment;
        range_high = range_low + range_increment;
        for (int j = range_low; j < range_high; j++){
            //Low value means the obstacle is close to the drone
            //(since the value is the height of the obstacle in the image)
            if (safe_array[j] > 0 && obstacle_sector_array[i] > safe_array[j]){
                obstacle_sector_array[i] = 1;
                break;
            }
        }
    }

    // TODO: what variables do you want to publish via Abi? -S.
    //      What about this: Assume we divide the entire width of the frame into 5 sectors: far left, near left, center, near right, far right.
    //      If an obstacle is detected in a sector, we set the value to 1, if it's safe, set the value to 0, see the block above.
    //      Now what we have is essentially a binary number, i.e: obstacle in the far and near left -> 11000.
    //      We can convert this binary to decimal, and send that decimal via Abi.
    //      Whatever is subscribed to it will need to decode it back to binary and then to an array.
    uint8_t test_val = binary_encoder(obstacle_sector_array);
    AbiSendMsgFLOOR_DETECTION(ABI_FLOOR_DETECTION_ID, test_val);    // placeholder; send the result via Abi.
    //TODO: How can I log/print this?
    /// Actually, with c pprz module, you can just use printf then it will print out on the pprz center

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


enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND,
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS,
    REENTER_ARENA
};

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


