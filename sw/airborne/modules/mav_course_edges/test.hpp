/**
 * This file is to test the opencv implementations on the given dataset without running paparazzi
 * ! Dont forget to set USEDATASET to 1 in opencv_function.cpp and opencv_function.h !
 */
/**
 * @file "modules/mav_course_edges/test.h"
 * @author Group 3 MAV course 2021
 */

#ifndef TEST_H
#define TEST_H

// Define setting
// to be compatible with what the module files excpect from paparazzi settings
extern float eb_hor_thresh;
extern int eb_blur_size;
extern int eb_canny_thresh_1;
extern int eb_canny_thresh_2;
extern float eb_size_thresh;
extern int eb_diff_thresh;

#endif