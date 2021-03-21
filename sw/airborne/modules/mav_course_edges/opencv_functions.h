/**
 * @file "modules/mav_course_edges/opencv_functions.h"
 * @author Group 3 MAV course 2021
 */

#ifndef OPENCV_FUNCTIONS_H
#define OPENCV_FUNCTIONS_H

// When using thet dataset images instead of the camera feed
#define USEDATASET 1

#if USEDATASET
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if !USEDATASET
void get_obstacles_edgebox(char *img, int w, int h);
#else
Mat get_obstacles_edgebox(Mat img, int w, int h);
#endif

#ifdef __cplusplus
}
#endif

#endif

