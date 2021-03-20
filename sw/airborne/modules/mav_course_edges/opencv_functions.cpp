/**
 * @file "modules/mav_course_edges/opencv_functions.cpp"
 * @author Group 3 MAV course 2021
 */

#include "opencv_functions.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include "opencv_image_functions.h"
#include "/home/casper/paparazzi/sw/airborne/modules/computer_vision/opencv_image_functions.h"

using namespace std;
using namespace cv;

// void edge_box(int, void*);

// When using thet dataset images instead of the camera feed
#define USEDATASET 1

// Define functions
#if !USEDATASET
void get_obstacles_edgebox(char *img, int w, int h);
#else
Mat get_obstacles_edgebox(Mat img, int w, int h);
#endif

#if !USEDATASET
void get_obstacles_edgebox(char *img, int w, int h) {
#else
Mat get_obstacles_edgebox(Mat img, int w, int h) {
#endif

  // Transform image buffer into an OpenCV YUV422 Mat
  #if !USEDATASET
  Mat M(h, w, CV_8UC2, img);
  // Convert to OpenCV BGR
  Mat image;
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // Convert to OpenCV grayscale
  Mat image_gray;
  cvtColor(M, image_gray, CV_YUV2GRAY_Y422);
  #else
  Mat image;
  image = img;
  // Convert to OpenCV grayscale
  Mat image_gray;
  cvtColor(image, image_gray, CV_BGR2GRAY);
  #endif

  // Get rid of the noise by blurring
  // blur(image_gray, image_gray, Size(3,3)); // Why example with size (5,5)??  
  GaussianBlur(image_gray, image_gray, Size(5, 5), 0);

  /*
  // using canny to detect the edges of the images
  Mat canny_image_output;
  int edgeThresh = 35;

  Canny(gray_image, canny_image_output, edgeThresh, edgeThresh * 3); // example says times 2 not 3 so why Casper?

  //Finding the countours and save them to vectors
  vector<vector<point> > contours;

  // Convert the image to an OpenCV Mat
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // Blur it, because we can
  blur(image, image, Size(5, 5));
  // Convert back to YUV422 and put it in place of the original image
  colorbgr_opencv_to_yuv422(image, img, w, h);
  */

  // Getting the bounding boxes
  
  // Implement contours with

  // Convert image back to YUV422
  #if !USEDATASET
  colorbgr_opencv_to_yuv422(image, img, w , h);
  #else
  return image_gray;
  #endif
}

/* 
 * Function to save a frame to disk for debugging
 * Referenced from video_capture.c
 */
// void save_frame(void) {
//   // Create output folder if necessary
//   if (access(save_dir, F_OK)) {
//     char save_dir_cmd[256];
//     sprintf(save_dir_cmd, "mkdir -p %s", save_dir);
//     if (system(save_dir_cmd) != 0) {
//       printf("[video_capture] Could not create images directory %s.\n", save_dir);
//       return;
//     }
//   }
// }

// void edge_box(int, void* )
// {
//   // using canny to detect the edges of the images
//   Mat canny_image_output;
//   int edgeThresh = 100;

//   Canny(gray_image, canny_image_output, edgeThresh * 2, edgeThresh); // example says times 2 not 3 so why Casper?

//   //Finding the countours and save them to vectors
//   vector<vector<Point> > contours;
//   findContours(canny_image_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
  
//   // finding contours with aprrox to polygons of accuracy 3, contour does not need to
//   // be closed so bool set to false
//   vector<vector<Point>> contours_poly( contours.size() );
//   vector<Rect> boundRect( contours.size() );
//   vector<Point2f>centers( contours.size() );
//   vector<float>radius( contours.size() );
  

//   for( size_t i = 0; i < contours.size(); i++ )
//   {
//     approxPolyDP( contours[i], contours_poly[i], 3, false );
//     boundRect[i] = boundingRect( contours_poly[i] );
//     // minEnclosingCircle( contours_poly[i], centers[i], radius[i] ); does not need to be circle
//   }
//   Mat drawing = Mat::zeros( canny_image_output.size(), CV_8UC3 );
//   for( size_t i = 0; i< contours.size(); i++ )
//   {
//     // This picks random color, change this to only red
//     Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) ); 
//     //This draws the contour with the picked color
//     drawContours( drawing, contours_poly, (int)i, color );
//     // this draws the rectangle
//     rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
//     // cirlce does not need to be drawn
//     // circle( drawing, centers[i], (int)radius[i], color, 2 );
//   }
// }

