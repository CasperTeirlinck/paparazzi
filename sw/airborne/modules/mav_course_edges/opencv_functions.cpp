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
#include "/home/kjell/paparazzi/sw/airborne/modules/computer_vision/opencv_image_functions.h"

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
  // Original image
  Mat image;
  image = img;
  // Cropped image
  Mat image_crop;
  image_crop = image(Rect((int)(0.4*h), 0, (int)(0.6*h), w));
  // Convert to OpenCV grayscale
  Mat image_gray;
  cvtColor(image, image_gray, CV_BGR2GRAY);
  // Blurred image
  Mat image_blur;
  image.copyTo(image_blur);
  #endif

  // Get rid of the noise by blurring
  //medianBlur(image_blur, image_blur, 25);  // Blur for real pictures
  medianBlur(image_blur, image_blur, 41);   // Blur for simulation pictures
  // GaussianBlur(image_gray, image_gray, Size(5, 5), 0);

  // using canny to detect the edges of the images
  Mat image_canny;
  int edgeThresh = 100;
  // Canny(image_blur, image_canny, edgeThresh * 2, edgeThresh);   // Canny for real pictures
  Canny(image_blur, image_canny, 200, 250 );          // Canny for simulation pictures

  // Finding contours
  vector<vector<Point>> contours;
  findContours(image_canny, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

  // Getting and filtering the bounding boxes
  vector<Rect> boundRect(contours.size());
  Mat boundRect_img;
  int boundRect_area;
  int boundRect_avg;
  Mat boundRect_diff_;
  int boundRect_diff;

  vector<Rect> boundRect_obst;

  for(size_t i = 0; i < contours.size(); i++)
  {
    boundRect[i] = boundingRect(contours[i]);

    // get texture
    boundRect_area = boundRect[i].width * boundRect[i].height;
    boundRect_img = image_gray(boundRect[i]);
    boundRect_avg = sum(boundRect_img)[0]/boundRect_area;
    pow(boundRect_img - boundRect_avg, 2, boundRect_diff_);
    boundRect_diff = sum(boundRect_diff_)[0]/boundRect_area;
    cout << boundRect_diff << endl;

    // filter boxes
    if (
      ((float)boundRect_area >= (75.f/10000.f)*(w*h)) && 
      // (boundRect_diff < 100)  // bound texture for real pictures
      (boundRect_diff < 60)  // Bound texture for simulaiton pictures 
      )
    {
      boundRect_obst.insert(boundRect_obst.end(), boundRect[i]);
    }

  }

  // Draw output
  Mat drawing = Mat::zeros(image_canny.size(), CV_8UC3);
  for(size_t i = 0; i< boundRect_obst.size(); i++)
  {
    // This picks random color, change this to only red
    Scalar color = Scalar(0, 0, 255); 
    // This draws the contour
    // drawContours(drawing, contours, (int)i, color);
    // This draws the rectangle
    rectangle(image, boundRect_obst[i].tl(), boundRect_obst[i].br(), color, 2);
  }

  // Convert image back to YUV422
  #if !USEDATASET
  colorbgr_opencv_to_yuv422(image, img, w , h);
  #else
  return image_crop;
  #endif
}