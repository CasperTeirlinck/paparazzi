/* 
 * This file contains the OpenCV functions used in the mav_course_edges paparazzi module.
 * 
 */
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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_image_functions.h"

using namespace std;
using namespace cv;

// When using thet dataset images instead of the camera feed
#define USEDATASET 0

#if !USEDATASET
// include for acceccing the module settings
#include "mav_course_edges.h"
#else
#include "test.hpp"
#endif

// Define functions
#if !USEDATASET
void get_obstacles_edgebox(char *img, int w, int h, struct obstacles_t *obstacles, int show_debug);
#else
Mat get_obstacles_edgebox(Mat img, int w, int h, struct obstacles_t *obstacles, int show_debug);
#endif

/* 
 * Function to process frame with OpenCV and detect/calculate obstacle positions
 */
#if !USEDATASET
void get_obstacles_edgebox(char *img, int w, int h, struct obstacles_t *obstacles, int show_debug) {
#else
Mat get_obstacles_edgebox(Mat img, int w, int h, struct obstacles_t *obstacles, int show_debug) {
#endif

  // Transform image buffer into an OpenCV YUV422 Mat
  #if !USEDATASET
  // Original image
  Mat M(h, w, CV_8UC2, img);
  // Convert to OpenCV BGR
  Mat image;
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // Convert to OpenCV grayscale
  Mat image_gray;
  cvtColor(M, image_gray, CV_YUV2GRAY_Y422);
  // Blurred image
  Mat image_blur;
  image.copyTo(image_blur);
  #else
  // Original image
  Mat image;
  image = img;
  // Convert to OpenCV grayscale
  Mat image_gray;
  cvtColor(image, image_gray, CV_BGR2GRAY);
  // Blurred image
  Mat image_blur;
  image.copyTo(image_blur);
  #endif

  // Get rid of the noise by blurring
  medianBlur(image_blur, image_blur, eb_blur_size);

  // Using canny to detect the edges of the images
  Mat image_canny;
  Canny(image_blur, image_canny, eb_canny_thresh_1, eb_canny_thresh_2 );

  // Finding contours
  vector<vector<Point>> contours;
  findContours(image_canny, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

  // Defining bounding boxes variables
  vector<Rect> boundRect(contours.size());
  Mat boundRect_img;
  int boundRect_area;
  int boundRect_avg;
  Mat boundRect_diff_;
  int boundRect_diff;

  // Clear obstacles array
  for (int i = 0; i < h; i++) {
    obstacles->x[i] = 0;
  }

  // Getting and filtering the bounding boxes
  for(size_t i = 0; i < contours.size(); i++)
  {
    // get bounding box
    boundRect[i] = boundingRect(contours[i]);
    
    // get texture
    boundRect_area = boundRect[i].width * boundRect[i].height;
    boundRect_img = image_gray(boundRect[i]);
    boundRect_avg = sum(boundRect_img)[0]/boundRect_area;
    // pow(boundRect_img - boundRect_avg, 2, boundRect_diff_);
    boundRect_diff_ = abs(boundRect_img - boundRect_avg);
    boundRect_diff = sum(boundRect_diff_)[0]/boundRect_area;

    // show all boxes and texture value
    if (show_debug) {
      Scalar text_color = Scalar(255, 0, 0);
      if (boundRect_diff < eb_diff_thresh) {
        text_color = Scalar(0, 0, 255);
      }
      rectangle(image, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 0, 0), 1);
      Point text_center = (boundRect[i].br() + boundRect[i].tl())*0.5;
      String text = to_string(boundRect_diff);
      int text_baseline = 0;
      Size text_size = getTextSize(text, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, 1, &text_baseline);
      text_center.x = text_center.x - text_size.width/2;
      text_center.y = text_center.y + text_size.height/2;
      putText(image, text, text_center, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, text_color);

      // This shows the lines of the centers of the bounding boxes
      // Point point1 = Point(eb_hor_thresh*w, 0);
      // Point point2 = Point(eb_hor_thresh*w, h);
      // Point point3 = Point((boundRect[i].x + (boundRect[i].width)/2), 0);
      // Point point4 = Point((boundRect[i].x + (boundRect[i].width)/2), h);
      // This shows the horizontal filter line
      // line(image, point1, point2, Scalar(0,0,255), 3);
      // line(image, point3, point4, Scalar(255,0,0), 2);
    }

    // filter boxes
    if (
      // filter on size
      ((float)boundRect_area >= (eb_size_thresh/10000.f)*(w*h)) && 
      // filter on texture
      (boundRect_diff < eb_diff_thresh) &&
      // filter on height
      boundRect[i].width > ((int) w * 0.2) &&
      // filter on horizontal position
      ((boundRect[i].x + (boundRect[i].width)/2) > eb_hor_thresh*w)
      )
    {
      // fill obstacles array
      int indexer_begin = boundRect[i].y;
      int indexer_end = (boundRect[i].y + boundRect[i].height ) ;
      for (int j = indexer_begin; j < (indexer_end + 1); j++)
      {
        obstacles->x[j] = 1;
      }

      // show box
      if (show_debug) {
        rectangle(image, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 255), 2);        
      }
    }
  }

  // Show obstacles array on the bottom the image
  if (show_debug) {
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < 20; x++) {
        Vec3b & color = image.at<Vec3b>(y, x);
        if (obstacles->x[y]) {
          color[0] = 0;
          color[1] = 0;
          color[2] = 255;
        } else {
          color[0] = 0;
          color[1] = 0;
          color[2] = 0;
        }
      }
    }
  }

  #if !USEDATASET
  // Convert image back to YUV422 and replace the frame buffer when debugging
  if (show_debug) {
    colorbgr_opencv_to_yuv422(image, img, w, h);
  }
  #else
  return image;
  #endif
}