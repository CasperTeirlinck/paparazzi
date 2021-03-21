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
void get_obstacles_edgebox(char *img, int w, int h, int show_debug);
#else
Mat get_obstacles_edgebox(Mat img, int w, int h);
#endif

#if !USEDATASET
void get_obstacles_edgebox(char *img, int w, int h, int show_debug) {
#else
Mat get_obstacles_edgebox(Mat img, int w, int h) {
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
  // GaussianBlur(image_gray, image_gray, Size(5, 5), 0);

  // using canny to detect the edges of the images
  Mat image_canny;
  Canny(image_blur, image_canny, eb_canny_thresh_1, eb_canny_thresh_2 );

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
  
    // filter boxes
    if (
      ((float)boundRect_area >= (eb_size_thresh/10000.f)*(w*h)) && 
      (boundRect_diff < eb_diff_thresh) &&
      ((boundRect[i].x + (boundRect[i].width)/2) > eb_hor_thresh*w)
      )
    {
      boundRect_obst.insert(boundRect_obst.end(), boundRect[i]);
      
    }

  }

  int obstacle[520];
  for (int i = 0; i < 520; i++)
  {
    obstacle[i] = 0;
    // cout << obstacle[i] << endl;
  }
  
  //  Prepare Draw output
  for(size_t i = 0; i< boundRect_obst.size(); i++)
  {
    // Below this the obstacles will be put into an array
    // cout << "Boundary of box left begin: " << boundRect_obst[i].y << endl;
    // cout << "Boundary of box width : "<< boundRect_obst[i].height << endl;
    // cout << "Boundary of box right end : "<< (boundRect_obst[i].y + boundRect[i].height) << endl;
    // CASPER we need to write something in the style of if boundRect_obst = true then
    // boundRect_obst[i].y = 1 
    
    int indexer_begin = boundRect_obst[i].y;
    int indexer_end = (boundRect_obst[i].y + boundRect_obst[i].height ) ;

    for (int j = indexer_begin; j < (indexer_end + 1); j++)
    {
      obstacle[j] = 1;
      // cout << obstacle[j];
    }
  

    // everything below this is for drawing the boxes and lines on the figures for testing
    
    // This picks random color, change this to only red
    Scalar color = Scalar(0, 0, 255); 
    // This draws the rectangle
    rectangle(image, boundRect_obst[i].tl(), boundRect_obst[i].br(), color, 2);
    // Point point1 = Point(0.4*w, 0);
    // Point point2 = Point(0.4*w, h);
    // This shows the lines of the centers of the bounding boxes
    // Point point3 = Point((boundRect_obst[i].x + (boundRect_obst[i].width)/2), 0);
    // Point point4 = Point((boundRect_obst[i].x + (boundRect_obst[i].width)/2),h);
    // This shows the horizontal filter line
    // line(image, point1, point2, Scalar(0,0,255), 1);
    // line(image, point3, point4, Scalar(255,0,0), 2);
  }
  
  // for (int k =0; k < 520; k++)
  // {
  //   cout << obstacle[k]; 
  // }

  // cout << "   " << endl; 
  // cout << "   " << endl; 

  #if !USEDATASET
  // Convert image back to YUV422 and replace the frame buffer when debugging
  if (show_debug) {
    colorbgr_opencv_to_yuv422(image, img, w, h);
  }
  #else
  return image;
  #endif
}