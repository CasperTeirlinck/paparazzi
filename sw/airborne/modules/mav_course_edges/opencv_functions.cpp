/**
 * @file "modules/mav_course_edges/opencv_functions.cpp"
 * @author C. Teirlinck
 */


// casper i propose to first try it on an image from datastet tbh

// #include "opencv_functions.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "modules/mav_course_edges/opencv_functions.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// check which ones are the right ones.
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;
#include "opencv_image_functions.h"

int opencv_example(char *img, int width, int height){
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;

  // #if OPENCVDEMO_GRAYSCALE
  //   //  Grayscale image example
  //   cvtColor(M, image, CV_YUV2GRAY_Y422);
  //   // Canny edges, only works with grayscale image
  //   int edgeThresh = 35;
  //   Canny(image, image, edgeThresh, edgeThresh * 3);
  //   // Convert back to YUV422, and put it in place of the original image
  //   grayscale_opencv_to_yuv422(image, img, width, height);
  // #else // OPENCVDEMO_GRAYSCALE
  //   // Color image example
  //   // Convert the image to an OpenCV Mat
  //   cvtColor(M, image, CV_YUV2BGR_Y422);
  //   // Blur it, because we can
  //   blur(image, image, Size(5, 5));
  //   // Convert back to YUV422 and put it in place of the original image
  //   colorbgr_opencv_to_yuv422(image, img, width, height);
  // #endif // OPENCVDEMO_GRAYSCALE

  // Convert the image to an OpenCV Mat
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // Blur it, because we can
  blur(image, image, Size(5, 5));
  // Convert back to YUV422 and put it in place of the original image
  colorbgr_opencv_to_yuv422(image, img, width, height);

  //return 0;
}

void edge_box(int, void*);

// Kjell part 
int main(char *img, int width, int height){
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;

  // convert image to the gray scale image
  //then get rid of the noise by bluring
  Mat grag_image;
  cvtColor(Image, gray_image, COLOR_BGR2GRay);  // can also try CV_YUV2GRAY_Y422);
  blur(gray_image, gray_image, size(3,3)); // Why example with size (5,5)??  
  
  // Create a window, with header
  const char* source_window = "Header";
  namedWindow ( source_window);
  imshow( source_window, image);

  // create trackbar on the source window
  const int max_thresh = 255;
  createTrackbar("Canny tresh:" , source_window, &edgeThresh, max_thresh, edge_box);
  edge_box(0,0);

  /*
  // using canny to detect the edges of the images
  Mat canny_image_output;
  int edgeTresh = 35;

  Canny(gray_image, canny_image_output, edgeTresh, edgeTresh * 3); // example says times 2 not 3 so why Casper?

  //Finding the countours and save them to vectors
  vector<vector<point> > contours;

  // Convert the image to an OpenCV Mat
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // Blur it, because we can
  blur(image, image, Size(5, 5));
  // Convert back to YUV422 and put it in place of the original image
  colorbgr_opencv_to_yuv422(image, img, width, height);
  */

  // Getting the bounding boxes
  
  // Implement contours with 

  waitKey();
  return 0;
}

void edge_box(int, void* )
{
  // using canny to detect the edges of the images
  Mat canny_image_output;
  int edgeThresh = 35;

  Canny(gray_image, canny_image_output, edgeTresh, edgeTresh * 3); // example says times 2 not 3 so why Casper?

  //Finding the countours and save them to vectors
  vector<vector<Point> > contours;
  findContours(canny_image_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
  
  // finding contours with aprrox to polygons of accuracy 3, contour does not need to
  // be closed so bool set to false
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>centers( contours.size() );
  vector<float>radius( contours.size() );
  

  for( size_t i = 0; i < contours.size(); i++ )
  {
    approxPolyDP( contours[i], contours_poly[i], 3, false );
    boundRect[i] = boundingRect( contours_poly[i] );
    // minEnclosingCircle( contours_poly[i], centers[i], radius[i] ); does not need to be circle
  }
  Mat drawing = Mat::zeros( canny_image_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
  {
    // This picks random color, change this to only red
    Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) ); 
    //This draws the contour with the picked color
    drawContours( drawing, contours_poly, (int)i, color );
    // this draws the rectangle
    rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
    // cirlce does not need to be drawn
    // circle( drawing, centers[i], (int)radius[i], color, 2 );
  }
  imshow( "Contours", drawing );
}

/*
int main(){

  return 0;
}
*/

