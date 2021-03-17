/**
 * @file "modules/mav_course_edges/opencv_functions.cpp"
 * @author C. Teirlinck
 */

// #include "opencv_functions.h"
#include "modules/mav_course_edges/opencv_functions.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

int opencv_example(char *img, int width, int height)
{
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

  return 0;
}
