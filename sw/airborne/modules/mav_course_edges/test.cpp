/**
 * This file is to test the opencv implementations on the given dataset without running paparazzi
 * This file is mean to be compiled stabdalone, but still using the bepop opencv build
 * ! Dont forget to set USEDATASET to 1 in opencv_function.cpp and opencv_function.h !
 */
/**
 * @file "modules/mav_course_edges/test.cpp"
 * @author Group 3 MAV course 2021
 */

/* 
To compile use in current directory:
$ g++ test.cpp opencv_functions.cpp -o test \
  -I ~/paparazzi/sw/ext/opencv_bebop/install_pc/include \
  -I ~/paparazzi/sw/airborne/modules/computer_vision \
  -L ~/paparazzi/sw/ext/opencv_bebop/install_pc/lib \
  -lopencv_world \
  -L ~/paparazzi/sw/ext/opencv_bebop/install_pc/share/OpenCV/3rdparty/lib \
  -llibprotobuf \
  -lquirc \
  -L /usr/lib/x86_64-linux-gnu \
  -ljpeg \
  -lpng \
  -ltiff \
  -ldc1394 \
  -L /usr/lib/x86_64-linux-gnu/hdf5/serial \
  -lhdf5 \
  -lpthread \
  -lsz \
  -lz \
  -ldl \
  -lm \
  -lfreetype \
  -lharfbuzz \
  -lrt
*/

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <regex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv_functions.h"
#include "test.hpp"

using namespace std;
using namespace cv;

// #define DATA_DIR "/paparazzi/prototyping/AE4317_2019_datasets/sim_poles_panels_mats/20190121-161931/"
// #define DATA_DIR "/paparazzi/prototyping/AE4317_2019_datasets/test_set_single/"
#define DATA_DIR "/paparazzi/prototyping/AE4317_2019_datasets/cyberzoo_poles_panels_mats/20190121-142935/"
#define DATA DATA_DIR "*.jpg"

// Define settings
float eb_hor_thresh = 0.4;
int eb_blur_size = 41;
int eb_canny_thresh_1 = 200;
int eb_canny_thresh_2 = 250;
float eb_size_thresh = 75;
int eb_diff_thresh = 60;

int main()
{

  // Define frame reading iterator
  vector<String> fn;
  string data_dir = (string) getenv("HOME") + DATA_DIR;
  string data = (string) getenv("HOME") + DATA;
  glob(data, fn, false);

  // Loop trough frames
  for (size_t i=0; i<fn.size(); i++)
  {
    Mat frame = imread(fn[i]);
    if (frame.empty()) continue; // skip loop if not succesfull

    frame = get_obstacles_edgebox(frame, frame.size[1], frame.size[0]); // note: frame is rotated 90deg!

    string filename = fn[i];
    filename.replace(filename.find(data_dir), data_dir.size()-1, "");

    string filepath = (string) getenv("HOME") + "/paparazzi/prototyping/paparazzi_capture";
    imwrite(filepath+filename, frame);
  }

  return 0;
}