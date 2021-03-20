/**
 * This file is to test the opencv implementations on the given dataset
 * This file is mean to be compiled stabdalone, but still using the bepop opencv build
 * ! Dont forget to set USEDATASET to 1 in opencv_function.cpp and opencv_function.h !
 */
/**
 * @file "modules/mav_course_edges/test.cpp"
 * @author Group 3 MAV course 2021
 */

/* 
To compile use:
$ g++ test.cpp opencv_functions.cpp -o test \
  -I ~/paparazzi/sw/ext/opencv_bebop/install_pc/include \
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
// #include "lib/encoding/jpeg.h"

using namespace std;
using namespace cv;

#define PAPARAZZI_SRC "~/paparazzi"
#define DATA_DIR PAPARAZZI_SRC "/prototyping/AE4317_2019_datasets/sim_poles_panels_mats/20190121-161931/"
#define DATA DATA_DIR "*.jpg"

// Save directory
static char save_dir[256];

int main()
{
  // Set frame output save path
  // sprintf(save_dir, "~/paparazzi/prototyping/paparazzi_capture");

  // Define frame reading iterator
  vector<String> fn;
  glob(DATA, fn, false);

  // Loop trough frames
  for (size_t i=0; i<fn.size(); i++)
  {
    Mat frame = imread(fn[i]);
    if (frame.empty()) continue; // skip loop if not succesfull

    frame = get_obstacles_edgebox(frame, frame.size[0], frame.size[1]);

    string filename = fn[i];
    filename.replace(filename.find(DATA_DIR), sizeof(DATA_DIR)-1, "");

    imwrite("~/paparazzi/prototyping/paparazzi_capture/"+filename, frame);
  }


  return 0;
}