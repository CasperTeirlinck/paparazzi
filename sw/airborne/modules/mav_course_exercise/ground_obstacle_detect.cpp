//TODO: Do we need to include all of these?
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;


/// <summary>
/// Bounds num between min and max.
/// </summary>
/// <param name="num"></param>
/// <param name="min"></param>
/// <param name="max"></param>
/// <returns></returns>
int bound_int(int num, int min, int max) {
    int out;
    if (num < min) {
        out = min;
    }
    else if (num > max) {
        out = max;
    }
    else {
        out = num;
    }
    return out;
}



//TODO: Is this correct?
// Only checked it on my windows laptop with the sample images; still needs to be tested in paparazzi

/// <summary>
/// Checks the bottom_count of pixels at the bottom of each column if they are white.
/// If more than certainty many black pixels are found at the bottom of the column, it is considered unsafe.
/// </summary>
/// <param name="img"> BW colorfiltered image in openCV::Mat </param>
/// <param name="bottom_count"> The width of the band on the bottom to scan for black </param>
/// <param name="certainty"> The number of black pixels in a column of the bottom_count band, that makes that direction unsafe. </param>
/// <returns> Array with indexes representing the column index, and values: 1 is safe, 2 is obstacle, 0 is outside of frame </returns>
int* ground_obstacle_detect(Mat img, int safe_vector[], int bottom_count = 20, int certainty = 1) {
    // (0, 0) is top left corner
    int threat;

    for (int col = 0; col < img.cols; col=col+1) {
        threat = 0;
        for (int row = int(img.rows) - 1; row >= 0; row--) {
            if (int(img.at<uchar>(row, col)) == 0) {
                if (row >= int(img.rows) - bottom_count) {
                    threat++;
                }

            } else if (int(img.at<uchar>(row, col)) == 255) {

                threat--;

            } else {
                cout << "wrong" << endl;
                cout << img.at<uchar>(row, col) << endl;
                break;
            }

            threat = bound_int(threat, 0, certainty);

            if (threat == certainty) {
                safe_vector[col] = 2;
            }
            else if (threat == 0) {
                safe_vector[col] = 1;
            }
        }

    }
    return safe_vector;
}