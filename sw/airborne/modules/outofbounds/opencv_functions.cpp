//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include "opencv_functions.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_image_functions.h"

#define PI 3.14159265

using namespace cv;
using namespace std;

float result;

vector<vector<int>> mySimColors{ {0, 120, 0, 255, 255, 120}, // blue sim
								{0, 106, 99, 115, 145, 166}, // black sim
								{62, 69, 70, 124, 110, 155} }; //green sim
													

vector<vector<int>> myRealColors{ {148, 127, 59, 255, 163, 127}, // blue real
								{0, 90, 96, 143, 145, 174}, // black real
								{141, 82, 93, 207, 129, 154} }; //green real

//vector<Scalar> myColorValues{ {255, 0, 0}, {0, 0, 0}, {0, 255, 0} };

int getContours(Mat imgMask, int j, bool val, float w, float h) {
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	int aux = 0;

	findContours(imgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	vector<Point> conPoly;
	Rect boundRect;
	Point myPoint(0, 0);

	int max_area = 0;
	vector<Point> final_contour;

	for (int i = 0; i < contours.size(); i++) {

		int area = contourArea(contours[i]);

		if (area > max_area) {
			final_contour = contours[i];
			max_area = area;
		}
	}

	string objectType;

	if ((max_area > ((w * h) / 6)) || ((max_area > 0) && (val))){

		aux = 1;
		
		float peri = arcLength(final_contour, true);
		approxPolyDP(final_contour, conPoly, 0.02 * peri, true);

		boundRect = boundingRect(conPoly);

		}

	if ((max_area > 0) && (val)) {
		float dx;
		float dy;

		myPoint.x = boundRect.x + boundRect.width / 2;
		myPoint.y = boundRect.y + boundRect.height / 2;

		if (myPoint.y <= (h / 2)) {
			dx = myPoint.x - (w / 2);
			dy = (h / 2) - myPoint.y;
			if (myPoint.x >= (w / 2)){
			result = -90 + (atan(dy / dx) * (180 / PI));}
			else{
			result = 90 - (atan(dy / -dx) * (180 / PI));}
		}
		else {
			dx = myPoint.x - (w / 2);
			dy = myPoint.y - (h / 2);
			if (myPoint.x >= (w / 2)){
			result = -90 - (atan(dy / dx) * (180 / PI));}
			else{
			result = 90 + (atan(dy / -dx) * (180 / PI));}
		}
	}
	return aux;
}


float findColor(char *img, int w, int h, int color) {

	vector<vector<int>> myColors(myRealColors.size());
	vector<int> danger(3);
	bool val = false;
	result = 0;
	
	Mat M(h, w, CV_8UC2, img);
	
	// convert UYVY in paparazzi to YUV in opencv
  	cvtColor(M, M, CV_YUV2RGB_Y422);
  	cvtColor(M, M, CV_RGB2YUV);
	
	if (color == 0) {
		myColors = mySimColors;
	}
	else {
		myColors = myRealColors;
	}

	for (int i = 0; i < myColors.size(); i++) {
		if (i == 2) {
			if ((danger[0] == 1) || (danger[1] == 1)) {
				val = true;
			}
		}
		Scalar lower(myColors[i][0], myColors[i][1], myColors[i][2]);
		Scalar upper(myColors[i][3], myColors[i][4], myColors[i][5]);
		// print(lower);
		Mat mask;
		inRange(M, lower, upper, mask);
		danger[i] = getContours(mask, i, val, (float)w, (float)h);
	}
	// cout << "Result: " << result << endl;
	return result;
}
