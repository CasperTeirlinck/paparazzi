/*
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/mav_course_navigation.c"
 * @author Group 3 MAV course 2021
 *
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>
#include "modules/mav_course_edges/mav_course_edges.h"
#include "modules/mav_course_exercise/floor_detection.h"

#define MAV_COURSE_NAVIGATION_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if MAV_COURSE_NAVIGATION_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Output from edgebox obstacle detection
#ifndef MT9F002_OUTPUT_HEIGHT
#define MT9F002_OUTPUT_HEIGHT 520
#endif


// ------------------------- NAVIGATION GLOBAL VARIABLES START-------------------------------------

int len_view, width_drone, center_view;
float thresh_front, green_max;
int flag_front, flag_heading, flag_go;

int view_green[MT9F002_OUTPUT_HEIGHT], view_line[MT9F002_OUTPUT_HEIGHT];

// ------------------------- NAVIGATION GLOBAL VARIABLES END-------------------------------------

// ------------------------- ABI COMMUNICATION START-------------------------------------

#ifndef ABI_FLOOR_DETECTION_ID
#define ABI_FLOOR_DETECTION_ID ABI_BROADCAST
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id, struct FloorDetectionOutput green_detection)
{
  view_green = green_detection.obstacle_vector; // View-analysis for green-detection
//    for (int i=0;i<520;i++){
//        printf("%d, %d, \n", i, very_nice_output.obstacle_vector[i]);
//    }
}

#ifndef EDGEBOX_ID
#define EDGEBOX_ID ABI_BROADCAST
#endif
static abi_event edgebox_ev;
static void edgebox_cb(uint8_t __attribute__((unused)) sender_id, struct obstacles_t line_detection)
{
  view_line = line_detection.x; // View-analysis for line-detection
//	printf("edgebox: ");
//  for (int i = 0; i < MT9F002_OUTPUT_HEIGHT; i++) {
//		printf("%d", obstacles.x[i]);
//  }
//	printf(" \n");
}

// ------------------------- ABI COMMUNICATION END-------------------------------------

// ------------------------- NAVIGATION FUNCTIONS START-------------------------------------
// Function used to merge the view-array received from line-detection and green-detection
void view_combine(int viewrange_line[MT9F002_OUTPUT_HEIGHT], int viewrange_green[MT9F002_OUTPUT_HEIGHT]) {

  // viewrange_line is the view-array from line-detection indicating an obstacle as [1] and no obstacle as [0]
  // viewrange_green is the view-array from green-detection indicating an obstacle as a [non-zero value] and no obstacle as [0]

  int len_viewrange = len_view;       // Take length of viewfield
  int viewrange_comb[len_viewrange];  // Setup empty array for combined view
  int flag_front = 0;                 // Setup flag for if an obstruction is detected in the flightpath of the drone
  float view_max = 0.;                // Setup maximum depth seen by the drone

  for (int i = 0; i < len_viewrange; i++) { // Cycle through the green detection view to find maximum value
    if (viewrange_green[i] > view_max) {
      view_max = viewrange_green[i];
    }
  }

  green_max = view_max;

  for (int i = 0; i < len_view; i++) {  // Cycle through the view from green-detection and line-detection

    if (viewrange_line[i]){             // If line-detection detected an obstacle in column i

      if (viewrange_green[i]) {         // If both line- and green-detection return obstacle for column i, take green-distance
        viewrange_comb[i] = viewrange_green[i];
      }
      else{                             // If only line-detection returns obstacle for column i, take absolute 0 for distance
        viewrange_comb[i] = 0;
      }

    }

    else{                       // If line-detection does not return an obstacle in column i
      if (!viewrange_green) {   // If green returns no obstacle in sight, take green max-distance
        viewrange_comb[i] = view_max;
      }
      else{
      viewrange_comb[i] = viewrange_green[i]; // If green returns distance to obstacle, take green-distance
      }

    } // End of else


    if (i > (len_view/2-width_drone/2) || i < (len_view/2+width_drone/2)) { // If obstacle is in flight-path and too close, raise flag
      if(viewrange_comb[i] < thresh_front){
        flag_front = 1;
      }
    }

  } // End of i


    return viewrange_comb;
}

// Function used to evaluate optimal flight-heading based on availability of depth in a certain direction
int triangle(int viewrange[MT9F002_OUTPUT_HEIGHT]){


  int len_viewrange = len_view;               // Copy global variable to local variable
  float view_max = green_max;                 // Copy global variable to local variable
  float triang_min[len_viewrange];            // Initialize array to store minimum distance for each heading
  float triang_curmin, bord;                  // Initialize minimum value of current triangle and border-value of current index
  float width = (float)width_drone, jfloat;   // Initialize datatype-change variables
  int cent_eval, ind_eval;                    // Initialize variables for index-of-evaluation-center and index-of-evaluation


  int cent_start = width_drone/2;   // Initialize evaluation heading at left-side of view placing the left side of the drone at 0

  for (int i = 0; i < len_viewrange-width_drone; i++) { // Cycle through all possible headings, up until right side of the drone reaches right side of view

    triang_curmin = view_max;                 // Initialize current heading-minimum to be maximum value of viewfield
    bord = view_max;                          // Initialize current border-value to be maximum value of viewfield

    cent_eval = cent_start + i;               // Compute center of to-be-evaluated triangle

    for (int j = 0; j < width_drone; j++) {   // Cycle through the width of the to-be-evaluated triangle
      jfloat = (float)j;                      // Copy integer variable to float variable for calculation purposes

      if (j <= width_drone/2) {                   // If evaluation-index falls in the left-half of the evaluated triangle
        bord = view_max / (width/2.) * jfloat ;   // Compute border value limiting the evaluated range in the column
      }
      else if (j > width_drone/2) { // If evaluation-index falls in the right-half of the evaluated triangle
        bord = view_max / (width/2.) * (width - jfloat) ; // Compute border value limiting the evaluated range in the column
      }

      ind_eval = cent_eval - (width_drone/2) + j;   // Compute index within to-be-evaluated triangle

      if (viewrange[ind_eval] < bord) {             // If depth-value of column is within evaluation-triangle
        if (viewrange[ind_eval] < triang_curmin) {  // If depth-value of column is lower than thusfar received minimum-depth
          triang_curmin = viewrange[ind_eval];      // Update minimum-depth of heading-triangle
        }
      }

    } // End of j

    triang_min[i] = triang_curmin;  // Update heading-minimum in storage array of heading minima

  } // End of i

float dir_max = 0;          // Initialize maximum of evaluated headings
int target = -999;          // Initialize target heading to -999 to simulate impossible, far, out of bounds target
int center = center_view;   // Copy global variable to local variable

for (int k = 0; k < len_viewrange; k++) { // Cycle through evaluated headings
    if (triang_min[k] > dir_max) {        // If minimum depth of heading-triangle is more than thusfar seen maximum depth
      dir_max = triang_min[k];            // Update maximum depth seen so far
      target = k;                         // Update optimal heading index
    }
    else if (triang_min[k] == dir_max && abs(k - center) < abs(target - center)) {
      // If minimum depth of heading-triangle is equal to the thusfar seen maximum depth, but index is closer to center (meaning smaller cange)
      dir_max = triang_min[k];            // Update maximum depth seen so far
      target = k;                         // Update optimal heading index
    }
  }

int flag_error = 1;                     // Initialize error flag to true for out-of-bounds error
if(target > 0 && target < len_view){    // If suitable target is found within the range of view
    flag_error = 0;                     // Set error flag to false
  }

  return target;  // Return target heading and error flag
}

// ------------------------- NAVIGATION FUNCTIONS END-------------------------------------

/*
 * Initialisation function
 */
void mav_course_navigation_init(void)
{
  AbiBindMsgFLOOR_DETECTION(ABI_FLOOR_DETECTION_ID, &floor_detection_ev, floor_detection_cb);

  AbiBindMsgOBSTACLES(EDGEBOX_ID, &edgebox_ev, edgebox_cb);
}

/*
 * Periodic function
 */
void mav_course_navigation_periodic(void)
{

  // ------------------------- NAVIGATION MAIN START-------------------------------------

  // RECEIVE VIEW ARRAYS HERE
  // int flag_bottom = RICARDO
  // int angle_bottom = RICARDO


  len_view = sizeof(view_green)/sizeof(view_green[0]);  // Compute range of viewfield resolution
  width_drone = len_view*0.2;                           // Determine flightpath width, set at 20% of view
  center_view = len_view/2;                             // Determine center index of viewfield

  thresh_front = 10.; // Threshold for ostacles within flightpath, obstacles within this will trigger flag_front

  flag_front = 1;     // Initialize flag for closeby obstacle, only nullified when no obstacle is found close in flightpath
  flag_heading = 1;   // Initialize flag for heading, only nullified when optimal heading within range is found
  flag_go = 1;        // Initialize flag for go-command, only nullified when optimal in-bound heading is found

  float view_comb;  //[len_view]              // Initialize combined view-array

  float len_view_float = (float)len_view;   // Initialize datatype-change variable
  float dy_ind, dy;         // Initialize index-difference between

  float headingchange, velocity;


  view_comb = view_combine(view_line, view_green);   // Combine received view-analyses using view_combine function

  if (flag_front || flag_bottom) {  // If an error flag is triggered for either too-close object or out-of-bounds state

    velocity = 0.0;                               // Set velocity to 0
    guidance_h_set_guided_body_vel(velocity,0);   // Command drone to stop

    if (flag_bottom && (angle_bottom < 270 && angle_bottom > 90) ) {  // If out-of-bounds is triggered and out-of-bounds is in flight-direction

      headingchange  = angle_bottom;              // Set heading-change to have heading coincide with green-centroid of bottom-camera (a.k.a. playing-field)
      guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);  // Command drone to turn
    }
  }


  heading = triangle(view_comb);  // Evaluate headings to find optimal heading of drone

  if (flag_heading) {  // If heading error is flagged meaning no possible heading is found

    velocity = 0.0;                               // Set velocity to 0
    guidance_h_set_guided_body_vel(velocity,0);   // Command drone to stop

    headingchange  = 60.f;                        // Set heading-change to 60 degrees clockwise to provide new heading-possibilities
    guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);  // Command drone to turn

  }

  else{ // If no heading-error is flagged meaning a possible heading is found
    dy_ind = (float)(heading - center_view);        // Determine heading-change in number of pixels
    headingchange = dy_ind / len_view_float * 103.; // Convert heading-change in pixels to degrees (FOV of 103 degrees = horizontal pixel-width)

    guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);  // Command drone to turn

    flag_go = 0;  // Set flag-go to zero allowing the drone to continue flying forward

  }

  if (!flag_go) { // If no go-prohibiting errors are encountered

    velocity = 0.0;                               // Set velocity to low speed
    guidance_h_set_guided_body_vel(velocity,0);   // Command drone to move forward
  }


  // ------------------------- NAVIGATION MAIN END-------------------------------------
}
