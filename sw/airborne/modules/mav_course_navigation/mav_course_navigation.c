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
int thresh_front, green_max;
int flag_front, flag_heading, flag_go;
float velocity;
float headingchange;
int view_green[MT9F002_OUTPUT_HEIGHT];
int view_line[MT9F002_OUTPUT_HEIGHT];
float angle_bottom;



// ------------------------- NAVIGATION GLOBAL VARIABLES END-------------------------------------


// ------------------------- ABI COMMUNICATION START-------------------------------------

#ifndef ABI_FLOOR_DETECTION_ID
#define ABI_FLOOR_DETECTION_ID ABI_BROADCAST
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id, struct FloorDetectionOutput green_detection)
{
  for (int i = 0; i < MT9F002_OUTPUT_HEIGHT; i++) {
      view_green[i] = green_detection.obstacle_vector[i]; // View-analysis for green-detection
  }

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
  for (int i = 0; i < MT9F002_OUTPUT_HEIGHT; i++) {
      view_line[i] = line_detection.x[i]; // View-analysis for line-detection
  }

//	printf("edgebox: ");
//  for (int i = 0; i < MT9F002_OUTPUT_HEIGHT; i++) {
//		printf("%d", obstacles.x[i]);
//  }
//	printf(" \n");
}

#ifndef OUTOFBOUNDS_ID
#define OUTOFBOUNDS_ID ABI_BROADCAST
#endif
static abi_event outofbounds_ev;
static void outofbounds_cb(uint8_t __attribute__((unused)) sender_id, float angle_rec)
{
  angle_bottom = angle_rec - 90;
}

// ------------------------- ABI COMMUNICATION END-------------------------------------

// ------------------------- NAVIGATION FUNCTIONS START-------------------------------------




/*
 * Initialisation function
 */
void mav_course_navigation_init(void)
{
  AbiBindMsgFLOOR_DETECTION(ABI_FLOOR_DETECTION_ID, &floor_detection_ev, floor_detection_cb);

  AbiBindMsgOBSTACLES(EDGEBOX_ID, &edgebox_ev, edgebox_cb);

  AbiBindMsgOUTOFBOUNDS(OUTOFBOUNDS_ID, &outofbounds_ev, outofbounds_cb);
}

/*
 * Periodic function
 */


void mav_course_navigation_periodic(void)
{

	int holesize = 0;
	int biggesthole = 0;
	int biggestleft = 0;
	int biggestright = 0;
	int leftbound;
	int rightbound;

	for (int i=0;i<MT9F002_OUTPUT_HEIGHT;i++){
	  if (holesize == 0){// looking for the start of a hole, holesize of 0 means we are not in a hole
	    if (view_green[i] == 0 && view_line[i] == 0){//it's the left bound
	      leftbound = i;
	      holesize = holesize + 1;
	    }
	    }
	  else{
	    if (view_green[i] == 0 && view_line[i] == 0){
	      rightbound = i;
	      holesize = holesize + 1;
	      if (i==MT9F002_OUTPUT_HEIGHT-1){// if we are at the end of the data, we are also at the end of the hole
	        if (holesize > biggesthole){//found the new biggest hole
	                biggesthole=holesize;
	                biggestleft = leftbound;
	                biggestright = rightbound;
	              }
	              holesize = 0; //reset holesize
	    }
	  }
	    else{// found the end of a hole
	      if (holesize > biggesthole){//found the new biggest hole
	        biggesthole=holesize;
	        biggestleft = leftbound;
	        biggestright = rightbound;
	      }
	      holesize = 0; //reset holesize
	    }
	  }
	}


	if (biggesthole > 0){ //if there is a safe direction, turn to that direction
	  velocity = 0.3;
	  headingchange =( (biggestleft + biggestright)/2.f -MT9F002_OUTPUT_HEIGHT/2.)/(MT9F002_OUTPUT_HEIGHT/2.) * 52.f;

	}
	else{// if there is no safe direction, stop and turn 60 degrees
	  velocity = 0.0;
	  headingchange  = 60.f; //now it always turns clockwise, maybe change
	}


	guidance_h_set_guided_body_vel(velocity,0);
	guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);

	VERBOSE_PRINT("biggestleft %d,biggestright %d,biggesthole %d, headingchange %.2f, velocity %.2f\n",biggestleft,biggestright,biggesthole,headingchange,velocity);

}

//  // ------------------------- NAVIGATION MAIN START-------------------------------------
//
//  // RECEIVE VIEW ARRAYS HERE
//  int flag_bottom = 0;
//  float angle_bottom = 10;
//
//
//  len_view = MT9F002_OUTPUT_HEIGHT;   // sizeof(view_green)/sizeof(view_green[0]);  // Compute range of viewfield resolution
//  width_drone = len_view*0.1;                           // Determine flightpath width, set at 20% of view
//  center_view = len_view/2;                             // Determine center index of viewfield
//
//  thresh_front = 10; // Threshold for ostacles within flightpath, obstacles within this will trigger flag_front
//
//  flag_front = 1;     // Initialize flag for closeby obstacle, only nullified when no obstacle is found close in flightpath (1 means issue)
//  flag_heading = 1;   // Initialize flag for heading, only nullified when optimal heading within range is found (1 means issue)
//  flag_go = 1;        // Initialize flag for go-command, only nullified when optimal in-bound heading is found (1 means issue)
//
//  int view_comb;  // Initialize combined view-array
//  int heading;   // Initialize heading to be computed by optimality algorithm
//
//  float len_view_float = (float)len_view;   // Initialize datatype-change variable
//  float dy_ind, dy;         // Initialize index-difference between
//
//  float headingchange, velocity;
//
//
//  view_combine(view_line, view_green);   // Combine received view-analyses using view_combine function
//
//  if (flag_front != 0 || flag_bottom != 0) {  // If an error flag is triggered for either too-close object or out-of-bounds state
//
//    velocity = 0.0;                               // Set velocity to 0
//    guidance_h_set_guided_body_vel(velocity,0);   // Command drone to stop
//
//    if (flag_bottom && (angle_bottom < 270 && angle_bottom > 90) ) {  // If out-of-bounds is triggered and out-of-bounds is in flight-direction
//
//      headingchange  = angle_bottom;              // Set heading-change to have heading coincide with green-centroid of bottom-camera (a.k.a. playing-field)
//      guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);  // Command drone to turn
//    }
//  }
//
//
//  heading = triangle(view_comb);  // Evaluate headings to find optimal heading of drone
//
//  if (flag_heading) {  // If heading error is flagged meaning no possible heading is found
//
//    velocity = 0.0;                               // Set velocity to 0
//    guidance_h_set_guided_body_vel(velocity,0);   // Command drone to stop
//
//    headingchange  = 60.f;                        // Set heading-change to 60 degrees clockwise to provide new heading-possibilities
//    guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);  // Command drone to turn
//
//  }
//  else{ // If no heading-error is flagged meaning a possible heading is found
//    dy_ind = (float)(heading - center_view);        // Determine heading-change in number of pixels
//    headingchange = dy_ind / len_view_float * 103.; // Convert heading-change in pixels to degrees (FOV of 103 degrees = 520 horizontal pixel-width)
//
//    guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);  // Command drone to turn
//
//    flag_go = 0;  // Set flag-go to zero allowing the drone to continue flying forward
//
//  }
//
//  if (!flag_go) { // If no go-prohibiting errors are encountered
//
//    velocity = 0.3;                               // Set velocity to low speed
//    guidance_h_set_guided_body_vel(velocity,0);   // Command drone to move forward
//  }
//
//
//  // ------------------------- NAVIGATION MAIN END-------------------------------------

