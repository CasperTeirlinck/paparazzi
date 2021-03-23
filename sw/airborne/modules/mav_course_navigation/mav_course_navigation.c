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

int view_line[MT9F002_OUTPUT_HEIGHT], view_green[MT9F002_OUTPUT_HEIGHT], view_merge[MT9F002_OUTPUT_HEIGHT];
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

void view_merge(int viewrange_line[MT9F002_OUTPUT_HEIGHT], int viewrange_green[MT9F002_OUTPUT_HEIGHT]){
  for (int i = 0; i < MT9F002_OUTPUT_HEIGHT; i++) {

    if (viewrange_line[i] || viewrange_green[i]) {
      view_merge[i] = 1;
    }
    else{
      view_merge[i] = 0;
    }
  }
}

// ------------------------- NAVIGATION FUNCTIONS END-------------------------------------


int biggest_hole(){
  int vvarray[10] = {vv1,vv2,vv3,vv4,vv5,vv6,vv7,vv8,vv9,vv10};

int holesize = 0;
int biggesthole = 0;
int biggestleft = 0;
int biggestright = 0;
int leftbound;
int rightbound;

for (int i=0;i<10;i++){
  if (holesize == 0){// looking for the start of a hole, holesize of 0 means we are not in a hole
    if (vvarray[i] == 0){//it's the left bound
      leftbound = i;
      holesize = holesize + 1;
    }
    }
  else{
    if (vvarray[i] == 0){
      rightbound = i;
      holesize = holesize + 1;
      if (i==9){// if we are at the end of the data, we are also at the end of the hole
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
  headingchange =( (biggestleft + biggestright)/2.f -4.5)/4.5 * 52.f;

}
else{// if there is no safe direction, stop and turn 60 degrees
  velocity = 0.0;
  headingchange  = 60.f; //now it always turns clockwise, maybe change
}

guidance_h_set_guided_body_vel(velocity,0);
guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);

VERBOSE_PRINT("biggestleft,biggestright,biggesthole, headingchange: %d, %d, %d, %.2f\n",biggestleft,biggestright,biggesthole,headingchange);
}


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
  // ------------------------- NAVIGATION MAIN START-------------------------------------

  if (flag_bottom) {
    velocity = 0.0;
    guidance_h_set_guided_body_vel(velocity,0);

    headingchange  = angle_bottom;
    guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);

    velocity = 0.3;
    guidance_h_set_guided_body_vel(velocity,0);

  }


  // ------------------------- NAVIGATION MAIN END-------------------------------------
}
