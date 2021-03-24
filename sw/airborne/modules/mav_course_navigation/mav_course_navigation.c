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

int green_detection_message_number = 1;
int out_of_bounds_message_number = 1;
int edge_detection_message_number = 1;
int latest_green_detection_message = 0;
int latest_out_of_bounds_message = 0;
int latest_edge_detection_message = 0;


// ------------------------- NAVIGATION GLOBAL VARIABLES END-------------------------------------


// ------------------------- ABI COMMUNICATION START-------------------------------------

#ifndef ABI_FLOOR_DETECTION_ID
#define ABI_FLOOR_DETECTION_ID ABI_BROADCAST
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id, struct FloorDetectionOutput green_detection)
{
	printf("View green:");
  for (int i = 0; i < MT9F002_OUTPUT_HEIGHT; i++) {
      view_green[i] = green_detection.obstacle_vector[i]; // View-analysis for green-detection
      if (i % 50 == 0)
          		printf(" %d", view_green[i]);
  }
	printf("\n");
//  printf("new green detection message %d\n",green_detection_message_number);
  green_detection_message_number++;

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
//  printf("Edge detection message %d\n",edge_detection_message_number);
  edge_detection_message_number++;



}

#ifndef OUTOFBOUNDS_ID
#define OUTOFBOUNDS_ID ABI_BROADCAST
#endif
static abi_event outofbounds_ev;
static void outofbounds_cb(uint8_t __attribute__((unused)) sender_id, float angle_rec)
{
  angle_bottom = angle_rec;
  if (angle_bottom>0.1 || angle_bottom<-0.1){
  printf("Out of bounds detection message %d, angle %f \n",out_of_bounds_message_number,angle_bottom);
  }
  out_of_bounds_message_number++;

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
	int number_of_obstructed_columns = 0;

	for (int i = 0; i < MT9F002_OUTPUT_HEIGHT; i++) {
		  if (view_line[i]){
			  printf("%d ",i);
		  number_of_obstructed_columns++;
		  }
	  }
	if (number_of_obstructed_columns>0){
					printf("\n Line detection detects %d obstructed columns\n", number_of_obstructed_columns);
				}

	// stop if bottom camera indicates out of bounds
	// run this even if no new green_detection_message
	if (angle_bottom > 90 && angle_bottom < 270 && out_of_bounds_message_number > latest_out_of_bounds_message){
		latest_out_of_bounds_message = out_of_bounds_message_number;
				velocity = 0.0;
				headingchange = angle_bottom; //Could be minus
//				guidance_h_set_guided_body_vel(velocity,0);
//				guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);
				VERBOSE_PRINT("OUT OF BOUNDS DETECTED, would have turned %.2f degrees\n",headingchange);

	}

//	else{ //if no out of bounds detected
		if (green_detection_message_number>latest_green_detection_message && edge_detection_message_number>latest_edge_detection_message){ // to prevent changing the heading multiple times for the same message
			latest_green_detection_message = green_detection_message_number;
			latest_edge_detection_message = edge_detection_message_number;
			int holesize = 0;
			int biggesthole = 0;
			int biggestleft = 0;
			int biggestright = 0;
			int leftbound;
			int rightbound;
			int close_obstacle = 0;

			for (int i=0;i<MT9F002_OUTPUT_HEIGHT;i++){
			  if (holesize == 0){// looking for the start of a hole, holesize of 0 means we are not in a hole
				if ((view_green[i] == 0 || view_green[i] > 30) && view_line[i] == 0){//it's the left bound
				  leftbound = i;
				  holesize = holesize + 1;
				}
				}
			  else{
				if ((view_green[i] == 0 || view_green[i] > 30) && view_line[i] == 0){
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
			  if (i>0.4*MT9F002_OUTPUT_HEIGHT && i<0.6*MT9F002_OUTPUT_HEIGHT && view_green[i]<5 && view_green[i]>0){
				  close_obstacle = 1;

			  }
			}


			if (biggesthole > 0.06*MT9F002_OUTPUT_HEIGHT && !close_obstacle){ //if there is a safe direction, turn to that direction

			  headingchange = (float)(0.5*(biggestleft+biggestright) - 259.5)/259.5 * 51.5;
			  if (abs(headingchange) > 40){
				  velocity = 0.0;
			  }
			  else{
				  velocity = 0.3;
			  }

			}
			else{// if there is no safe direction, stop and turn 60 degrees
				if (close_obstacle){
					VERBOSE_PRINT("CLOSE OBSTACLE, setting velocity to zero\n");
				}
			  velocity = 0.0;
			  headingchange  = 60.f; //now it always turns clockwise, maybe change
			}






			guidance_h_set_guided_body_vel(velocity,0);
			guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);

			VERBOSE_PRINT("biggesthole %d, location %.2f, headingchange %.2f, velocity %.2f\n",biggesthole,(float)(0.5*(biggestleft+biggestright) - 259.5)/259.5,headingchange,velocity);
			}
//		}//end else statement



}

