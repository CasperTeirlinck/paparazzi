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
float velocity = 0.0;
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
float integrated_velocity = 0;
float loop_number = 1.0;
float average_velocity;
int noise_threshold = 4;
int consecutive_zero_velocity = 0;
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
  edge_detection_message_number++;



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

}

/*
 * Periodic function
 */


void mav_course_navigation_periodic(void)
{
	// Only run the module if we are in the correct flight mode
	  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
	    return;
	  }



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
			int nothing_ahead = 1;
			int noise_counter = 0;
			int number_of_threats_ahead = 0;

			// analyze data
			// find biggest free space (hole) in vision
			for (int i=0;i<MT9F002_OUTPUT_HEIGHT;i++){
			  if (holesize == 0){// looking for the start of a hole, holesize of 0 means we are not in a hole
				if ((view_green[i] == 0 || view_green[i] > 30) && view_line[i] == 0){//it's the left bound
				  leftbound = i;
				  holesize = 1;
				  noise_counter = 0; //the column is safe so reset noise counter
				}
				}
			  else{

				if ((view_green[i] == 0 || view_green[i] > 25) && view_line[i] == 0){// if the column is safe
				  rightbound = i;
				  holesize = rightbound-leftbound;
				  noise_counter = 0; //the column is safe so reset noise counter
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
					if (holesize>0){// only increase noise counter if we are counting the size of a hole
						noise_counter++;// run counter to check if it's just noise
					}
					if (noise_counter > noise_threshold){// if more than noise_threshold adjacent columns are unsafe
						if (holesize > biggesthole){//found the new biggest hole
							biggesthole = holesize;
							biggestleft = leftbound;
							biggestright = rightbound;
						  }
						holesize = 0; //reset holesize
						noise_counter = 0;
					}
					else{
						if (i==MT9F002_OUTPUT_HEIGHT-1){// if we are at the end of the data, and noise counter is below limit

							rightbound = i;
							holesize = rightbound - leftbound;
									if (holesize > biggesthole){//found the new biggest hole
											biggesthole=holesize;
											biggestleft = leftbound;
											biggestright = rightbound;
										  }

										}
					}
				}
			  }


			  if (i>0.35*MT9F002_OUTPUT_HEIGHT && i<0.65*MT9F002_OUTPUT_HEIGHT && view_green[i]<10 && view_green[i]>0){
				  number_of_threats_ahead++;

			  }
			  if(i>0.3*MT9F002_OUTPUT_HEIGHT && i<0.7*MT9F002_OUTPUT_HEIGHT && view_green[i]>0){
				  nothing_ahead = 0;
			  }
//			  VERBOSE_PRINT("index %d, view_green %d, holesize %d, bounds %d %d, noise_counter %d\n",i,view_green[i], holesize, leftbound,rightbound,noise_counter);
			}
//
			if (number_of_threats_ahead > 20){
				 close_obstacle = 1;
			}

			if (velocity<0.1){
							consecutive_zero_velocity++;
						}
						else{
							consecutive_zero_velocity=0;
						}

			//choose decision
			if (biggesthole > 0.1*MT9F002_OUTPUT_HEIGHT && consecutive_zero_velocity<20){ //if there is a safe direction, turn to that direction

			  headingchange = (float)(0.5*(biggestleft+biggestright) - 259.5)/259.5 * 51.5;
			  if (abs(headingchange) > 40 || close_obstacle){//set velocity to zero if the heading change is too big or if there is an obstacle ahead
				  velocity = 0.0;

			  }
			  else if(abs(headingchange)>10){
				  velocity = 0.3;
			  }
			  else if (abs(headingchange)<5 && nothing_ahead){
				  velocity = 1.0;
			  }
			  else{
				  velocity = 0.5;
			  			  }

			}
			else{// if there is no safe direction, stop and turn 60 degrees
			  velocity = 0.0;
			  headingchange  = 60.f; //always turn in the same direction to avoid getting stuck in going back and forth
			  if (consecutive_zero_velocity>=20){
				  VERBOSE_PRINT("Turning because we're stuck");
			  }
			  consecutive_zero_velocity = 0;


			}
			if (close_obstacle){
				VERBOSE_PRINT("\nCLOSE OBSTACLE, setting velocity to zero\n");
			}




			//execute decision
			guidance_h_set_guided_body_vel(velocity,0);
			guidance_h_set_guided_heading(RadOfDeg(headingchange)+stateGetNedToBodyEulers_f()->psi);

//			integrated_velocity = integrated_velocity + velocity;
//			average_velocity = integrated_velocity/loop_number;
//			loop_number += 1.0f;
//			VERBOSE_PRINT("average velocity: %.1f\n",average_velocity);

			VERBOSE_PRINT("left %d, right %d, biggesthole %d, location %.2f, headingchange %.2f, velocity %.2f\n",biggestleft,biggestright,biggesthole,(float)(0.5*(biggestleft+biggestright) - 259.5)/259.5,headingchange,velocity);
			}
//		}//end else statement



}

