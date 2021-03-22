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

int vv1;
int vv2;
int vv3;
int vv4;
int vv5;
int vv6;
int vv7;
int vv8;
int vv9;
int vv10;

/* 
 * Define global variables
 */
enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND,
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS,
    REENTER_ARENA
};
float headingchange;
float velocity;

/* 
 * ABI messages
 */
#ifndef ABI_FLOOR_DETECTION_ID
#define ABI_FLOOR_DETECTION_ID ABI_BROADCAST
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int v1, int v2, int v3, int v4, int v5, int v6, int v7, int v8, int v9, int v10)
{
  //    I checked it working well!
      printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", v1,v2,v3,v4,v5,v6,v7,v8,v9,v10);
  //
  vv1 = v1;
  vv2 = v2;
  vv3 = v3;
  vv4 = v4;
  vv5 = v5;
  vv6 = v6;
  vv7 = v7;
  vv8 = v8;
  vv9 = v9;
  vv10 = v10;
}

/*
 * Initialisation function
 */
void mav_course_navigation_init(void)
{
  AbiBindMsgFLOOR_DETECTION(ABI_FLOOR_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Periodic function
 */
void mav_course_navigation_periodic(void)
{
  
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




    //TODO: For most of this, check the periodic in orange_avoider_guided

    //TODO: Decide on a safe distance from the drone, i.e. 1 meter
    //TODO: Measure how many pixel wide (+ safety margin) the image of the drone from the safe distance.
    // ...i.e. take a picture with a drone of an other drone located 1 meter away in front of it
    //TODO: take this width in the center of the image, I'll call it safety band from now on.

    /// Front camera horizontal FOV 1.8rad~=103deg (bebop2.sdf #233) -S.

    //TODO: switch case logic, like in orange_avoider_guided:
    // ...if obstacle inside the safety band on one side, then turn away one increment
    // ...if obstacle inside the safety band on BOTH side, then do some evasive manoeuvre, like rotate 90deg
    // ...if no obstacle inside the safety band, keep flying forward
}
