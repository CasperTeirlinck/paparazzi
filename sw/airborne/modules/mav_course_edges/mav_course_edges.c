/*
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/mav_course_edges/mav_course_edges.c"
 * @author Group 3 MAV course 2021
 */

#include "modules/mav_course_edges/mav_course_edges.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>

#include "lib/encoding/jpeg.h"
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

// Import custom opencv functions
#include "modules/mav_course_edges/opencv_functions.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[mav_course_edges->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

// Define functions
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);

static struct image_t *camera_cb(struct image_t *img);
static struct image_t *video_capture_cb(struct image_t *img);
static void video_capture_save(struct image_t *img);

// Define settings
#ifndef EB_HOR_THRESH
#define EB_HOR_THRESH 0.6
#endif
#ifndef EB_BLUR_SIZE
#define EB_BLUR_SIZE 41
#endif
#ifndef EB_CANNY_THRESH_1
#define EB_CANNY_THRESH_1 200
#endif
#ifndef EB_CANNY_THRESH_2
#define EB_CANNY_THRESH_2 250
#endif
#ifndef EB_SIZE_THRESH
#define EB_SIZE_THRESH 75
#endif
#ifndef EB_DIFF_THRESH
#define EB_DIFF_THRESH 60
#endif
#ifndef EB_SHOW_DEBUG
#define EB_SHOW_DEBUG 0
#endif

float eb_hor_thresh = EB_HOR_THRESH;
int eb_blur_size = EB_BLUR_SIZE;
int eb_canny_thresh_1 = EB_CANNY_THRESH_1;
int eb_canny_thresh_2 = EB_CANNY_THRESH_2;
float eb_size_thresh = EB_SIZE_THRESH;
int eb_diff_thresh = EB_DIFF_THRESH;
int show_debug = EB_SHOW_DEBUG; // toggle writing frames to memory and disk

// Define and initialise global variables
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
float moveDistance = 2;                 // waypoint displacement [m]
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]

static char save_dir[256];

#ifndef MT9F002_OUTPUT_HEIGHT
#define MT9F002_OUTPUT_HEIGHT 520
#endif
int obstacles[MT9F002_OUTPUT_HEIGHT];

/*
 * Initialisation function
 */
void mav_course_edges_init(void)
{
  // Set frame output save path
  sprintf(save_dir, "/home/casper/paparazzi/prototyping/paparazzi_capture");

  // Attach callback function to the front camera for obstacle avoidance
  cv_add_to_device(&front_camera, camera_cb, 0);
  // Attach callback function to the front camera for debugging
  cv_add_to_device(&front_camera, video_capture_cb, 2);
}

/*
 * Callback from the camera for obstacle detection
 * @param img - input image to process
 * @return img
 */
struct image_t *camera_cb(struct image_t *img)
{
  get_obstacles_edgebox((char *) img->buf, img->w, img->h, show_debug);

  return img;
}

/*
 * Callback from the camera for video capture
 * @param img - input image to process
 * @return img
 */
struct image_t *video_capture_cb(struct image_t *img)
{
  if (show_debug) {
    video_capture_save(img);
  }

  return NULL;
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void mav_course_edges_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      // Check:
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;
    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);

      navigation_state = SAFE;
      break;
    case OUT_OF_BOUNDS:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);

        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/* 
 * Saves a frame to disk for debugging the cv algorithm output
 * Derived from video_capture.c
 */
void video_capture_save(struct image_t *img)
{
  // Create output folder if necessary
  if (access(save_dir, F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", save_dir);
    if (system(save_dir_cmd) != 0) {
      printf("[mav_course_edges->video_capture] Could not create images directory %s.\n", save_dir);
      return;
    }
  }

  // Declare storage for image location
  char save_name[256];

  // Generate image filename from image timestamp
  sprintf(save_name, "%s/%u.jpg", save_dir, img->pprz_ts);
  printf("[mav_course_edges->video_capture] Saving image to %s.\n", save_name);

  // Create jpg image from raw frame
  struct image_t img_jpeg;
  image_create(&img_jpeg, img->w, img->h, IMAGE_JPEG);
  jpeg_encode_image(img, &img_jpeg, 99, true);

  // Open file
  FILE *fp = fopen(save_name, "w");
  if (fp == NULL) {
    printf("[mav_course_edges->video_capture] Could not write shot %s.\n", save_name);
    return;
  }

  // Save it to the file and close it
  fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, fp);
  fclose(fp);

  // Free image
  image_free(&img_jpeg);
}