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
#include "subsystems/abi.h"
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
#include <sys/types.h>
#include <pwd.h>

// Import custom opencv functions
#include "modules/mav_course_edges/opencv_functions.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[mav_course_edges->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

// Define functions
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

static char save_dir[256];

#ifndef MT9F002_OUTPUT_HEIGHT
#define MT9F002_OUTPUT_HEIGHT 520
#endif
int obstacles[MT9F002_OUTPUT_HEIGHT] = {0};

/*
 * Initialisation function
 */
void mav_course_edges_init(void)
{
  // Set frame output save path
  sprintf(save_dir, "%s/edgebox_capture", STRINGIFY(VIDEO_CAPTURE_PATH));

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
  // get obstacles from opencv implementation
  get_obstacles_edgebox((char *) img->buf, img->w, img->h, obstacles, show_debug);

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
 * Function that sends the obstacle data
 */
void mav_course_edges_periodic(void)
{
  AbiSendMsgOBSTACLES(EDGEBOX_ID, obstacles);
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