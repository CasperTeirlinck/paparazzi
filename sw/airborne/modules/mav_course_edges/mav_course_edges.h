/*
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/mav_course_edges/mav_course_edges.h"
 * @author Group 3 MAV course 2021
 */

#ifndef MAV_COURSE_EDGES_H
#define MAV_COURSE_EDGES_H

// settings
extern float eb_hor_thresh;
extern int eb_blur_size;
extern int eb_canny_thresh_1;
extern int eb_canny_thresh_2;
extern float eb_size_thresh;
extern int eb_diff_thresh;
extern int show_debug;

// functions
extern void mav_course_edges_init(void);
extern void mav_course_edges_periodic(void);

#endif

