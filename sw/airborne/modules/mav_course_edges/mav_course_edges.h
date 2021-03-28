/*
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * This module sends an obstacle array via ABI containing the lateral position of detected obstacles in the frame.
 * Edgedetection is used as the main cv mothod to detect the obstacles.
 * 
 */
/**
 * @file "modules/mav_course_edges/mav_course_edges.h"
 * @author Group 3 MAV course 2021
 */

#ifndef MAV_COURSE_EDGES_H
#define MAV_COURSE_EDGES_H

#ifndef MT9F002_OUTPUT_HEIGHT
#define MT9F002_OUTPUT_HEIGHT 520
#endif
struct obstacles_t
{
  int x[MT9F002_OUTPUT_HEIGHT];
};

// settings
extern int eb_active;
extern float eb_hor_thresh;
extern int eb_blur_size;
extern int eb_canny_thresh_1;
extern int eb_canny_thresh_2;
extern float eb_size_thresh;
extern int eb_diff_thresh;
extern int show_debug;

// functions
extern void mav_course_edges_init(void);

#endif

