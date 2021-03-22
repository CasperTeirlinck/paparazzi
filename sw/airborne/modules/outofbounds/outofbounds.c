/*
 * Copyright (C) Ricardo Caldas Santana <ricardocsantana99@gmail.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/outofbounds/outofbounds.c"
 * @author Ricardo Caldas Santana <ricardocsantana99@gmail.com>
 * Avoid reaching OOB.
 */

#include "modules/outofbounds/outofbounds.h"
#include "subsystems/abi.h"
#include "modules/outofbounds/opencv_functions.h"

#ifndef FLOOR_DETECT_TYPE
#define FLOOR_DETECT_TYPE 0 ///< 0 is simulation, 1 is real flight
#endif

//static struct image_t *camera_cb(struct image_t *img);

float result;

struct image_t *camera_func(struct image_t *img)
{
  result = findColor((char *) img->buf, img->w, img->h, FLOOR_DETECT_TYPE);

  return img;
}

void init_outofbounds(void)
{
	// Attach callback function to the front camera for obstacle avoidance
  	cv_add_to_device(&bottom_camera, camera_func, 0);
 
    }
    


