/*
 * Copyright (C) Daniel Toth <d.toth@student.tudelft.nl>
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

/** @file "modules/mav_course_exercise/colorfilter_avoider.c"
 * @author Daniel Toth <d.toth@student.tudelft.nl>
 * Experimental obstacle avoidance based on color filtering.
 */

#include "modules/mav_course_exercise/colorfilter_avoider.h"


// TODO: define color ranges

//TODO: find_object_centroid from cv_detect_color_object

//TODO: OpenCV findContours

//TODO: For each contour: filter them based on OpenCV moments['M00']

void colorfilter_avoider_init(void)
{
  // your init code here
}

void colorfilter_avoider_periodic(void)
{
  // your periodic code here.
  // freq = 4.0 Hz
}


