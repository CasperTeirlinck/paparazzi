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

/** @file "modules/mav_course_exercise/floor_detection.h"
 * @author Daniel Toth <d.toth@student.tudelft.nl>
 * Obstacle detection based on floor colorfiltering.
 */

#ifndef FLOOR_DETECTION_H
#define FLOOR_DETECTION_H
#include "std.h"

extern void floor_detection_init(void);
struct YUV_color{
    uint8_t y;
    uint8_t u;
    uint8_t v;
};

extern struct YUV_color floor_min;
extern struct YUV_color floor_max;

#endif  // FLOOR_DETECTION_H
