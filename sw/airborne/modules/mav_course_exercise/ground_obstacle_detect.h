//
// Created by dani on 12-03-21.
//

#ifndef MAV2021_GROUND_OBSTACLE_DETECT_H
#define MAV2021_GROUND_OBSTACLE_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

int* ground_obstacle_detect(Mat img, int safe_vector[], int bottom_count, int certainty);


#ifdef __cplusplus
}
#endif

#endif //MAV2021_GROUND_OBSTACLE_DETECT_H
