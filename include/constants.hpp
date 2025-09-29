#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// Miscellaneous
#define STANDARD_DELAY 20
#define STANDARD_HEIGHT -170.0f

// Resting position parameters
#define RESTING_X 100
#define RESTING_Y 100
#define RESTING_H -220

// Sitting positions parameters
#define SITTING_LOWER_HEIGHT -120.0f
#define SITTING_HIGHER_HEIGHT -170.0f

// Walking gait parameters
#define WALKING_BODY_HEIGHT -170.0f
#define WALKING_LEG_DISTANCE_FROM_BODY 100.0f
#define WALKING_LEG_DISTANCE_ALLOWED_BETWEEN_FRONT_AND_BACK 0

// Running gait parameters
#define RUNNING_BODY_HEIGHT -120.0f
#define RUNNING_LEG_DISTANCE_FROM_BODY 100.0f
#define RUNNING_STEP_SIZE 50.0f
#define RUNNING_STEP_HEIGHT 30.0f
#define RUNNING_POINTS_PER_MOVEMENT 9

// Turning gait parameter
#define TURNING_BODY_HEIGHT -120.0f
#define TURNING_ANGLE_A_STEP_COVERS 15.0f

// Level keeping controller parameters
#define LEVELING_P 2.0f
#define LEVELING_I 0.0f
#define LEVELING_D 0.5f

#endif // CONSTANTS_HPP