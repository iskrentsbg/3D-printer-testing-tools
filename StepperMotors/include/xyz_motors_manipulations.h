#ifndef XYZ_MOTORS_MANIPULATIONS_H
#define XYZ_MOTORS_MANIPULATIONS_H

#include <AccelStepper.h>

// Motor states used in go_homing()
#define MOVE 1
#define IDLE 2

// Motor mode
#define HALFSTEP 8

// Button pins
#define BUTTON_X 34
#define BUTTON_Y 35
#define BUTTON_Z 32

// Motor pins
// Motor X
#define MOTOR_X_IN_1 14
#define MOTOR_X_IN_2 27
#define MOTOR_X_IN_3 26
#define MOTOR_X_IN_4 25
// Motor Y
#define MOTOR_Y_IN_1 4
#define MOTOR_Y_IN_2 16
#define MOTOR_Y_IN_3 17
#define MOTOR_Y_IN_4 5
// Motor Z
#define MOTOR_Z_IN_1 19
#define MOTOR_Z_IN_2 21
#define MOTOR_Z_IN_3 22
#define MOTOR_Z_IN_4 23

/// @brief Used to save energy while the motors do not move.
void enable_outputs();

/// @brief Used to save energy while the motors do not move.
void disalble_outputs();

/// @brief Makes the necessary preparations before starting manipulating
/// the motors (i.e., defines the buttons as inputs and sets the speed and
/// acceleration and deceleration rates for the motors). 
///
void motors_setup();

/// @brief Tests the operation of the motors. Moves them to the end position, 
/// waits, moves them backwards to the middle position, waits and then moves them
/// backwards to the start position.
bool test_motors();

/// @brief Make all the motors idle and sets their current position to 0.
void make_motors_idle();

/// @brief Rotates each motor while the corresponding button responsible for it is pressed. 
/// The side effect is that the function is blocking since no other process can be executed 
/// before all of the buttons are being pressed. 
bool go_homing();

/// @brief Rotates all the motors to the absolute positions given. 
/// The side effect is that the function is blocking since no other process can be executed 
/// before all of the motors are moved to their target position. 
bool move_motors(int x_pos, int y_pos, int z_pos);

#endif /*  XYZ_MANIPULATIONS_H */
