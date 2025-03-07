#include "xyz_motors_manipulations.h"

// motor_x.move(-1000); // -1000 is clockwise
// motor_x.run();

// motor_x.move(1000); // 1000 is anticlockwise
// motor_x.run();

#define MOTOR_VALUE 1000.0
#define GO_HOMING_SPEED 500.0

typedef enum
{
    TESTING_STATE_INITIALIZE,
    TESTING_STATE_WAIT_AFTER_INITIALIZE,
    TESTING_STATE_GO_TO_END,
    TESTING_STATE_WAIT_AFTER_END,
    TESTING_STATE_GO_BACK_TO_HALF,
    TESTING_STATE_WAIT_AFTER_HALF,
    TESTING_STATE_GO_BACK_TO_BEGGINING
} Testing_State;

typedef enum
{
    MOVE_MOTORS_INITIALIZE,
    MOVE_MOTORS_MOVE
} Move_Motors_State;

typedef enum
{
    HOMING_GO_FORTH_TO_ZERO_POSITION,
    HOMING_GO_BACK_TO_ZERO_POSITION
} Homing_State;

// Absolute positions used in test_motors()
#define END_POSITION 4096
#define HALF_POSITION 2048
#define ZERO_POSITION 0



// Intervals
#define WAITING_INTERVAL 1000
#define BUTTON_DEBOUNCE_ITERVAL 40

Testing_State test_state;
Move_Motors_State move_motors_state;

Homing_State homing_state_x = HOMING_GO_FORTH_TO_ZERO_POSITION;
Homing_State homing_state_y = HOMING_GO_FORTH_TO_ZERO_POSITION;
Homing_State homing_state_z = HOMING_GO_FORTH_TO_ZERO_POSITION;

// States
 int state_motor_x;
 int state_motor_y;
 int state_motor_z;

int state_btn_x;
int state_btn_y;
int state_btn_z;

int last_state_btn_x = HIGH;
int last_state_btn_y = HIGH;
int last_state_btn_z = HIGH;

// Time variables
unsigned long button_x_debounce_time;
unsigned long button_y_debounce_time;
unsigned long button_z_debounce_time;

unsigned long long start_time;

// Motors variables
AccelStepper motor_x(HALFSTEP, MOTOR_X_IN_1, MOTOR_X_IN_3, MOTOR_X_IN_2, MOTOR_X_IN_4);
AccelStepper motor_y(HALFSTEP, MOTOR_Y_IN_1, MOTOR_Y_IN_3, MOTOR_Y_IN_2, MOTOR_Y_IN_4);
AccelStepper motor_z(HALFSTEP, MOTOR_Z_IN_1, MOTOR_Z_IN_3, MOTOR_Z_IN_2, MOTOR_Z_IN_4);

void enable_outputs()
{
    motor_x.enableOutputs();
    motor_y.enableOutputs();
    motor_z.enableOutputs();
}

void disalble_outputs()
{
    motor_x.disableOutputs();
    motor_y.disableOutputs();
    motor_z.disableOutputs();
}

void motors_setup()
{
    // Prepares motor X
    motor_x.setMaxSpeed(MOTOR_VALUE);
    motor_x.setAcceleration(MOTOR_VALUE);

    // Prepares motor Y
    motor_y.setMaxSpeed(MOTOR_VALUE);
    motor_y.setAcceleration(MOTOR_VALUE);

    // Prepares motor Z
    motor_z.setMaxSpeed(MOTOR_VALUE);
    motor_z.setAcceleration(MOTOR_VALUE);

    pinMode(BUTTON_X, INPUT);
    pinMode(BUTTON_Y, INPUT);
    pinMode(BUTTON_Z, INPUT);

    Serial.begin(9600);

    state_motor_x = MOVE;
    state_motor_y = MOVE;
    state_motor_z = MOVE;

    test_state = TESTING_STATE_INITIALIZE;
    move_motors_state = MOVE_MOTORS_INITIALIZE;

    button_x_debounce_time = millis();
    button_y_debounce_time = millis();
    button_z_debounce_time = millis();
}

bool test_motors()
{
    bool is_finished = false;

    switch (test_state)
    {
    case TESTING_STATE_INITIALIZE:
    {
        if (move_motors(ZERO_POSITION, ZERO_POSITION, ZERO_POSITION))
        {
            test_state = TESTING_STATE_WAIT_AFTER_INITIALIZE;
        }
        break;
    }
    case TESTING_STATE_WAIT_AFTER_INITIALIZE:
    {
        if (millis() - start_time > WAITING_INTERVAL)
        {
            test_state = TESTING_STATE_GO_TO_END;
        }
        break;
    }
    case TESTING_STATE_GO_TO_END:
    {
        if (move_motors(END_POSITION, END_POSITION, END_POSITION))
        {
            test_state = TESTING_STATE_WAIT_AFTER_END;
        }
        break;
    }
    case TESTING_STATE_WAIT_AFTER_END:
    {
        if (millis() - start_time > WAITING_INTERVAL)
        {
            test_state = TESTING_STATE_GO_BACK_TO_HALF;
        }
        break;
    }
    case TESTING_STATE_GO_BACK_TO_HALF:
    {
        if (move_motors(HALF_POSITION, HALF_POSITION, HALF_POSITION))
        {
            test_state = TESTING_STATE_WAIT_AFTER_HALF;
        }

        break;
    }
    case TESTING_STATE_WAIT_AFTER_HALF:
    {
        if (millis() - start_time > WAITING_INTERVAL)
        {
            test_state = TESTING_STATE_GO_BACK_TO_BEGGINING;
        }
        break;
    }
    case TESTING_STATE_GO_BACK_TO_BEGGINING:
    {
        if (move_motors(ZERO_POSITION, ZERO_POSITION, ZERO_POSITION))
        {
            is_finished = true;
            test_state = TESTING_STATE_INITIALIZE;
        }
        break;
    }
    }

    // Moves the motors.
    motor_x.run();
    motor_y.run();
    motor_z.run();

    return is_finished;
}

bool is_btn_x_pressed_after_debouncing = false;
bool is_btn_y_pressed_after_debouncing = false;
bool is_btn_z_pressed_after_debouncing = false;

void read_button_x()
{
    // Debouncing a button
    if (millis() - button_x_debounce_time > BUTTON_DEBOUNCE_ITERVAL)
    {
        state_btn_x = digitalRead(BUTTON_X);

        if (state_btn_x != last_state_btn_x)
        {
            button_x_debounce_time = millis();

            if (state_btn_x == LOW)
            {
                is_btn_x_pressed_after_debouncing = true;
            }
            else
            {
                is_btn_x_pressed_after_debouncing = false;
            }
            last_state_btn_x = state_btn_x;
        }
    }
}

void read_button_y()
{
    // Debouncing a button
    if (millis() - button_y_debounce_time > BUTTON_DEBOUNCE_ITERVAL)
    {
        state_btn_y = digitalRead(BUTTON_Y);

        if (state_btn_y != last_state_btn_y)
        {
            button_y_debounce_time = millis();

            if (state_btn_y == LOW)
            {
                is_btn_y_pressed_after_debouncing = true;
            }
            else
            {
                is_btn_y_pressed_after_debouncing = false;
            }
            last_state_btn_y = state_btn_y;
        }
    }
}

void read_button_z()
{
    // Debouncing a button
    if (millis() - button_z_debounce_time > BUTTON_DEBOUNCE_ITERVAL)
    {
        state_btn_z = digitalRead(BUTTON_Z);

        if (state_btn_z != last_state_btn_z)
        {
            button_z_debounce_time = millis();

            if (state_btn_z == LOW)
            {
                is_btn_z_pressed_after_debouncing = true;
            }
            else
            {
                is_btn_z_pressed_after_debouncing = false;
            }
            last_state_btn_z = state_btn_z;
        }
    }
}

void go_homing_motor_x()
{
    read_button_x();

    switch (homing_state_x)
    {
    case HOMING_GO_FORTH_TO_ZERO_POSITION:
    {
        if (!is_btn_x_pressed_after_debouncing)
        {
            homing_state_x = HOMING_GO_BACK_TO_ZERO_POSITION;
        }

        motor_x.move(1000);
        motor_x.run();

        break;
    }
    case HOMING_GO_BACK_TO_ZERO_POSITION:
    {
        if (is_btn_x_pressed_after_debouncing)
        {
            homing_state_x = HOMING_GO_FORTH_TO_ZERO_POSITION;
            motor_x.stop();
            motor_x.setCurrentPosition(0);
            state_motor_x = IDLE;

            return;
        }

        motor_x.move(-1000);
        motor_x.run();

        break;
    }
    }
}

void go_homing_motor_y()
{
    read_button_y();

    switch (homing_state_y)
    {
    case HOMING_GO_FORTH_TO_ZERO_POSITION:
    {
        if (!is_btn_y_pressed_after_debouncing)
        {
            homing_state_y = HOMING_GO_BACK_TO_ZERO_POSITION;
        }

        motor_y.move(1000);
        motor_y.run();

        break;
    }
    case HOMING_GO_BACK_TO_ZERO_POSITION:
    {
        if (is_btn_y_pressed_after_debouncing)
        {
            homing_state_y = HOMING_GO_FORTH_TO_ZERO_POSITION;
            motor_y.stop();
            motor_y.setCurrentPosition(0);
            state_motor_y = IDLE;

            return;
        }

        motor_y.move(-1000);
        motor_y.run();

        break;
    }
    }
}

void go_homing_motor_z()
{
    read_button_z();

    switch (homing_state_z)
    {
    case HOMING_GO_FORTH_TO_ZERO_POSITION:
    {
        if (!is_btn_z_pressed_after_debouncing)
        {
            homing_state_z = HOMING_GO_BACK_TO_ZERO_POSITION;
        }

        motor_z.move(1000);
        motor_z.run();

        break;
    }
    case HOMING_GO_BACK_TO_ZERO_POSITION:
    {
        if (is_btn_z_pressed_after_debouncing)
        {
            homing_state_z = HOMING_GO_FORTH_TO_ZERO_POSITION;
            motor_z.stop();
            motor_z.setCurrentPosition(0);
            state_motor_z = IDLE;

            return;
        }

        motor_z.move(-1000);
        motor_z.run();

        break;
    }
    }
}

void make_motors_idle()
{
    state_motor_x = IDLE;
    state_motor_y = IDLE;
    state_motor_z = IDLE;

    motor_x.setCurrentPosition(0);
    motor_y.setCurrentPosition(0);
    motor_z.setCurrentPosition(0);
}

bool go_homing()
{
    if (state_motor_x == IDLE && state_motor_y == IDLE && state_motor_z == IDLE)
    {
        state_motor_x = MOVE;
        state_motor_y = MOVE;
        state_motor_z = MOVE;

        return true;
    }

    motor_x.setMaxSpeed(GO_HOMING_SPEED);
    motor_y.setMaxSpeed(GO_HOMING_SPEED);
    motor_z.setMaxSpeed(GO_HOMING_SPEED);

    if (state_motor_x == MOVE)
    {
        go_homing_motor_x();
    }
    if (state_motor_y == MOVE)
    {
        go_homing_motor_y();
    }
    if (state_motor_z == MOVE)
    {
        go_homing_motor_z();
    }

    motor_x.setMaxSpeed(MOTOR_VALUE);
    motor_y.setMaxSpeed(MOTOR_VALUE);
    motor_z.setMaxSpeed(MOTOR_VALUE);

    return false;
}

bool move_motors(int x_pos, int y_pos, int z_pos)
{
    motor_x.moveTo(x_pos);
    motor_y.moveTo(y_pos);
    motor_z.moveTo(z_pos);

    if (motor_x.distanceToGo() == 0 && motor_y.distanceToGo() == 0 && motor_z.distanceToGo() == 0)
    {
        return true;
    }

    motor_x.run();
    motor_y.run();
    motor_z.run();

    return false;
}
