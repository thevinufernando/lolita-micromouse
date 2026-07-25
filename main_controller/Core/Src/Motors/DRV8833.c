#include "DRV8833.h"

extern TIM_HandleTypeDef htim3;

// Enable the driver (wake)
void MotorDriver_Enable(void) {

    HAL_GPIO_WritePin(MOTOR_DRIVER_ENABLE_GPIO_Port, MOTOR_DRIVER_ENABLE_Pin, GPIO_PIN_SET);
    
    //Give some time to wake up
    HAL_Delay(10); 
}

// Disable the driver (sleep)
void MotorDriver_Disable(void) {

    HAL_GPIO_WritePin(MOTOR_DRIVER_ENABLE_GPIO_Port, MOTOR_DRIVER_ENABLE_Pin, GPIO_PIN_RESET);
    
    //Give some time to sleep
    HAL_Delay(10); 
}

// Helper function to map speed (0-255) to PWM value
static uint16_t mapSpeedToPWM(uint8_t speed);

// ================= HELPER FUNCTIONS Declarations for Forward and Backward runs =================
static void LeftMotor_Forward(uint16_t pwm);
static void RightMotor_Forward(uint16_t pwm);
static void LeftMotor_Backward(uint16_t pwm);
static void RightMotor_Backward(uint16_t pwm);
static void LeftMotor_Stop(void);
static void RightMotor_Stop(void);

//================= END OF HELPERS =================

//Drive both motors forward at specified speeds (0-255)
void MotorForward_runSpeed(uint8_t leftspeed, uint8_t rightspeed) {

    //Check motor speed limits
    if (leftspeed > MOTOR_SPEED_MAX) leftspeed = MOTOR_SPEED_MAX;
    if (rightspeed > MOTOR_SPEED_MAX) rightspeed = MOTOR_SPEED_MAX;
    
    //Map speeds to PWM values
    uint16_t leftpwm = mapSpeedToPWM((uint8_t)leftspeed);
    uint16_t rightpwm = mapSpeedToPWM((uint8_t)rightspeed);
    
    //Drive motors
    if (leftspeed > 0 && rightspeed > 0) {
        LeftMotor_Forward(leftpwm);
        RightMotor_Forward(rightpwm);
    }
    else {
        LeftMotor_Stop();
        RightMotor_Stop();
    }
}

//Drive both motors backward at specified speeds (0-255)
void MotorBackward_runSpeed(uint8_t leftspeed, uint8_t rightspeed) {

    //Check motor speed limits
    if (leftspeed > MOTOR_SPEED_MAX) leftspeed = MOTOR_SPEED_MAX;
    if (rightspeed > MOTOR_SPEED_MAX) rightspeed = MOTOR_SPEED_MAX;
    
    //Map speeds to PWM values
    uint16_t leftpwm = mapSpeedToPWM((uint8_t)leftspeed);
    uint16_t rightpwm = mapSpeedToPWM((uint8_t)rightspeed);
    
    //Drive motors
    if (leftspeed > 0 && rightspeed > 0) {
        LeftMotor_Backward(leftpwm);
        RightMotor_Backward(rightpwm);
    }
    else {
        LeftMotor_Stop();
        RightMotor_Stop();
    }
}

//Drive left(anticlockwise) pivot turnings
void MotorLeftTurn_runSpeed(uint8_t speed) {

    //Check motor speed limits
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;

    //Map speeds to PWM values
    uint16_t pwm = mapSpeedToPWM((uint8_t)speed);

    //Drive motors
    if (speed > 0) {
        RightMotor_Forward(pwm);
        LeftMotor_Backward(pwm);
    }
    else {
        LeftMotor_Stop();
        RightMotor_Stop();
    }
}

//Drive right(clockwise) pivot turnings
void MotorRightTurn_runSpeed(uint8_t speed) {

    //Check motor speed limits
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;

    //Map speeds to PWM values
    uint16_t pwm = mapSpeedToPWM((uint8_t)speed);

    //Drive motors
    if (speed > 0) {
        RightMotor_Backward(pwm);
        LeftMotor_Forward(pwm);
    }
    else {
        LeftMotor_Stop();
        RightMotor_Stop();
    }
}

//Clamp a signed speed request to [-MOTOR_SPEED_MAX, +MOTOR_SPEED_MAX]
static uint8_t clampSignedSpeed(float speed, int8_t *out_sign) {

    //Resolve direction before taking the magnitude
    *out_sign = (speed < 0.0f) ? -1 : 1;

    float magnitude = (speed < 0.0f) ? -speed : speed;

    if (magnitude > (float)MOTOR_SPEED_MAX) magnitude = (float)MOTOR_SPEED_MAX;

    return (uint8_t)magnitude;
}

//Drive each motor with a signed speed (-255..255); sign selects direction per wheel
void Motor_runSignedSpeed(float leftspeed, float rightspeed) {

    int8_t left_sign;
    int8_t right_sign;

    uint8_t left_magnitude = clampSignedSpeed(leftspeed, &left_sign);
    uint8_t right_magnitude = clampSignedSpeed(rightspeed, &right_sign);

    uint16_t leftpwm = mapSpeedToPWM(left_magnitude);
    uint16_t rightpwm = mapSpeedToPWM(right_magnitude);

    //Left wheel
    if (left_magnitude == 0) {
        LeftMotor_Stop();
    }
    else if (left_sign > 0) {
        LeftMotor_Forward(leftpwm);
    }
    else {
        LeftMotor_Backward(leftpwm);
    }

    //Right wheel
    if (right_magnitude == 0) {
        RightMotor_Stop();
    }
    else if (right_sign > 0) {
        RightMotor_Forward(rightpwm);
    }
    else {
        RightMotor_Backward(rightpwm);
    }
}

//Brake both motors
void Motor_Brake(void) {

    LeftMotor_Stop();
    RightMotor_Stop();
}

// Helper function to map speed (0-255) to PWM value
static uint16_t mapSpeedToPWM(uint8_t speed) {

    // Scale 0–255 to 0–MOTOR_PWM_MAX
    return (uint16_t)(((uint32_t)speed * MOTOR_PWM_MAX) / MOTOR_SPEED_MAX);
}

// ================= HELPER FUNCTIONS for Forward and Backward runs =================

static void LeftMotor_Forward(uint16_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // AIN1 = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);   // AIN2 = PWM
}

static void RightMotor_Forward(uint16_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);     // AIN1 = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);   // AIN2 = PWM
}

static void LeftMotor_Backward(uint16_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);     // AIN1 = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);   // AIN2 = PWM
}

static void RightMotor_Backward(uint16_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);     // AIN1 = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);   // AIN2 = PWM
}

//Braek motors (both pins HIGH)
static void LeftMotor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_PWM_MAX);     // AIN1 = max
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_PWM_MAX);     // AIN2 = max
}

//Braek motors (both pins HIGH)
static void RightMotor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_PWM_MAX);     // AIN1 = max
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_PWM_MAX);     // AIN2 = max
}

//================= END OF HELPERS =================

