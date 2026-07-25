#include "encoders.h"

// timer handles from CubeMX 
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// Encoder instances
static Encoder_t left_encoder = {.encoder_polarity = 1};
static Encoder_t right_encoder = {.encoder_polarity = -1};

void Encoders_Init(void) {

    // Reset encoders
    Encoders_Reset();
}

void Encoders_Reset(void) {
    
    // Reset hardware timers
    LEFT_ENCODER_TIMER->CNT = 0;
    RIGHT_ENCODER_TIMER->CNT = 0;

    // Reset left encoder
    left_encoder.count = 0;
    left_encoder.prev_count = 0;
    left_encoder.delta_count = 0;
    left_encoder.distance = 0.0f;

    // Reset right encoder
    right_encoder.count = 0;
    right_encoder.prev_count = 0;
    right_encoder.delta_count = 0;
    right_encoder.distance = 0.0f;
}

void Encoders_Update(void) {

    // Current counts from hardware timers
    int16_t current_left_count = (int16_t)LEFT_ENCODER_TIMER->CNT;
    int16_t current_right_count = (int16_t)RIGHT_ENCODER_TIMER->CNT;

    // Update delta counts
    left_encoder.delta_count = (current_left_count - left_encoder.prev_count) * left_encoder.encoder_polarity;
    right_encoder.delta_count = (current_right_count - right_encoder.prev_count) * right_encoder.encoder_polarity;

    // Update previous counts
    left_encoder.prev_count = current_left_count;
    right_encoder.prev_count = current_right_count;

    // Update total counts
    left_encoder.count += left_encoder.delta_count;
    right_encoder.count += right_encoder.delta_count;

    //Update the traveled distance
    left_encoder.distance = (((float)left_encoder.count / (WHEEL_GEAR_RATIO * MOTOR_GEAR_RATIO * ENCODER_PPR * 4.0f)) * (WHEEL_DIAMETER_MM * PI)) / 10.0f; // in cm
    right_encoder.distance = (((float)right_encoder.count / (WHEEL_GEAR_RATIO * MOTOR_GEAR_RATIO * ENCODER_PPR * 4.0f)) * (WHEEL_DIAMETER_MM * PI)) / 10.0f; // in cm
}

int32_t Encoder_getLeftCount(void) {

    // Return the total count of the left encoder
    return left_encoder.count;
}

int32_t Encoder_getRightCount(void) {

    // Return the total count of the right encoder
    return right_encoder.count;
}


int16_t Encoder_getLeftDeltaCount(void) {

    // Return the delta count of the left encoder
    return left_encoder.delta_count;
}

int16_t Encoder_getRightDeltaCount(void) {

    // Return the delta count of the right encoder
    return right_encoder.delta_count;
}


float Encoder_getLeftDistance(void) {

    // Return the traveled distance of the left encoder
    return left_encoder.distance;
}

float Encoder_getRightDistance(void) {

    // Return the traveled distance of the right encoder
    return right_encoder.distance;
}


float Encoder_getAverageDistance(void) {

    // Return the average traveled distance of both encoders
    return (left_encoder.distance + right_encoder.distance) / 2.0f;
}