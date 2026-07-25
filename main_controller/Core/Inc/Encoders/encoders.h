#ifndef ENCODERS_H
#define ENCODERS_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define LEFT_ENCODER_TIMER TIM2
#define RIGHT_ENCODER_TIMER TIM1

// Parameters to calculate traveled distance
#define ENCODER_PPR 7.0f
#define MOTOR_GEAR_RATIO 50.0f
#define WHEEL_GEAR_RATIO 1.25f
#define WHEEL_DIAMETER_MM 32.0f

#define PI 3.14159265358979323846f

// Encoder data structure
typedef struct {
    
    // Required variables
    float distance;
    int32_t count;
    int16_t prev_count;
    int16_t delta_count;

    int8_t encoder_polarity; // Direction polarity: 1 or -1
} Encoder_t;

// Function prototypes
void Encoders_Init(void);
void Encoders_Reset(void);
void Encoders_Update(void);

int32_t Encoder_getLeftCount(void);
int32_t Encoder_getRightCount(void);
int16_t Encoder_getLeftDeltaCount(void);
int16_t Encoder_getRightDeltaCount(void);
float Encoder_getLeftDistance(void);
float Encoder_getRightDistance(void);

float Encoder_getAverageDistance(void);

#endif // ENCODERS_H