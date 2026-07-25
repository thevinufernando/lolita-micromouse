#ifndef DRV8833_H_
#define DRV8833_H_

#include "main.h"

//Motor driver pins configuration
#define MOTOR_DRIVER_ENABLE_GPIO_Port   GPIOB
#define MOTOR_DRIVER_ENABLE_Pin         GPIO_PIN_14

#define MOTOR_PWM_MAX 4799
#define MOTOR_SPEED_MAX 255

void MotorDriver_Enable(void);
void MotorDriver_Disable(void);

//Functions prototypes for Forward and Backward movement
void MotorForward_runSpeed(uint8_t leftspeed, uint8_t rightspeed);
void MotorBackward_runSpeed(uint8_t leftspeed, uint8_t rightspeed);
void MotorLeftTurn_runSpeed(uint8_t speed);
void MotorRightTurn_runSpeed(uint8_t speed);

#endif /* DRV8833_H_ */