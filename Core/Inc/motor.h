/*
 * motor.h
 *
 *  Created on: Dec 21, 2024
 *      Author: 86131
 *
 *  Description: 底盘PWM和PID控制库
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MAX_PWM 36000
#define LEFT_MAX_SPEED 1022.14
#define RIGHT_MAX_SPEED 1007.91


typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float cur_e; // e(k)
	float last_e; // e(k-1)
	float pre_e; // e(k-2)
	float integral;
	float res;
	float limit;
} PID;

typedef struct {
	float leftMotorSpeed, rightMotorSpeed;
	// 0-STOP 1-GO 2-BACK
	int leftDirection, rightDirection;
	PID leftPID, rightPID;
	float leftTargetSpeed, rightTargetSpeed;
	float angle;
} ChassisData;

void LeftMotor_Go(void);
void LeftMotor_Back(void);
void LeftMotor_Stop(void);
void RightMotor_Go(void);
void RightMotor_Back(void);
void RightMotor_Stop(void);
void Update_PluseArray(void);
float Calculate_Speed(int);
float PID_Putout(float, float, PID*);
void Chassis_Init(void);
void Chassis_Reset(PID*);
void Chassis_Update(void);

#endif /* INC_MOTOR_H_ */
