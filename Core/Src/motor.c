/*
 * 底盘控制库
 *
 * 硬件：
 * stm32f103c8t6，mpu6050，TT霍尔编码器马达，L298N
 *
 * 定时器使用：
 * 1 PWM 通道1右电机
 * 2 测速 右电机
 * 3 测速 左电机
 * 4 定时 50毫秒
 *
 */

#include "motor.h"
#include "gpio.h"
#include "tim.h"

ChassisData chassisData;
extern float yaw;

// 电机方向控制
void LeftMotor_Back() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}
void LeftMotor_Go() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}
void LeftMotor_Stop() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
void RightMotor_Back() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}
void RightMotor_Go() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}
void RightMotor_Stop() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

// 编码器测速
short pluseArray[2] = { 0 };
void Update_PluseArray() {
	pluseArray[0] = -(short) __HAL_TIM_GET_COUNTER(&htim2);
	pluseArray[1] = (short) __HAL_TIM_GET_COUNTER(&htim3);
//	printf("rightpluse:%d,leftpluse:%d\r\n\r\n", pluseArray[0], pluseArray[1]);
	__HAL_TIM_GET_COUNTER(&htim2) = 0;
	__HAL_TIM_GET_COUNTER(&htim3) = 0;
}

// 小车速度(mm/s) = 周长/电机转动一圈的脉冲数 * 一定时间内测量的脉冲数/定时器设置的时间
float Calculate_Speed(int pluse) {
	/*
	 int wheel_diameter = 65;
	 int timer = 50;
	 int full_pluse = 1980;
	 */
	float k = 2.06162;
	return (float) (k * pluse);
}

// PID算法速度控制
float PID_Putout(float target, float actual, PID *P) {
	float res;

	P->cur_e = target - actual;

	// 积分视为三次误差累积
	// integral = ek + e(k-1) + e(k-2)
//	P->integral = P->cur_e + P->last_e + P->pre_e;

//	printf("cur_e = %f - %f\r\n", target, actual);
//	printf("integral = %f\r\n", P->integral);
//	printf("res = %f * %f + (%f * %f) + %f * (%f - %f)\r\n\r\n", P->Kp,
//			P->cur_e, P->Ki, P->integral, P->Kd, P->last_e, P->cur_e);

//	res = P->Kp * P->cur_e + (P->Ki * P->integral)
//			+ P->Kd * (P->last_e - P->cur_e);

	// 增量式
	res = P->Kp * (P->cur_e - P->last_e) + P->Ki * P->cur_e;

	P->res += res;

	if(P->res > P->limit) P->res = P->limit;
	if(P->res < -P->limit) P->res = -P->limit;

//	P->pre_e = P->last_e;
	P->last_e = P->cur_e;
	return P->res;
}

// 初始化底盘
void Chassis_Init() {
	chassisData.leftPID.Kp = 1.0;
	chassisData.leftPID.Ki = 1.0;
	chassisData.leftPID.Kd = 0.0;
	chassisData.leftPID.res = 0.0;
	chassisData.leftPID.limit = LEFT_MAX_SPEED;
	chassisData.leftDirection = 0;
	chassisData.leftMotorSpeed = 0.0;
	chassisData.leftTargetSpeed = 0.0;
	chassisData.rightPID.Kp = 1.0;
	chassisData.rightPID.Ki = 1.0;
	chassisData.rightPID.Kd = 0.0;
	chassisData.rightPID.res = 0.0;
	chassisData.rightPID.limit = RIGHT_MAX_SPEED;
	chassisData.rightDirection = 0;
	chassisData.rightMotorSpeed = 0.0;
	chassisData.rightTargetSpeed = 0.0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
}

// 复位
void Chassis_Reset(PID *P) {
	P->last_e = 0.0;
	P->cur_e = 0.0;
	P->res = 0.0;
}

// 更新底盘
void Chassis_Update() {

	// 放大数据
	chassisData.angle = 88 * yaw;

	// 计算速度
	Update_PluseArray();
	float leftMotorSpeed = Calculate_Speed(pluseArray[1]);
	float rightMotorSpeed = Calculate_Speed(pluseArray[0]);

//	printf("leftMotorSpeed:%f\r\n", leftMotorSpeed);
//	printf("rightMotorSpeed:%f\r\n", rightMotorSpeed);
	chassisData.leftMotorSpeed = leftMotorSpeed;
	chassisData.rightMotorSpeed = rightMotorSpeed;

	switch (chassisData.leftDirection) {
	case 0:
		LeftMotor_Stop();
		Chassis_Reset(&chassisData.leftPID);
		break;
	case 1:
		LeftMotor_Go();
		break;
	case 2:
		LeftMotor_Back();
		// PID函数限制正值
//		chassisData.leftMotorSpeed = -chassisData.leftMotorSpeed;
		break;
	default:
		LeftMotor_Stop();
		Chassis_Reset(&chassisData.leftPID);
		break;
	}

	switch (chassisData.rightDirection) {
	case 0:
		RightMotor_Stop();
		Chassis_Reset(&chassisData.rightPID);
		break;
	case 1:
		RightMotor_Go();
		break;
	case 2:
		RightMotor_Back();
//		chassisData.rightMotorSpeed = -chassisData.rightMotorSpeed;
		break;
	default:
		RightMotor_Stop();
		Chassis_Reset(&chassisData.rightPID);
		break;
	}

	// 控制速度
	float leftPWM = PID_Putout(chassisData.leftTargetSpeed,
			chassisData.leftMotorSpeed, &chassisData.leftPID) / LEFT_MAX_SPEED * MAX_PWM;
	float rightPWM = PID_Putout(chassisData.rightTargetSpeed,
			chassisData.rightMotorSpeed, &chassisData.rightPID) / RIGHT_MAX_SPEED * MAX_PWM;

	// Absolute
	if (leftPWM < 0)
		leftPWM = -leftPWM;
	if (rightPWM < 0)
		rightPWM = -rightPWM;
//	printf("leftpwm:%f\r\n\r\n", leftPWM);
//	printf("rightpwm:%f\r\n\r\n", rightPWM);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, rightPWM);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, leftPWM);
}

