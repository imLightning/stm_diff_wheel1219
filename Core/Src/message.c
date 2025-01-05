/*
 * message.c
 *
 *  Created on: Dec 21, 2024
 *      Author: 86131
 *
 *  Description: 上位机通信函数库
 */

#include "message.h"
#include "usart.h"
#include "motor.h"
#include "string.h"

/*
 * 通信协议
 * 消息格式
 *     消息头 数据长度 数据 控制位 校验位 消息尾
 *     55 aa length 00 00 00 00 ctrl crc8 0d 0a
 *         控制位
 *         无 无 左电机方向 右电机方向
 *         00000000
 */

const unsigned char HEADER[2] = { 0x55, 0xaa };
const unsigned char ENDER[2] = { 0x0d, 0x0a };

ChassisMessage chassisMessage;
MessageData LeftTargetSpeed, RightTargetSpeed, LeftMotorSpeed, RightMotorSpeed,
		Angle;
// tempBuffer暂存接收消息
unsigned char sendBuffer[SENDBUFFERSIZE], tempBuffer[MAXBUFFERSIZE];
extern ChassisData chassisData;

// 计算八位循环冗余校验
unsigned char CRC8(unsigned char *arr, unsigned short len) {
	unsigned char crc = 0, i;
	while (len--) {
		crc ^= *arr++;
		for (i = 0; i < 8; i++) {
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}

// TODO 速度精度误差
void Send_ChassisData() {

	int i = 0, len = 0x07;

	LeftMotorSpeed.val = (short) chassisData.leftMotorSpeed;
	RightMotorSpeed.val = (short) chassisData.rightMotorSpeed;
	Angle.val = (short) chassisData.angle;

	// 设置消息
	sendBuffer[0] = HEADER[0];
	sendBuffer[1] = HEADER[1];
	sendBuffer[2] = len;
	for (i = 0; i < 2; i++) {
		sendBuffer[i + 3] = LeftMotorSpeed.data[i];
		sendBuffer[i + 5] = RightMotorSpeed.data[i];
		sendBuffer[i + 7] = Angle.data[i];
	}
	sendBuffer[9] = ((unsigned char) chassisData.leftDirection << 2)
			+ ((unsigned char) chassisData.rightDirection);
	sendBuffer[10] = CRC8(sendBuffer, 3 + len);
	sendBuffer[11] = ENDER[0];
	sendBuffer[12] = ENDER[1];

//	for(i=0;i<13;i++) {
//		printf("buffer[%d]=%c\r\n", i, sendBuffer[i]);
//	}
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) sendBuffer, sizeof(sendBuffer));
}

// 参数是DMA缓冲数组
void Receive_ChassisData(unsigned char receiveBuffer[]) {
	// 拷贝数组
	memcpy(tempBuffer, receiveBuffer, 13);
	// 消息校验
	if (tempBuffer[0] == HEADER[0] && tempBuffer[1] == HEADER[1]) {
		short len = tempBuffer[2];
		unsigned char check = CRC8(&tempBuffer[0], len + 3);
//		printf("check:%#X\r\n", check);
		if (check == tempBuffer[len + 3]) {
			// 读取数据
			for (int i = 0; i < 2; i++) {
				LeftTargetSpeed.data[i] = tempBuffer[i + 3];
				RightTargetSpeed.data[i] = tempBuffer[i + 5];
			}
			unsigned char control = tempBuffer[9];
			chassisData.leftTargetSpeed = (int) LeftTargetSpeed.val;
			chassisData.rightTargetSpeed = (int) RightTargetSpeed.val;
			// 移位读取控制位
			chassisData.leftDirection = 0b00001100 & control;
			chassisData.leftDirection >>= 2;
			chassisData.rightDirection = 0b00000011 & control;
		}
	}
}
