/*
 * message.h
 *
 *  Created on: Dec 21, 2024
 *      Author: 86131
 *
 *  Description: 上位机通信函数库
 */

#ifndef INC_MESSAGE_H_
#define INC_MESSAGE_H_

#define MAXBUFFERSIZE 200
#define SENDBUFFERSIZE 13

typedef struct {
	int leftTargetSpeed, rightTargetSpeed;
	unsigned char control;
} ChassisMessage;

typedef union {
	// 16位读取数据
	short val;
	// 8位发送数据
	unsigned char data[2];
} MessageData;

unsigned char CRC8(unsigned char*, unsigned short);
void Receive_ChassisData(unsigned char*);
void Send_ChassisData();

#endif /* INC_MESSAGE_H_ */
