/*
 * ControlerConfig.h
 *
 *  Created on: Dec 30, 2025
 *      Author: Z440
 */

#ifndef INC_CONTROLERCONFIG_H_
#define INC_CONTROLERCONFIG_H_

typedef struct{
	uint16_t TargetSpeed;
	uint16_t TargetPos;
	uint16_t Acceleration;
	uint16_t Deceleration;
	uint16_t MaxTorque;
}sControlConfig;


extern sControlConfig ControlConfig;

#endif /* INC_CONTROLERCONFIG_H_ */
