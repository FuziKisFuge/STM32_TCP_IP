/*
 * Display.h
 *
 *  Created on: Jan 2, 2026
 *      Author: Z440
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

typedef enum{
	Main,
}eMenu;

eMenu Menu = Main;

void updateDisplay(u8g2_t *u8g2, eMenu m)
{
	char str[100];
	u8g2_ClearDisplay(u8g2);

	switch(m)
	{
	case Main:
		sprintf(str, "TargetSpeed: %lu", (unsigned long) ControlConfig.TargetSpeed);
		u8g2_DrawStr(u8g2, 0, 15, str);

		sprintf(str, "TargetPos: %lu", (unsigned long) ControlConfig.TargetPos);
		u8g2_DrawStr(u8g2, 0, 30, str);

		break;
	}

	u8g2_SendBuffer(u8g2);
}

#endif /* INC_DISPLAY_H_ */
