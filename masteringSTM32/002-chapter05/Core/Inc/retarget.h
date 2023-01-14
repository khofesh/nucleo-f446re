/*
 * retarget.h
 *
 *  Created on: Jan 14, 2023
 *      Author: fahmad
 */

#ifndef INC_RETARGET_H_
#define INC_RETARGET_H_

#include "stm32f4xx_hal.h"
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);


#endif /* INC_RETARGET_H_ */
