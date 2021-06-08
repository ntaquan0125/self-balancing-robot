/*
 * serial.h
 *
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include <stdint.h>

#include "robot.h"

// Function prototypes
void uart_init(uint32_t baudrate);
void uart_receive(char *buf);
void uart_send(char *buf, uint16_t len);

#endif /* INC_SERIAL_H_ */
