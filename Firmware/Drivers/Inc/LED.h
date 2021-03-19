/*
 * LED.h
 *
 */

#ifndef INC_LED_H_
#define INC_LED_H_

typedef enum {
    LED_RED,
    LED_BLUE,
    LED_GREEN
} led_t;

typedef enum {
    LED_OFF,
    LED_ON
} led_state_t;

// Function prototypes
void led_init(void);
void set_led(led_t led, led_state_t state);

#endif /* INC_LED_H_ */
