#ifndef __PWM_MUTEBUTTON_H
#define __PWM_MUTEBUTTON_H

#ifdef CONFIG_PWM_MUTEBUTTON

#define LED_PWM_MAX_SCALING 0x7F
#define BYTEMASK 0xFF

#define LED_CALIB_ENABLE_DEFAULT 0
#define LED_PWM_SCALING_DEFAULT 0x007F0000
#define LED_PWM_MAX_LIMIT_DEFAULT 0x00FF0000
#define LED_PWM_INTERCEPT_DEFAULT 0x007F0000

#define BRIGHTNESS_LEVEL_MAX 255
#define LED_PWM_PERIOD_DEFAULT 40000

struct led_mutebutton_data {
	struct mutex lock;
	bool led_calib_enable;
	int led_pwm_scaling;
	int led_pwm_max_limit;
	int led_pwm_intercept;
	bool is_two_point_cal_enabled;
	bool invert;
        u32 led_pwm_period;
};

#endif

#endif	/* __PWM_MUTEBUTTON_H */
