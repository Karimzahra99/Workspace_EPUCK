#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <spi_comm.h>
#include "moving.h"
#include "read_image.h"
#include <audio/play_melody.h>
#include <audio/microphone.h>
#include <audio/audio_thread.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
			115200,
			0,
			0,
			0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

	halInit();
	chSysInit();
	mpu_init();
	dac_start();

	//Initialisation bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	// Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);

	//starts the serial communication / can be removed if communication not needed
	serial_start();
	//start the USB communication / can be removed if communication not needed
	usb_start();
	//starts the camera
	dcmi_start();
	po8030_start();
	//For RGB LEDS
	spi_comm_start();

	/* Tuning parameters for camera :
	 * rgb_gains : [0 255] for each, default : 94, 64, 93
	 * contrast : [0 255], default 64
	 * birghtness : [-128 127], default 0, careful function of po8030.c takes an uint8_t but the format is the following [7]:[6:0] = Sign : Magnitude
	 * line_index_top : [0 480]
	 * mode_detect : MAX_ONLY, MEAN_ONLY, MAX_N_MEANS
	 * plot_pixels_color : RED_IDX, GREEND_IDX, BLUE_IDX
	 * send_params : NO_VISUALIZE_PARAMS, VISUALIZE_PARAMS
	 */

	rgb_gain_t rgb_gains = {150, 130, 150};
	uint8_t contrast = 78;
	uint8_t brightness = 10;
	//tuning uses line_index_top for plot visualization
	uint16_t line_index_top = 10;
	detect_mode_t mode_detect = MAX_ONLY;
	visualize_mode_t send_params = NO_VISUALIZE_PARAMS;
#ifdef TUNE
	//chose which color intensity to plot with plot_image.py
	color_index_t plot_pixels_color = GREEN_IDX;
#else
	uint16_t line_index_bot = 400;
#endif


	//TUNE is defined in main.h
#ifdef TUNE
	//Contrast level, camera line index, detect_mode, image color, visualize parameters such as means, maxs, counts on terminal
	//Adjust Contrast, Line_Idx and detection mode  in Main.h
	tuning_config_t tunning = {rgb_gains, contrast, brightness, line_index_top, mode_detect, plot_pixels_color, send_params};
	tune_image_start(tunning);
#else

	//inits the motors
	motors_init();

	config_t config = {rgb_gains, contrast, brightness, line_index_top, line_index_bot, mode_detect, send_params};
	read_image_start(config);

	proximity_start();
	calibrate_ir();

	//give sime time to find the color if there is one
	chThdSleepMilliseconds(1000);

	mic_start(NULL);

	//playMelodyStart();

	moving_start();

#endif

	while (1) {
		//waits 1 second
		chThdSleepMilliseconds(1000);

		//chprintf((BaseSequentialStream *)&SD3, "Mode =%-7d \r\n\n",get_rolling_mode());
//		chprintf((BaseSequentialStream *)&SD3, "TOP =%-7d BOT =%-7d DIFF =%-7d COLOR =%-7d \r\n\n",
//						get_middle_top(), get_middle_bot(), get_middle_diff(),get_color());

		chprintf((BaseSequentialStream *)&SD3, "TOP =%-7d BOT =%-7d DIFF =%-7d COLOR =%-7d Mode =%-7d \r\n\n",
					get_middle_top(), get_middle_bot(), get_middle_diff(),get_color(),get_rolling_mode());

	}
}

//Led setting function
void set_leds(color_index_t color_index){
	if (color_index == RED_IDX){
		set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_OFF);
		return;
	}
	else {
		if (color_index == GREEN_IDX){
			set_rgb_led(LED_RGB_2, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_4, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_6, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_8, LED_OFF, LED_ON, LED_OFF);
			return;
		}
		else {
			if (color_index == BLUE_IDX){
				set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_ON);
				return;
			}
			else {
				if (color_index == YELLOW_IDX){
					set_rgb_led(LED_RGB_2, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_4, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_6, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_8, LED_ON, LED_ON, LED_OFF);
					return;
				}
				else {
					if (color_index == PURPLE_IDX){
						set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_ON);
						return;
					}
					else {
						if (color_index == NO_LINE){
							set_rgb_led(LED_RGB_2, LED_OFF, LED_ON, LED_ON);
							set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_OFF);
							set_rgb_led(LED_RGB_6, LED_OFF, LED_ON, LED_ON);
							set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_OFF);
							return;
						}
						else {
							if (color_index == FIND_COLOR){
								set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_OFF);
								set_rgb_led(LED_RGB_4, LED_OFF, LED_ON, LED_OFF);
								set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_ON);
								set_rgb_led(LED_RGB_8, LED_ON, LED_ON, LED_ON);
								return;
							}
							else {
								if (color_index == NO_COLOR){
									set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_OFF);
									set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_OFF);
									set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_OFF);
									set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_OFF);
									return;
								}
							}
						}
					}
				}
			}
		}
	}
}

//Function for visualization
void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

//Security function
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}

