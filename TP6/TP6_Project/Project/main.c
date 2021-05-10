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

//Distances parameters
#define PI                 			3.1415926536f
#define WHEEL_PERIMETER     		13 		//[cm]
#define NSTEP_ONE_TURN      		1000	// number of step for 1 turn of the motor
#define WHEEL_DISTANCE      		5.30f    //cm
#define PERIMETER_EPUCK     		(PI * WHEEL_DISTANCE)


static	int x = 0;
static	int y = 0;
static int old_pos_r = 0;
static int old_pos_l = 0;

void motor_set_position_test(float position_r, float position_l, int16_t speed_r, int16_t speed_l);
int cm_to_step_test (float cm);
int step_to_cm_test (int step);
int16_t cms_to_steps_test (int16_t speed_cms);

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
	 * line_index_top : [0 480]
	 * mode_detect : MAX_ONLY, MEAN_ONLY, MAX_N_MEANS
	 * plot_pixels_color : RED_IDX, GREEND_IDX, BLUE_IDX
	 * send_params : NO_VISUALIZE_PARAMS, VISUALIZE_PARAMS
	 */

//	rgb_gain_t rgb_gains = {94, 80, 80};
//	uint8_t contrast = 85;
//	//tuning uses line_index_top for plot visualization
//	uint16_t line_index_top = 10;
//	detect_mode_t mode_detect = MAX_ONLY;
//	visualize_mode_t send_params = YES_VISUALIZE_PARAMS;
//#ifdef TUNE
//	//chose which color intensity to plot with plot_image.py
//	color_index_t plot_pixels_color = BLUE_IDX;
//#else
//	uint16_t line_index_bot = 400;
//#endif
//
//
//	//TUNE is defined in main.h
//#ifdef TUNE
//	//Contrast level, camera line index, detect_mode, image color, visualize parameters such as means, maxs, counts on terminal
//	//Adjust Contrast, Line_Idx and detection mode  in Main.h
//	tuning_config_t tunning = {rgb_gains, contrast, line_index_top, mode_detect, plot_pixels_color, send_params};
//	tune_image_start(tunning);
//#else
//	//inits the motors
	motors_init();
//
//	config_t config = {rgb_gains, contrast, line_index_top, line_index_bot, mode_detect, send_params};
//	read_image_start(config);
//
//	proximity_start();
//
//	moving_start();
//
//
//#endif

//	while (1) {
//
//
//
//		//waits 1 second
//		chThdSleepMilliseconds(1000);
//	}






	while(1){

		x = 0;
		y = 0;

		old_pos_r = 0;
		old_pos_l = 0;

		left_motor_set_pos(0);
		right_motor_set_pos(0);

		motor_set_position_test(10, 10, 4, 4);
		chThdSleepMilliseconds(2000);
		motor_set_position_test(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2, 3,-3);
		chThdSleepMilliseconds(2000);
		motor_set_position_test(10, 10, 4, 4);
		chThdSleepMilliseconds(2000);
		motor_set_position_test(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2, 3,-3);
		chThdSleepMilliseconds(5000);
	}

}


static uint8_t position_reached = 0;

void motor_set_position_test(float position_r, float position_l, int16_t speed_r, int16_t speed_l){

	position_reached = 0;

	int d_center = 0;
	int theta = 0;

	left_motor_set_pos(0);
	right_motor_set_pos(0);

	int position_to_reach_left = cm_to_step_test(position_l);
	int position_to_reach_right = - cm_to_step_test(position_r);

	left_motor_set_speed(cms_to_steps_test(speed_l));
	right_motor_set_speed(cms_to_steps_test(speed_r));

	while (!position_reached){

		d_center = (right_motor_get_pos() + left_motor_get_pos())/2;
		theta = (right_motor_get_pos() - left_motor_get_pos())/WHEEL_DISTANCE;

		x = x+ d_center*cos(theta);
		y = y + d_center*sin(theta);

		chprintf((BaseSequentialStream *)&SD3, "x =%-7d y =%-7d theta =%-7d \r\n\n",
							step_to_cm_test(x),step_to_cm_test(y), theta);

		chprintf((BaseSequentialStream *)&SD3, "rpos =%-7d lpos =%-7d \r\n\n",
				right_motor_get_pos(),left_motor_get_pos());


		if (abs(right_motor_get_pos()) > abs(position_to_reach_right) && abs(left_motor_get_pos()) > abs(position_to_reach_left) ){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			position_reached = 1;

			d_center = (right_motor_get_pos() + old_pos_r + left_motor_get_pos() + old_pos_l)/2;
			theta = (right_motor_get_pos() + old_pos_r - left_motor_get_pos() - old_pos_l)/WHEEL_DISTANCE;

			x = d_center*cos(theta);
			y = d_center*sin(theta);

			old_pos_r = old_pos_r + right_motor_get_pos();
			old_pos_l = old_pos_l + left_motor_get_pos();


			chprintf((BaseSequentialStream *)&SD3, "x =%-7d y =%-7d theta =%-7d \r\n\n",
										step_to_cm_test(x),step_to_cm_test(y), theta);

					chprintf((BaseSequentialStream *)&SD3, "rpos =%-7d lpos =%-7d \r\n\n",
							right_motor_get_pos(),left_motor_get_pos());
		}
	}
}




int step_to_cm_test (int step) {
	return step * WHEEL_PERIMETER / NSTEP_ONE_TURN;
}

int16_t cms_to_steps_test (int16_t speed_cms) {
	return speed_cms * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

int cm_to_step_test (float cm) {
	return (int)(cm * NSTEP_ONE_TURN / WHEEL_PERIMETER);
}



#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}

//Functions for communication and visualization
void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

void set_leds(color_index_t color_index){
	if (color_index == RED_IDX){
		set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_OFF);
	}
	else {
		if (color_index == GREEN_IDX){
			set_rgb_led(LED_RGB_2, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_4, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_6, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_8, LED_OFF, LED_ON, LED_OFF);
		}
		else {
			if (color_index == BLUE_IDX){
				set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_ON);
			}
			else {
				if (color_index == YELLOW_IDX){
					set_rgb_led(LED_RGB_2, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_4, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_6, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_8, LED_ON, LED_ON, LED_OFF);
				}
				else {
					if (color_index == PURPLE_IDX){
						set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_ON);
					}
					else {
						if (color_index == NO_COLOR){
							set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_OFF);
							set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_OFF);
							set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_OFF);
							set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_OFF);
						}
					}
				}
			}
		}
	}
}
