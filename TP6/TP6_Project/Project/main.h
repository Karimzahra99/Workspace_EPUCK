#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//Comment after tuning the color detection parameters in read_image.c
//#define TUNE

//Difference between middle top and bottom line neglected (used in read_image.c and pid_regulator.c)
#define DEAD_ZONE_WIDTH				100

typedef enum {
	MAX_ONLY = 0,
	MEAN_ONLY,
	MAX_N_MEAN
} detect_mode_t;

typedef enum {
	NO_COLOR = 0,
	RED_IDX,
	GREEN_IDX,
	BLUE_IDX,
	YELLOW_IDX,
	PURPLE_IDX
} color_index_t;

typedef enum {
	NO_VISUALIZE_PARAMS = 0,
	YES_VISUALIZE_PARAMS,
} visualize_mode_t;

typedef struct {
	uint8_t contrast;
	uint16_t line_idx;
	detect_mode_t detection_mode;
	color_index_t color_idx;
	visualize_mode_t send_data_terminal;
} tuning_config_t;

typedef struct {
	uint8_t contrast;
	uint16_t line_idx_top;
	uint16_t line_idx_bot;
	detect_mode_t detection_mode;
	visualize_mode_t send_data_terminal;
} config_t;


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
