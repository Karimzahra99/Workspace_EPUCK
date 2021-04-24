#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define LINEWIDTH				100		//en fonction de la ligne imprimmer, a voir si utile
#define TOLERANCE				3
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define KD						25.0f	//a tuner -> utiliser deuxieme methode de ZN avec Ku et Pu
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define WHEEL_PERIMETER     	13 		//[cm]
#define NSTEP_ONE_TURN      	1000	// number of step for 1 turn of the motor

//Vertical index of line (0 to 480) 0 : highest, 479 :lowest (due to camera library we take two lines)
#define LINE_INDEX					25

//Le nombre minimum de pixel pour valider une detection de ligne pour une certaine couleur
#define MIN_COUNT					5

//Valeur maximal d'intensite d'un pixel (vert ramener sur [0,31])
#define MAX_VALUE					31

//Shift pour remettre les bits des couleurs dans l'ordre lors de l'extraction du format RGB565
#define SHIFT_3						3
#define SHIFT_5						5

//Index des couleurs / utiliser enum ?
#define RED_IDX						1
#define GREEN_IDX					2
#define BLUE_IDX					3

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
