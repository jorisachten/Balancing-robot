#ifndef COMMUNICATION_BUS
	#define COMMUNICATION_BUS

	#include <wiringPi.h>
	#include <wiringPiSPI.h>
	#include <stdio.h>
	#include "wiringPiSPI.h"

	// defines for IO
	#define ADRESS_0 		3
	#define ADRESS_1 		2
	#define ADRESS_2 		0
	#define ADRESS_3 		7
	#define RESET_WRITE 		6
	#define SPI_CHANNEL 		0
	#define SPI_SPEED   		5000


	// defines for devices
	#define POWER_UNIT_ID		10
	#define POWER_UNIT_ADRESS	1
	#define POWER_UNIT_SPEED	30000

	#define PI_UNIT_ID		11
	#define PI_UNIT_ADRESS		2
	#define PI_UNIT_SPEED		400000


	// other defines
	#define ADRESS			0
	#define ID			1
	#define SPEED			2



	//-------- communication functions -------------
	char init_communication_bus();
	void set_communication_adress(char adress);
	void write_spi_buffer(unsigned int* device, unsigned char* buffer, int buffer_size);
	void force_reset(char value);
	char scann_devices();

	//-------- interface functions -------------
	void interface_set_led(unsigned int value);
	void interface_set_backlight(char value);
	void interface_set_flashlight(char value);
	void interface_set_servo(char value);
	char interface_ask_distance();
	void interface_lcd_clear();					// LCD operations need 50ms to be executed!!!
	void interface_lcd_text(char x, char y, char* text);
	void interface_lcd_image(char * data);

	//-------- motor functions -------------
	signed short int* motor_update_encoder();
	void motor_update_power(signed int R, signed int L);


#endif
