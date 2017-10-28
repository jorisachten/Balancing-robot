#include <string.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <unistd.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "COMMUNICATION_BUS.h"
#include <sys/time.h>
#include <unistd.h>
#include "JOYSTICK_CLIENT.h"
#include <stdlib.h>
#include <pthread.h>

#define servo_offset -30


// gyro variable
MPU6050 accelgyro;

// processes
pthread_t  thread_espeak;
pthread_t  thread_balance;
pthread_t  thread_joystick;


char ip[] = "192.168.0.100";
char thread_stop = 0;


int joypad_steer = 0;
int joypad_speed = 0;
int joypad_flash = 0;
int joypad_servo = 0;
int joypad_ask_update = 1;	// start with an update

char flash = 0;
char servo = 0;
int speed = 0;
int steer = 0;
char joypad_time_out_counter = 0;
char joypad_refresh = 0;




//******************************************************
//	This function will shut down all the hardware and software of the task
//	Motors will stop imiditly, cam servo will move to a safe position, warning will be displayed on LCD
//	After 3 seconds all harware will go down in power saving mode
//****************************************************/

void shutdown(int sig)
{
	char error_image[1024] = {
  	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 192, 240, 252, 254, 254, 255, 255, 255, 254, 254, 252, 240, 192,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 128, 224, 248, 252, 255, 255, 255, 255, 255, 255, 127,  63, 127, 255, 255, 255, 255, 255, 255, 252, 248, 224, 128,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 192, 240, 248, 254, 255, 255, 255, 255, 255, 255,  63,  31,   7,   1,   0,   0,   0,   1,   7,  31,  63, 255, 255, 255, 255, 255, 255, 254, 248, 240, 192,   0,   0,   0,   0,   0,   0, 252,   0,   0,   0,   0, 252,   0,   0, 252,  12,  48, 192,   0, 252,   0,   0, 252,  36,  36,  36,   0,   4,   8, 144,  96, 144,   8,   4,   0, 252,  68,  68,  68,  56,   0,   0, 252,  36,  36,  36,   0,   0, 240,   8,   4,   4,   4,   8,   4,   4, 252,   4,   4,   0,   0, 252,  36,  36,  36,   0,   0, 252,   4,   4,   4,   8, 240,   0,   0,   0,   0, 
  	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 128, 192, 240, 252, 255, 255, 255, 255, 255, 255, 127,  63,  15,   3,   0,   0,   0,   0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   3,  15,  63, 127, 255, 255, 255, 255, 255, 255, 252, 240, 192, 128,   0,   1,   2,   2,   2,   2,   1,   0,   0,   3,   0,   0,   0,   3,   3,   0,   0,   3,   2,   2,   2,   0,   2,   1,   0,   0,   0,   1,   2,   0,   3,   0,   0,   0,   0,   0,   0,   3,   2,   2,   2,   0,   0,   0,   1,   2,   2,   2,   1,   0,   0,   3,   0,   0,   0,   0,   3,   2,   2,   2,   0,   0,   3,   2,   2,   2,   1,   0,   0,   0,   0,   0, 
  	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 128, 224, 248, 254, 255, 255, 255, 255, 255, 255,  63,  31,   7,   1,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   1,   7,  31,  63, 255, 255, 255, 255, 255, 255, 254, 248, 224, 128,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 254,  16,  16,  16,  16, 254,   0, 128,  96,  88,  70,  88,  96, 128,   0, 254,   0,   0,   0,   2,   2, 254,   2,   2,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  	0,   0,   0,   0,   0,   0,   0, 192, 240, 252, 255, 255, 255, 255, 255, 255, 127,  63,  15,   3,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 225, 243, 243, 243, 225,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   3,  15,  63, 127, 255, 255, 255, 255, 255, 255, 252, 240, 192,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0,   1,   0,   1,   0,   0,   0,   0,   0,   1,   0,   1,   1,   1,   1,   0,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  	0,   0, 128, 224, 248, 254, 255, 255, 255, 255, 255, 255, 191, 159, 135, 129, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 129, 129, 129, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 129, 135, 159, 191, 255, 255, 255, 255, 255, 255, 254, 248, 224, 128,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
 	28, 127, 127, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 127, 127,  28,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0
	};


	printf("Shut Down!!! - Signal %d\n", sig);
	thread_stop=1;
	delay(100);
	motor_update_power(0,0);
	interface_set_led(0);
        interface_set_backlight(255);
        interface_set_flashlight(0);
        interface_set_servo(0);

        interface_lcd_image(error_image);

	delay(3000);


	force_reset(1);
	delay(100);


	// kill all pid's
	kill(getpid(), SIGTERM);
}



//******************************************************
//	This function is the timer from the balance process
//	It will run every 100 ms
//	This function calculates the PID of standing up and communicate all parameters over the communication bus
//****************************************************/
void timer_handler (int signum)
{
	(void) signal(SIGINT, shutdown);		// interrupt routine


	#define P_ANGLE 110
	#define I_ANGLE 70

	static signed int   angle     = 0;
	static signed long  angle_err = 0;
	static signed long  i_angle   = 0;
	static signed int   angle_out = 0;


	static signed int   speed_out = 0;
	static signed int   d_speed_out = 0;

	static signed long distance = 0;
	static signed long Idistance;




	signed short int encoderR;
	signed short int encoderL;
	signed short int *p;



	if(thread_stop)				// is the execution halted?
		return;




//	p = motor_update_encoder();
//	encoderR = *p;
//	encoderL = *(p+1);

//	distance = encoderR + encoderL;
//	Idistance += distance;

	angle = angle + (accelgyro.getRotationX() - 383) - ((speed_out - d_speed_out)/4 + speed_out/15) - speed/100; // integrate gyro signal to becoma angle and not angle speed
	d_speed_out = speed_out;


	angle_err = angle;
	i_angle = i_angle + angle_err;								// integrate angle for PI regulator
	speed_out = (angle_err * P_ANGLE) + (i_angle * I_ANGLE);				// PID regulator

	speed_out = 0;


	speed_out = speed_out *-1;								// adapt signal for transfer to H-bridge
	speed_out = speed_out/150;



	if (speed_out > 16383 || speed_out < -16383)						// saftey limitation!
	{
		motor_update_power(0,0);
		printf("Overflow in motor!! check system\n");
		delay(100);
		shutdown(0);
		while(1);
	}




	steer = steer / 20;
	motor_update_power(speed_out-steer,speed_out+steer);




	//***************************** update other bus parameters **************************//

        interface_set_flashlight(flash);


	int servo_position = servo + servo_offset + (i_angle/700);


	if (servo_position > 255)
		servo = 255;
	else if (servo_position < 0)
		servo = 0;

        interface_set_servo(servo_position);


	// led
	static char led_counter = 0;
	static char led_direction = 0;
	if (led_direction == 0)
	{
		led_counter ++;
		if (led_counter == 15)
			led_direction = 1;
	}
	else
	{
		led_counter --;
		if (led_counter == 0)
			led_direction = 0;
	}

	interface_set_led(1<<led_counter);


}



//******************************************************
//	This process will read every 250 ms the joystick data from a given ip adress
//	To make this work, you have to run the joystick server software on the host PC
//****************************************************/
void* joystick(void * thread_id)
{


	char speed = 0;
	char steer = 0;

	while (thread_stop == 0)
	{
		if (joypad_ask_update)
		{
			if(ask_joypad_update(ip) == 1) // update succesfull?
			{
				joypad_steer = get_ax(0);
				joypad_speed = get_ax(1);
				joypad_flash = get_ax(2);
				joypad_servo = get_ax(3);
				joypad_ask_update = 0;
			}
		}

	}

	pthread_exit(NULL);
}


struct espeak_data{
   int  thread_id;
   const char *message;
};

void* espeak(void* text)
{
	struct espeak_data *my_data;
	my_data = (struct espeak_data *) text;


	char command[512];
	strcat(command,"espeak \"");
	strcat(command, my_data->message);
	strcat(command,"\"");


	system((const char*)command);

	pthread_exit(NULL);
}




void* balance(void* thread_id)
{
	// timer interrupt variable
	struct sigaction sa;
	struct itimerval timer;



	// -------------------------------- init timer ---------------------------------------------------------

	printf("Init timer\n");
	/* Install timer_handler as the signal handler for SIGVTALRM. */
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = &timer_handler;
	sigaction (SIGVTALRM, &sa, NULL);

	/* Configure the timer to expire after 250 msec... */
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = 100000;
	/* ... and every 250 msec after that. */
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = 100000;
	/* Start a virtual timer. It counts down whenever this process is
	  executing. */
	setitimer (ITIMER_VIRTUAL, &timer, NULL);

	// -------------------------------- end of init ---------------------------------------------------------

	printf("Execute Program\n");



	while (thread_stop == 0);

	pthread_exit(NULL);
}





main(int argc, char* argv[])
{
	int rc;


	// -------------------------------- init IIC ---------------------------------------------------------
        printf("Initializing I2C devices...\n");
        accelgyro.initialize();

        // verify connection
        printf("Testing device connections...\n");
        printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
        delay(100);



	// -------------------------------- init spi and communication bus ---------------------------------------------------------
	if (init_communication_bus() == 0)		// init HW pins of pi
	{
		printf("init of the communication bus failed!\n");
	}


	printf("Release reset of communication bus...\n");
	force_reset(0);
	delay(1000);

	printf("Testing device connections...\n");
	if(scann_devices() == 0)
	{

		char image_device_error[1024] = {
  		0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 192, 240, 252, 254, 254, 255, 255, 255, 254, 254, 252, 240, 192,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  		0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 128, 224, 248, 252, 255, 255, 255, 255, 255, 255, 127,  63, 127, 255, 255, 255, 255, 255, 255, 252, 248, 224, 128,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  		0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 192, 240, 248, 254, 255, 255, 255, 255, 255, 255,  63,  31,   7,   1,   0,   0,   0,   1,   7,  31,  63, 255, 255, 255, 255, 255, 255, 254, 248, 240, 192,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 252,   4,   4,   4,   4,   8, 240,   0,   0, 252,  68,  68,  68,  68,  68,  68,   0,  12,  48, 192,   0,   0,   0, 192,  48,  12,   0, 252,   0,   0, 240,   8,   4,   4,   4,   8,  16,   0,   0, 252,  68,  68,  68,  68,  68,  68,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  		0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 128, 192, 240, 252, 255, 255, 255, 255, 255, 255, 127,  63,  15,   3,   0,   0,   0,   0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   3,  15,  63, 127, 255, 255, 255, 255, 255, 255, 252, 240, 192, 128,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  15,   8,   8,   8,   8,   4,   3,   0,   0,  15,   8,   8,   8,   8,   8,   8,   0,   0,   0,   0,   3,  12,   3,   0,   0,   0,   0,  15,   0,   0,   3,   4,   8,   8,   8,   4,   2,   0,   0,  15,   8,   8,   8,   8,   8,   8,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  		0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 128, 224, 248, 254, 255, 255, 255, 255, 255, 255,  63,  31,   7,   1,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   1,   7,  31,  63, 255, 255, 255, 255, 255, 255, 254, 248, 224, 128,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 252,  68,  68,  68,  68,  68,  68,   0,   0, 252, 132, 132, 132, 132, 132, 120,   0,   0, 252, 132, 132, 132, 132, 132, 120,   0,   0, 240,   8,   4,   4,   4,   4,   8, 240,   0,   0, 252, 132, 132, 132, 132, 132, 120,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  		0,   0,   0,   0,   0,   0,   0, 192, 240, 252, 255, 255, 255, 255, 255, 255, 127,  63,  15,   3,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 225, 243, 243, 243, 225,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   3,  15,  63, 127, 255, 255, 255, 255, 255, 255, 252, 240, 192,   0,   0,   0,   0,   0,   0,   0,   0,   0,  15,   8,   8,   8,   8,   8,   8,   0,   0,  15,   0,   0,   0,   1,   6,   8,   0,   0,  15,   0,   0,   0,   1,   6,   8,   0,   0,   3,   4,   8,   8,   8,   8,   4,   3,   0,   0,  15,   0,   0,   0,   1,   6,   8,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  		0,   0, 128, 224, 248, 254, 255, 255, 255, 255, 255, 255, 191, 159, 135, 129, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 129, 129, 129, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 129, 135, 159, 191, 255, 255, 255, 255, 255, 255, 254, 248, 224, 128,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
 		28, 127, 127, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 127, 127,  28,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0
		};

	        interface_set_backlight(20);

		delay(50);
		interface_lcd_image(image_device_error);
		delay(50);

		printf("EXIT PROGRAM !!!\n");
		force_reset(1);
		delay(100);
		while(1);
	}

	// do some gyro measerments to readout zero's
	accelgyro.getRotationX();
	accelgyro.getRotationX();
	accelgyro.getRotationX();







	printf("start creating threads");



	rc = pthread_create(&thread_balance, NULL, balance, (void *)3);
      	if (rc)
	{
		printf("error creating thread for espeak");
		exit(-1);
	}

	rc = pthread_create(&thread_joystick, NULL, joystick, (void *)5);
      	if (rc)
	{
		printf("error creating thread for espeak");
		exit(-1);
	}



	while (1)
	{

		struct espeak_data espeak_td;
		char btn_old[20];


		if (joypad_ask_update == 0)		// is ther no update in progress?
		{
			flash = (joypad_flash+32767)/256;
			servo = (joypad_servo+32767)/256;
			speed = joypad_speed;
			steer = joypad_steer;
			joypad_time_out_counter = 0;
			joypad_refresh ++;
			if (joypad_refresh > 1)
			{
				joypad_ask_update = 1;
				joypad_refresh = 0;
			}
		}
		else					// watch out, update in progress!
		{
			if (joypad_time_out_counter > 10)
			{
				speed = 0;
				steer = 0;
				servo = 0;
				flash = 0;
			}
			else
			{
				joypad_time_out_counter++;
			}

		}








		if((get_btn(0) == 1) && (btn_old[0] == 0))
		{
			btn_old[0] = 1;
			system("espeak \"shut up\" ");
		}
		else if((get_btn(1) == 1) && (btn_old[1] == 0))
		{
			btn_old[1] = 1;
			system("espeak \"open the god dem door\" ");
		}

		else
		{
			btn_old[0] = 0;
		}
		usleep(300000);

	}
}
