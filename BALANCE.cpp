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
#include "mongoose.h"
#include "time.h"

#define servo_offset -30

struct mg_server *server;




// gyro variable
MPU6050 accelgyro;

// processes
pthread_t  thread_espeak;
pthread_t  thread_balance;
pthread_t  thread_joystick;
pthread_t  thread_webserver;


char ip[] = "192.168.0.102";
char thread_stop = 0;
char flash = 0;
char servo = 127;
char sensor_sharp_camunit=0;
short int ledbar = 0;

char joypad_ask_update = 1;	// start with an update

signed int enc1,enc2;




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
	delay(10);
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
//void timer_handler (int signum)
void timer_handler()
{

	#define P_ANGLE 35 //110
	#define I_ANGLE 30 //70
	#define P_SPEED 200 //110
	#define I_SPEED 20

	static signed long  angle     = 0;
	static signed long  i_angle   = 0;
//	static signed int   angle_out = 0;
	static signed long  i_speed   = 0;
	static signed long speed_old = 0;

	static signed int   speed_out = 0;

	static int speed = 0;
	static int steer = 0;
	static char joypad_time_out_counter = 0;
	static char joypad_refresh = 0;


	signed short int *p;



	if (joypad_ask_update == 0)					// is ther no update in progress?
	{
		speed = get_ax(1);
		steer = get_ax(0);

		joypad_time_out_counter = 0;				// clear time out
		joypad_ask_update = 1;					// ask new update, if update is done than this bit wil be automaticly cleared
	}
	else								// watch out, update in progress!
	{
		if (joypad_time_out_counter > 10)
		{
			speed = 0; steer = 0; servo = 0; flash = 0;
		}
		else
		{
			joypad_time_out_counter++;
		}
	}



	// PID loop of gyro


	char i;
	int gyro_mean = 0;
	int acc;


	for(i = 0; i<10; i++)
	{
		gyro_mean += accelgyro.getRotationX();
	}

	gyro_mean = (gyro_mean/10)-383;

//	acc = accelgyro.getAccelerationX()/50;
	//acc = (acc*-1);


/*	angle += gyro_mean;								// integrate angle for PI regulator

	signed int sum_angle_speed;
	sum_angle_speed = angle + 	((P_SPEED * speed_old) + (I_SPEED * i_speed))/50000;

	i_angle =+ sum_angle_speed;
	speed_out = (sum_angle_speed * P_ANGLE) + (i_angle * I_ANGLE);		// PID regulator

	i_speed += speed_out;
	speed_old = speed_out;


	angle += (speed_out / 4000);
*/
	angle += (gyro_mean)/250;								// integrate angle for PI regulator


	int angle_error = 0;

	angle_error = angle + speed_old + i_speed/5;

	i_angle =+ angle_error;
	speed_out = (angle_error * P_ANGLE) + (i_angle * I_ANGLE);		// PID regulator

//	speed_old = speed_out/500;



	speed_old+= speed_out/1000;
	i_speed+=speed_old;

	speed_out = speed_out *-1;								// adapt signal for transfer to H-bridge





//		speed_out = 0;



	if (speed_out > 16383 || speed_out < -16383)						// saftey limitation!
	{
		motor_update_power(0,0);
		printf("Overflow in motor!! check system\n");
		printf("i angle = %d",i_angle);
		delay(100);
		shutdown(0);
		while(1);
	}


	steer = steer / 20;
	motor_update_power(speed_out-steer,speed_out+steer);


	//***************************** update other bus parameters **************************//

        interface_set_flashlight(flash);

	p = motor_update_encoder();
	enc1 += *p;
	enc2 += *(p+1);

	int servo_position = servo + servo_offset;
//	int servo_position = servo + servo_offset + (i_angle/500);
	if (servo_position > 255)
		servo_position = 255;
	else if (servo_position < 0)
		servo_position = 0;

        interface_set_servo(servo_position);
	interface_set_led(ledbar);
	sensor_sharp_camunit = interface_ask_distance();

}



//******************************************************
//	This process will make the joystick readout asynchrone (double buffer
//	To make this work, you have to run the joystick server software on the host PC
//****************************************************/
void* joystick(void * thread_id)
{
	static char error_counter = 0;


	if(ask_joypad_update(ip) == 1) // update succesfull?
	{
		printf("Joystick server found\n");
	}
	else
	{
		printf("No joystick server found :-(\n");
		sleep(1);
		pthread_exit(NULL);
	}







	while (thread_stop == 0)
	{
		if (joypad_ask_update)
		{
			if(ask_joypad_update(ip) == 1) // update succesfull?
			{
				joypad_ask_update = 0;
				error_counter = 0;
			}
			else
			{
				if (error_counter > 3)
				{
					printf("Disable joystick input!!!");
					pthread_exit(NULL);
				}
				error_counter++;
			}
		}
	}
	pthread_exit(NULL);
}



// This function will be called by mongoose on every new request
static int index_html(struct mg_connection *conn) {
  	//mg_printf_data(conn, "Hello! Requested URI is [%s]", conn->uri);
	mg_printf_data(conn, "Head distance = %d cm\nJoystick data = %d,%d\nEncoder = %d,%d", sensor_sharp_camunit,get_ax(1),get_ax(0),enc1,enc2);
 	return MG_REQUEST_PROCESSED;
}


// see layout of index page in index_html function above
void* webserver(void* thread_id)
{
	while (thread_stop == 0)
		mg_poll_server(server, 1000);
}



void* balance(void* thread_id)
{
	printf("Execute balance software\n");

	while (thread_stop == 0)
	{
		timer_handler();
		usleep(10000);
	}

	pthread_exit(NULL);
}





main(int argc, char* argv[])
{
	int rc;
	(void) signal(SIGINT, shutdown);		// interrupt routine

	delay(3000);

	// ------------------------------- set up webserver ------------------------------------------------

	// Create and configure the server
	server = mg_create_server(NULL);
	mg_set_option(server, "listening_port", "8081");
	mg_set_request_handler(server, index_html);

	// Serve request. Hit Ctrl-C to terminate the program
	printf("Starting on port %s\n", mg_get_option(server, "listening_port"));

	rc = pthread_create(&thread_webserver, NULL, webserver, (void *)6);
      	if (rc)
	{
		printf("error creating thread for webserver");
		exit(-1);
	}




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


	printf("start creating threads\n");



	rc = pthread_create(&thread_joystick, NULL, joystick, (void *)5);
	if (rc)
	{
		printf("error creating thread for joystick\n");
		exit(-1);
	}



	rc = pthread_create(&thread_balance, NULL, balance, (void *)3);
      	if (rc)
	{
		printf("error creating thread for balancing pid\n");
		exit(-1);
	}





	char btn_old[20];
	char led_counter = 0;
	char led_direction = 0;


	while (1)	// main loop of non critical timing
	{

		usleep(250000);



		// led bar movement

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

		ledbar = 1<<led_counter;




		if((get_btn(0) == 1) && (btn_old[0] == 0))
		{
			btn_old[0] = 1;
			system("espeak \"Hello, how are you\"");
		}
		else
		{
			btn_old[0] = 0;
		}

		if((get_btn(1) == 1) && (btn_old[1] == 0))
		{
			btn_old[1] = 1;
			system("espeak \"please let me pass\"");

		}
		else
		{
			btn_old[1] = 0;
		}

		if((get_btn(2) == 1) && (btn_old[2] == 0))
		{
			btn_old[2] = 1;
			system("espeak \"you stupid moron\"");

		}
		else
		{
			btn_old[2] = 0;
		}

		if((get_btn(3) == 1) && (btn_old[3] == 0))
		{
			btn_old[3] = 1;
			system("espeak \"i love you\"");

		}
		else
		{
			btn_old[3] = 0;
		}


		if((get_btn(4) == 1) && (btn_old[4] == 0))
		{
			btn_old[4] = 1;
			flash = (get_ax(2)+32767)/256;

		}
		else
		{
			btn_old[4] = 0;
		}

		if((get_btn(5) == 1) && (btn_old[5] == 0))
		{
			btn_old[5] = 1;
			servo = (get_ax(2)+32767)/256;

		}
		else
		{
			btn_old[5] = 0;
		}

	}
}
