#include "COMMUNICATION_BUS.h"


unsigned int power_unit[] = {POWER_UNIT_ADRESS	,POWER_UNIT_ID	,POWER_UNIT_SPEED};
unsigned int pi_unit[]	  = {PI_UNIT_ADRESS	,PI_UNIT_ID	,PI_UNIT_SPEED};

unsigned char buffer[1025];

char init_communication_bus()
{
	if (wiringPiSetup() == -1)
	{
		printf("error on ADRESS_output init\n");
		return 0;
	}

	pinMode(ADRESS_0,OUTPUT);
	pinMode(ADRESS_1,OUTPUT);
	pinMode(ADRESS_2,OUTPUT);
	pinMode(ADRESS_3,OUTPUT);
	pinMode(RESET_WRITE,OUTPUT);

	if(wiringPiSPISetup(SPI_CHANNEL,SPI_SPEED) == -1)
	{
		printf("error on SPI init\n");
		return 0;
	}

	return 1;
}


void set_communication_adress(char adress)
{
	if(adress & 0x01)
		digitalWrite(ADRESS_0,HIGH);
	else
		digitalWrite(ADRESS_0,LOW);



	if(adress & 0x02)
		digitalWrite(ADRESS_1,HIGH);
	else
		digitalWrite(ADRESS_1,LOW);



	if(adress & 0x04)
		digitalWrite(ADRESS_2,HIGH);
	else
		digitalWrite(ADRESS_2,LOW);



	if(adress & 0x08)
		digitalWrite(ADRESS_3,HIGH);
	else
		digitalWrite(ADRESS_3,LOW);

	return;
}


void force_reset(char value)
{
	if(value)
		digitalWrite(RESET_WRITE,LOW);
	else
		digitalWrite(RESET_WRITE,HIGH);
	return;
}

void write_spi_buffer(unsigned int* device, unsigned char* buffer, int buffer_size)
{
	set_communication_adress(device[ADRESS]);
	wiringPiSPIChangeSpeed(SPI_CHANNEL, device[SPEED]);
	wiringPiSPIDataRW(SPI_CHANNEL,buffer,buffer_size);
	set_communication_adress(0);
	return;
}



char scann_devices()
{
	buffer[0] = 0;
	buffer[1] = 0;
	write_spi_buffer(power_unit,buffer,2);
	if (buffer[1] != power_unit[ID])
	{
		printf("MOTOR CONTROLLER NOT FOUND!!!\n");
		return 0;
	}
	else
	{
		printf("--> ADRESS %d, ID %d, SPEED %d \t:motor controller\n",power_unit[ADRESS],power_unit[ID],power_unit[SPEED]);
	}

	buffer[0] = 0;
	buffer[1] = 0;

	write_spi_buffer(pi_unit,buffer,2);
	if (buffer[1] != pi_unit[ID])
	{
		printf("PI UNIT NOT FOUND!!!\n");
		return 0;
	}
	else
	{
		printf("--> ADRESS %d, ID %d, SPEED %d \t:interface controller\n",pi_unit[ADRESS],pi_unit[ID],pi_unit[SPEED]);
	}

	return 1;
}



// ------------------------------ INTERFACE UNIT COMMANDS ---------------------------------------------------------

void interface_set_led(unsigned int value)
{
	buffer[0] = 1;		// command
	buffer[1] = ~(value>>8);
	buffer[2] = ~(value);
	write_spi_buffer(pi_unit,buffer,3);
	return;
}

void interface_set_backlight(char value)
{
	buffer[0] = 2;
	buffer[1] = value;
	write_spi_buffer(pi_unit,buffer,2);
	return;
}

void interface_set_flashlight(char value)
{
	buffer[0] = 3;
	buffer[1] = value;
	write_spi_buffer(pi_unit,buffer,2);
	return;
}

void interface_set_servo(char value)
{
	buffer[0] = 4;
	buffer[1] = value;
	write_spi_buffer(pi_unit,buffer,2);
	return;
}


char interface_ask_distance()
{
	double voltage;

	buffer[0] = 5;
	buffer[1] = 0;
	buffer[2] = 0;
	write_spi_buffer(pi_unit,buffer,3);

	voltage = (buffer[1]<<8)+buffer[2];

	voltage = (voltage * 5)/1024;			// convert to cm
	voltage = 306.439 + voltage *(-512.611 + voltage * (382.268 + voltage * (-129.893 + voltage * 16.2537)));

	if (voltage > 150)
		voltage = 150;


	return (char)voltage;
}


void interface_lcd_clear()
{
	buffer[0] = 6;
	buffer[1] = 0;
	write_spi_buffer(pi_unit,buffer,2);
	return;
}


void interface_lcd_text(char x, char y, char* text)
{
	char counter;

	buffer[0] = 7;
	buffer[1] = x;
	buffer[2] = y;

	for(counter = 3; counter < 35; counter++)		// max 32 characters, else non proper string ending!
	{
		if ((*text) != 0)
		{
			buffer[counter] = *text;
			text++;
		}
		else
		{
			break;
		}
	}
	buffer[counter] = 0;
	counter++;

	write_spi_buffer(pi_unit,buffer,counter);

	return;
}


void interface_lcd_image(char* data)
{
	int counter;
	buffer[0] = 8;
	for (counter = 1; counter < 1025; counter ++)
	{
		buffer[counter] = *data;
		data++;
	}

	set_communication_adress(pi_unit[ADRESS]);
	wiringPiSPIChangeSpeed(SPI_CHANNEL, pi_unit[SPEED]);
	wiringPiSPIDataRW(SPI_CHANNEL,buffer,1025);
	set_communication_adress(0);
	return;
}


// ------------------------------------------- motor controller software here --------------------------------------------
signed short int* motor_update_encoder()
{
	static signed short int encoder_value[2] = {0,0};

	buffer[0] = 1;			// take encoder sample
	buffer[1] = 2;			// ask encoder 1
	buffer[2] = 0;
	buffer[3] = 3;			// ask encoder 2
	buffer[4] = 0;
	buffer[5] = 0;

	write_spi_buffer(power_unit,buffer,6);

	encoder_value[1] = (buffer[3]<<8)+buffer[2];
	encoder_value[0] = (buffer[5]<<8)+buffer[4];


	return encoder_value;
}
void motor_update_power(signed int R, signed int L)
{
	buffer[0] = 4;
	buffer[1] = R;
	buffer[2] = R>>8;
	buffer[3] = 5;
	buffer[4] = L;
	buffer[5] = L>>8;
	buffer[6] = 6;
	write_spi_buffer(power_unit,buffer,7);

	return;
}
