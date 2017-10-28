#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include "JOYSTICK_CLIENT.h"

static char recvBuff[255];
struct sockaddr_in serv_addr; 

char btn[20];
int axis[20];
char n_btn  = 0;
char n_axis = 0;


int get_ax(char ax_number)
{
	return axis[ax_number];
}

char get_btn(char btn_number)
{
	return btn[btn_number];
}

char get_ax_count()
{
	return n_axis;
}

char get_btn_count()
{
	return n_btn;
}



int ask_joypad_update(char ip[])
{
    int sockfd = 0, n = 0, counter;

    memset(recvBuff, '0',sizeof(recvBuff));
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error : Could not create socket for joystick\n");
        return -1;
    } 


    memset(&serv_addr, '0', sizeof(serv_addr)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5000); 



    if(inet_pton(AF_INET, ip, &serv_addr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return -1;
    } 


  
    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\n Error : Connect failed to joystick\n");
       return -1;
    }

    while ( (n = read(sockfd, recvBuff, sizeof(recvBuff)-1)) > 0)
    {
        recvBuff[n] = 0;

	n_axis = (recvBuff[3] * 10) + recvBuff[4] - 528;	//528 is for cast from ascii to char
	n_btn  = (recvBuff[10] * 10) + recvBuff[11] - 528;


	// get data from all axes and put into axis array
	for(counter = 0; counter < n_axis; counter++)
	{
		axis[counter] = (recvBuff[14+(counter*7)]-48)*10000 +
				(recvBuff[15+(counter*7)]-48)*1000 +
				(recvBuff[16+(counter*7)]-48)*100 +
				(recvBuff[17+(counter*7)]-48)*10 +
				(recvBuff[18+(counter*7)]-48);
		if ((recvBuff[13+(counter*7)]) == '-')
			axis[counter] = axis[counter] * -1;
	}

	// get data from btn's, and put into btn array
	char offset = 13+(n_axis*7);
	for(counter = 0; counter < n_btn; counter++)
	{
		btn[counter] = recvBuff[offset+(counter*2)]-48;
	}


//        if(fputs(recvBuff, stdout) == EOF)
//            printf("\n Error : Fputs error\n");
    }

    if(n < 0)
    {
        printf("\n Read error \n");
    }

    close(sockfd);
    return 1;

}

