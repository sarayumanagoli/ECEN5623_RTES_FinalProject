/* 
 * Edited by: Sarayu Managoli
 * Author: Dr. Sam Siewert
 * Overview: This file contains the source code for the server end of final project in the course ECEN5623 Real Time Operating Systems
 * Board Used: Raspberry Pi 3+ 
 * Code Leverage: Socket - https://www.geeksforgeeks.org/socket-programming-cc/
 */

//Header files
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <syslog.h>

#define PORT 8080


int main(int argc, char const *argv[]) 
{ 
	int sockfd,newsockfd,bytes_recieved;
	struct sockaddr_in server_addr;
	FILE *f;
	int opt = 1;
	int img_start = 0;
	int write_size = 0;
	int server_len = sizeof(server_addr);
	char server_image[] = "recv00000000.ppm";	
	unsigned char image_buffer[921600];
	int size;
	int imgnum = 1;	
				
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd <= 0)
	{
		syslog(LOG_ERR,"Socket function failed!");
		exit(EXIT_FAILURE);
	}

	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
				&opt, sizeof(opt)))
	{
		syslog(LOG_ERR,"ERROR setsockopt");
		exit(EXIT_FAILURE);
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = INADDR_ANY;
	server_addr.sin_port = htons( PORT );

	if (bind(sockfd, (struct sockaddr *) &server_addr,sizeof(server_addr)) < 0)
	{
		syslog(LOG_ERR,"ERROR during binding");
		exit(EXIT_FAILURE);
	}
	
	listen(sockfd,3);
	if (sockfd < 0)
	{
		syslog(LOG_ERR,"ERROR on listening");
		exit(EXIT_FAILURE);
	}
	
	
	newsockfd = accept(sockfd, (struct sockaddr *) &server_addr, (socklen_t *)&server_len);
	syslog(LOG_INFO,"Accepting");

	if (newsockfd < 0)
	{
		syslog(LOG_ERR,"ERROR on accepting connection");
		exit(EXIT_FAILURE);
	}


	while(1)
	{
		size = 0;

		do
		{
			bytes_recieved = recv(newsockfd,(char *)&image_buffer, 921600,0);
			if(bytes_recieved == 0)
				printf("\nNo bytes received\n");
			else
				size += bytes_recieved;

			if(img_start == 0)
			{
				//the above condition is checked to indicate the start of the image file
				img_start++;	
				
				//This definition is the same as that in dump_ppm function on the client side
				snprintf(&server_image[4], 9, "%08d", imgnum);
				strncat(&server_image[12], ".ppm", 5);
				
				//w+ is used to append to an existing file
				f = fopen(server_image,"w+");
				if(f == NULL)
				{
					syslog(LOG_ERR,"Error opening image file");		
				}							
			}

			if(fwrite(image_buffer,1,bytes_recieved,f) == 0)
			syslog(LOG_ERR,"Failed to write into the image buffer");		
		}while(size < 921600); //Until all bytes are received
		
		img_start = 0;
		imgnum++;
		fclose(f);
	}

	return 0; 
} 
