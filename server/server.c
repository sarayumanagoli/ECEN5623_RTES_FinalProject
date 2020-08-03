/* Standard C Library Headers */
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <syslog.h>

/* Defines */
#define PORT 8080

int size;
int imgnum = 1;
char server_image[] = "recv00000000.ppm";

int main(int argc, char const *argv[]) 
{ 
	int sockfd,newsockfd,bytes_recieved;
	struct sockaddr_in server_addr;
	int opt = 1;
	int server_len = sizeof(server_addr);	
	unsigned char buffer[921600];	
	FILE *file_ptr;			
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
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

	int i = 0;

	while(1)
	{
		size = 0;

		do{
			bytes_recieved = recv(newsockfd,(char *)&buffer, 921600,0);
			if(bytes_recieved == 0)
				printf("\nNo bytes received\n");
			else
				size += bytes_recieved;

			if(i == 0)
			{
				snprintf(&server_image[4], 9, "%08d", imgnum);
				strncat(&server_image[12], ".ppm", 5);
				file_ptr = fopen(server_image,"w+");	
				i++;							
			}

			int write_size = fwrite(buffer,1,bytes_recieved,file_ptr);		

		}while(size < 921600);

		fclose(file_ptr);
		i = 0;
		imgnum++;
	}

	return 0; 
} 
