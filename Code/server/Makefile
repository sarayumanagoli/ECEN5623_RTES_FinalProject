all:server

server: server.o 
	gcc -o server server.o 

server.o: server.c
	gcc -c server.c

clean:
	rm *.o  server
	rm *.ppm