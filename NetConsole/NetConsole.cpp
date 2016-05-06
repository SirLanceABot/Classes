/*
 * NetConsole.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: Robotics
 */
// NetConsole Replacement
/* Server program example for IPv4 */
#include <cctype>
// Microsoft:#define WIN32_LEAN_AND_MEAN
// Microsoft:#pragma comment(lib, "Ws2_32.lib")
#include <winsock2.h>
// Old Way: Properties / C/C++ Build / Settings / Tool Settings / MinGW C++ Linker / Miscellaneous    "C:\MinGW\\lib\libws2_32.a"
// Properties / C/C++ Build / Settings / Tool Settings / MinGW C++ Linker / Libraries -l + ws2_32
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define DEFAULT_PORT 6666
// default TCP socket type
#define DEFAULT_PROTO SOCK_DGRAM

void Usage(char *progname)
{
    fprintf(stderr,"Usage: %s -p [protocol] -e [port_num] -i [ip_address] -f [log file name]\n", progname);
    fprintf(stderr,"Where:\n\t- protocol is one of TCP or UDP\n");
    fprintf(stderr,"\t- port_num is the port to listen on\n");
    fprintf(stderr,"\t- ip_address is the ip address (in dotted\n");
    fprintf(stderr,"\t  decimal notation) to bind to. But it is not useful here\n");
	fprintf(stderr,"\t- log file name");
    fprintf(stderr,"\t- Hit Ctrl-C to terminate server program...\n");
    fprintf(stderr,"\t- The defaults are UDP, 6666, INADDR_ANY, and NetConsole.log\n");
    WSACleanup();
    exit(1);
}

int main(int argc, char **argv)
{
    char Buffer[1000];
    char *ip_address= NULL;
    unsigned short port=DEFAULT_PORT;
    int retval;
    int fromlen;
    int i;
    int socket_type = DEFAULT_PROTO;
    struct sockaddr_in local, from;
    WSADATA wsaData;
    SOCKET listen_socket, msgsock;
    char file_name_out[500] = "NetConsole.log";
    FILE *output;
	long lastFlushTime=0, currentTime;

    /* Parse arguments, if there are arguments supplied */
    if (argc > 1)
       {
        for(i=1; i<argc; i++)
              {
                     // switches or options...
            if ((argv[i][0] == '-') || (argv[i][0] == '/'))
                     {
                            // Change to lower...if any
                           switch(tolower(argv[i][1]))
                           {
                     // if -p or /p
                    case 'p':
                        if (!stricmp(argv[i+1], "TCP"))
                            socket_type = SOCK_STREAM;
                        else if (!stricmp(argv[i+1], "UDP"))
                            socket_type = SOCK_DGRAM;
                        else
                            Usage(argv[0]);
                        i++;
                        break;
                     // if -i or /i, for server it is not so useful...
                    case 'i':
                        ip_address = argv[++i];
                        break;
                    // if -e or /e
                    case 'e':
                        port = atoi(argv[++i]);
						break;
					case 'f':
						strcpy( file_name_out, argv[++i] );
						break;
                     // No match...
                    default:
                        Usage(argv[0]);
                        break;
                }
            }
            else
                Usage(argv[0]);
        }
    }

	if (( output = fopen(file_name_out , "a+b" )) == NULL )
	{printf( "%s was not opened.\n", file_name_out ); return 1;}

	fflush(NULL); // initialize file for display log program

    // Request Winsock version 2.2
    if ((retval = WSAStartup(0x202, &wsaData)) != 0)
       {
        fprintf(stderr,"Server: WSAStartup() failed with error %d\n", retval);
        WSACleanup();
        return -1;
    }
    else
       printf("Server: WSAStartup() is OK.\n");

    if (port == 0)
       {
        Usage(argv[0]);
    }

    local.sin_family = AF_INET;
    local.sin_addr.s_addr = (!ip_address) ? INADDR_ANY:inet_addr(ip_address);

    /* Port MUST be in Network Byte Order */
    local.sin_port = htons(port);
    // TCP socket
    listen_socket = socket(AF_INET, socket_type,0);

    if (listen_socket == INVALID_SOCKET){
        fprintf(stderr,"Server: socket() failed with error %d\n", WSAGetLastError());
        WSACleanup();
        return -1;
    }
    else
       printf("Server: socket() is OK.\n");

	// set timeout for output buffer flushing during inactivity
	struct timeval tv;
	tv.tv_sec = 3000; // struct says this is seconds but it's milliseconds
	tv.tv_usec = 0;
  if (SOCKET_ERROR == setsockopt(listen_socket, SOL_SOCKET, SO_RCVTIMEO,(const char*)&tv,sizeof(tv)) ) {
      perror("Error setting timeout");
  }

  // set allow reuse address if already in use
  int reuse=1;
  if (SOCKET_ERROR == setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR,(const char*)&reuse,sizeof(reuse)) ) {
        perror("Error setting reuseaddr");
    }

    // bind() associates a local address and port combination with the socket just created.
    // This is most useful when the application is a
    // server that has a well-known port that clients know about in advance.
    if (bind(listen_socket, (struct sockaddr*)&local, sizeof(local)) == SOCKET_ERROR)
       {
    	int socket_error_number = WSAGetLastError();
        fprintf(stderr,"Server: bind() failed with error %d\n", socket_error_number);
        if (socket_error_number == WSAEADDRINUSE)
        	fprintf(stderr,"Server: bind() failed with address in use error.  Close the \"real\" NetConsole.vi\n"); // errors in winsock2.h
        WSACleanup();
        return -1;
    }
    else
        printf("Server: bind() is OK.\n");

     // So far, everything we did was applicable to TCP as well as UDP.
     // However, there are certain steps that do not work when the server is
     // using UDP. We cannot listen() on a UDP socket.
    if (socket_type != SOCK_DGRAM)
       {
        if (listen(listen_socket,5) == SOCKET_ERROR)
              {
            fprintf(stderr,"Server: listen() failed with error %d\n", WSAGetLastError());
            WSACleanup();
            return -1;
        }
       else
              printf("Server: listen() is OK.\n");
    }

    printf("Server: %s: I'm listening and waiting for connection on port %d, protocol %s\n",
		argv[0], port, (socket_type == SOCK_STREAM)?"TCP":"UDP");
	printf("Server: Writing received datagrams to file %s\n", file_name_out);

       while(true)
       {
        fromlen =sizeof(from);
        // accept() doesn't make sense on UDP, since we do not listen()
        if (socket_type != SOCK_DGRAM)
              {
            msgsock = accept(listen_socket, (struct sockaddr*)&from, &fromlen);
            if (msgsock == INVALID_SOCKET)
           {
                fprintf(stderr,"Server: accept() error %d\n", WSAGetLastError());
                WSACleanup();
                return -1;
            }
           else
              printf("Server: accept() is OK.\n");
              printf("Server: accepted connection from %s, port %d\n", inet_ntoa(from.sin_addr), htons(from.sin_port)) ;

        }
        else
            msgsock = listen_socket;

        // In the case of SOCK_STREAM, the server can do recv() and send() on
        // the accepted socket and then close it.
        // However, for SOCK_DGRAM (UDP), the server will do recvfrom() and sendto() in a loop.
        if (socket_type != SOCK_DGRAM)
            retval = recv(msgsock, Buffer, sizeof(Buffer), 0);
        else
        {
            retval = recvfrom(msgsock,Buffer, sizeof(Buffer), 0, (struct sockaddr *)&from, &fromlen);
        }

		if (retval == SOCKET_ERROR)
            {
			if (WSAGetLastError() == WSAETIMEDOUT)
			{
				//fprintf(stderr,"Timed Out\n");
				fflush(NULL);  // no activity for timeout period so flush output buffers
 				continue;}
			else {
				fprintf(stderr,"Server: recv() failed: error %d\n", WSAGetLastError());
				closesocket(msgsock);
				continue;}
			}

        if (retval == 0)
              {
            printf("Server: Client closed connection.\n");
            closesocket(msgsock);
            continue;
        }
		if (retval > 0) fprintf(output,"%*.*s", retval, retval, Buffer);  // send message to log file
		currentTime = GetTickCount();
		if ((currentTime - lastFlushTime) >= 1000) // flush every second
			{
				fflush(NULL);
				lastFlushTime = currentTime;
			}
    }

	   if ( output != NULL && fclose( output ))
		{printf( "\noutput file not closed.\n" ); return 1;}

       return 0;
}




