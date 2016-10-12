#ifndef MY_SOCKET_H
#define MY_SOCKET_H

#include <sys/ioctl.h>

#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;


void error(const char *msg);
//Initialisation du socket
SOCKET openSocketClient(const char* address, int port);


//Ouvre le port donné
SOCKET openSocketServer(int portNum);

//Attend qu'un client se connecte
SOCKET waitConnect(SOCKET sockfd);


//Lit une entrée client. Retourne 1 si lecture OK, 0 si rien
int readNonBlockSocket(SOCKET fd, char buff[]);

void sendToSocket(SOCKET socket, char message[]);

#endif
