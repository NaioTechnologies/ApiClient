#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "Socket.hpp"

Socket::Socket(std::string host, uint16_t port) {
	struct sockaddr_in server ;

	fd_ = socket( AF_INET, SOCK_STREAM, 0 );
	if (fd_ == -1)
	{
		throw std::string("Could not create socket: ") + strerror(errno);
	}

	server.sin_addr.s_addr = inet_addr( host.c_str() );
	server.sin_family = AF_INET;
	server.sin_port = htons( port );

	//Connect to remote server
	if (!connect(fd_, (struct sockaddr *) &server, sizeof(server)))
	{
		throw std::string("connect error") + strerror(errno);
	}
}

Socket::~Socket() {
	close(fd_);
}

