#ifndef SOCKET_HPP
#define SOCKET_HPP

#include <string>

class Socket {
public:
	Socket(std::string host,  uint16_t port);
	virtual ~Socket();

	int get_fd() const { return fd_; }

private:
	int fd_;
};

#endif
