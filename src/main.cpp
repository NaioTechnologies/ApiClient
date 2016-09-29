#include <iostream>
#include <chrono>
#include "Core.hpp"

using namespace std;
using namespace std::chrono;

#define PORT_ROBOT_MOTOR 5555
#define DEFAULT_HOST_ADDRESS "127.0.0.1"


int main( int argc, char* argv[] )
{
	std::string hostAdress = DEFAULT_HOST_ADDRESS;

	int hostPort = PORT_ROBOT_MOTOR;

	// core initialisation
	Core core;

	if( argc > 1 )
	{
		hostAdress = argv[1];
	}

	if( argc > 2 )
	{
		hostPort = atoi( argv[2] );
	}

	// start main core thread
	core.init( hostAdress, static_cast<uint16_t>( hostPort ) );

	// waits the thread exits
	core.joinMainThread();

	return 0;
}
