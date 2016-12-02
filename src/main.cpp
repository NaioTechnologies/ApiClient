#include "Core.hpp"
#include <sys/resource.h>

#define PORT_ROBOT_MOTOR 5559
#define DEFAULT_HOST_ADDRESS "127.0.0.1"


int main( int argc, char** argv )
{
//	const rlim_t kStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
//	struct rlimit rl;
//	int result;
//
//	result = getrlimit(RLIMIT_STACK, &rl);
//	if (result == 0)
//	{
//		if (rl.rlim_cur < kStackSize)
//		{
//			rl.rlim_cur = kStackSize;
//			result = setrlimit(RLIMIT_STACK, &rl);
//			if (result != 0)
//			{
//				fprintf(stderr, "setrlimit returned result = %d\n", result);
//			}
//		}
//	}

	std::string hostAdress = DEFAULT_HOST_ADDRESS;

	int hostPort = PORT_ROBOT_MOTOR;

	// core initialisation
	Core* core = new Core();

	if( argc > 1 )
	{
		hostAdress = argv[1];
	}

	if( argc > 2 )
	{
		hostPort = atoi( argv[2] );
	}

	// start main core thread
	core->init( hostAdress, static_cast<uint16_t>( hostPort ) );

	// waits the thread exits
	core->joinMainThread();

	delete core;

	return 0;
}
