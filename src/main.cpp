#include "Core.hpp"
#include <sys/resource.h>

#define PORT_ROBOT_MOTOR 5555
#define DEFAULT_HOST_ADDRESS "127.0.0.1"

int main( int argc, char** argv )
{
//	struct sockaddr_in imageServer;
//
//	//Create socket
//	int image_socket_desc_ = socket( AF_INET, SOCK_STREAM, 0 );
//
//	if ( image_socket_desc_ == -1 )
//	{
//		std::cout << "NetStereoImporter Could not create socket" << std::endl;
//	}
//
//	imageServer.sin_addr.s_addr = inet_addr( "192.168.1.106" );
//	imageServer.sin_family = AF_INET;
//	imageServer.sin_port = htons( static_cast<uint16_t>( 5558 ) );
//
//	//Connect to remote server
//	if ( connect( image_socket_desc_, ( struct sockaddr * ) &imageServer, sizeof( imageServer ) ) < 0 )
//	{
//		puts( "NetStereoImporter image connect error" );
//	}
//	else
//	{
//		puts( "NetStereoImporter Connected image\n" );
//	}
//
//	std::cout << "NetStereoImporter socket created" << std::endl;
//
//	char receive_buffer[ 4000000 ];
//
//	char protocol_start[ 6 ] = { 'N','A','I','O','0','0' };
//	uint protocol_state = 0;
//
//	int nb_bytes = 0;
//
//	ssize_t send_size = send( image_socket_desc_, protocol_start, 6, 0 );
//	(void)send_size;
//
//	int last_read_size = 4096;
//
//	while( true )
//	{
//		//ssize_t readSize = recv( image_socket_desc_, receive_buffer, last_read_size, MSG_WAITALL );
//		ssize_t readSize = recv( image_socket_desc_, receive_buffer, 512, MSG_DONTWAIT );
//
//		last_read_size = ( last_read_size + last_read_size + readSize ) / 3;
//
//		if( last_read_size <= 0 )
//		{
//			last_read_size = 1;
//		}
//
//		if ( readSize == 0 )
//		{
//		//	std::cout << "socket error readSize : " << static_cast<int>( readSize ) << std::endl;
//			//std::this_thread::sleep_for( std::chrono::microseconds( 10 ) );
//		//	exit(0);
//		}
//		else if ( readSize < 0 )
//		{
//			//std::cout << "socket error readSize : " << static_cast<int>( readSize ) << std::endl;
//			//std::this_thread::sleep_for( std::chrono::microseconds( 50 ) );
//		}
//		else if ( readSize >= 0 )
//		{
//			//std::cout << "readSize : " << static_cast<int>( readSize ) << std::endl;
//
//			for ( uint received_buffer_idx = 0 ; received_buffer_idx < readSize ; received_buffer_idx++ )
//			{
//				if( receive_buffer[ received_buffer_idx ] != 0 )
//				{
//					std::cout << "nb_bytes : " << nb_bytes << " : " << static_cast<int>( receive_buffer[ received_buffer_idx ] ) << std::endl;
//				}
//
//				nb_bytes++;
//			}
//
////			ssize_t send_size = send( image_socket_desc_, protocol_start, 1, 0 );
////			(void)send_size;
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

	bool graphical_display_on = true;

	for( int i = 0 ; i < argc ; i++ )
	{
		std::string nogui = argv[ i ];

		if( nogui == "nogui" )
		{
			std::cout << "Starting Simulatoz Bridge in no gui mode." << std::endl;

			graphical_display_on = false;
		}
	}

	// start main core thread
	core->init( graphical_display_on, hostAdress, static_cast<uint16_t>( hostPort ) );

	std::cout << "Simulatoz Bridge Started" << std::endl;

	while ( not core->stop_main_thread_asked_ )
	{
		std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
	}

//	// waits the thread exits
//	core->join_main_thread( );

	delete core;

	return 0;
}
