#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <unistd.h>
#include <SDL2/SDL.h>

//#include "ApiCodec/Naio01Codec.hpp"
#include "Core.hpp"


using namespace std;
using namespace std::chrono;

#define PORT_ROBOT_MOTOR 5555
#define DEFAULT_HOST_ADDRESS "127.0.0.1"

SDL_Window *initSDL(const char* name, int szX, int szY, SDL_Renderer** renderer){
	SDL_Window *screen;

	SDL_Init(SDL_INIT_EVERYTHING);
//	TTF_Init();
	screen = SDL_CreateWindow(name, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, szX, szY, SDL_WINDOW_SHOWN);

	*renderer =  SDL_CreateRenderer( screen, 0, SDL_RENDERER_ACCELERATED);

	// Set render color to black ( background will be rendered in this color )
	SDL_SetRenderDrawColor( *renderer, 0, 0, 0, 255 );

	SDL_RenderClear( *renderer );
	SDL_RenderPresent(*renderer);

//	font = TTF_OpenFont("arial.ttf", 10); //this opens a font style and sets a size


	return screen;
}

void exitSDL()
{
	SDL_Quit();
}

int main( int argc, char* argv[] )
{
	std::string hostAdress = DEFAULT_HOST_ADDRESS;

	int hostPort = PORT_ROBOT_MOTOR;

	// core initialisation
	Core core;

	core.init();

	if( argc > 1 )
	{
		hostAdress = argv[1];
	}

	if( argc > 2 )
	{
		hostPort = atoi( argv[2] );
	}




	SDL_Window* screen;
	SDL_Renderer* renderer;

	screen = initSDL("Visu Robot", 500, 500, &renderer);

	int sdlKey[SDL_NUM_SCANCODES];

	for (int i = 0;i< SDL_NUM_SCANCODES;i++)
	{
		sdlKey[i] = 0;
	}

	if( argc > 1 )
	{
		hostAdress = argv[1];
	}

	if( argc > 2 )
	{
		hostPort = atoi( argv[2] );
	}

	cout << "Connecting to : " << hostAdress << ":" <<  hostPort << endl;

	int socket_desc;
	struct sockaddr_in server;

	//Create socket

	socket_desc = socket( AF_INET, SOCK_STREAM, 0 );
	if (socket_desc == -1)
	{
		cout << "Could not create socket" << endl;
	}

	server.sin_addr.s_addr = inet_addr( hostAdress.c_str() );
	server.sin_family = AF_INET;
	server.sin_port = htons( hostPort );

	//fcntl(sockfd, F_SETFL, O_NONBLOCK);

	//Connect to remote server
	if (connect(socket_desc, (struct sockaddr *) &server, sizeof(server)) < 0)
	{
		puts("connect error");
	}
	else
	{
		puts("Connected\n");
	}

	char *message;

	uint8_t receiveBuffer[4000000];

	ssize_t bytes_recieved = 0;

	milliseconds ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
	int64_t now = static_cast<int64_t>( ms.count() );

	int64_t duration = 50;

	int64_t nextTick = now + duration;

	uint8_t standCmd[17] =         { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t forwardCmd[17] =       { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0x64, 0x64, 0x00, 0x00, 0x00, 0x00 };
	uint8_t backwardCmd[17] =      { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0xAA, 0xAA, 0x00, 0x00, 0x00, 0x00 };
	uint8_t leftCmd[17] =          { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0xAA, 0x64, 0x00, 0x00, 0x00, 0x00 };
	uint8_t rightCmd[17] =         { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0x64, 0xAA, 0x00, 0x00, 0x00, 0x00 };
	uint8_t leftForwardCmd[17] =   { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0x32, 0x64, 0x00, 0x00, 0x00, 0x00 };
	uint8_t rightForwardCmd[17] =  { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0x64, 0x32, 0x00, 0x00, 0x00, 0x00 };
	uint8_t leftBackwardCmd[17] =  { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xAA, 0x00, 0x00, 0x00, 0x00 };
	uint8_t rightBackwardCmd[17] = { 0x4e, 0x41, 0x49, 0x4f, 0x30, 0x31, 0x01, 0x00, 0x00, 0x00, 0x02, 0xAA, 0xCA, 0x00, 0x00, 0x00, 0x00 };

	// Naio01Codec naioCodec;

	while ( true )
	{
		int readSize = (int)read( socket_desc, receiveBuffer, 4000000 );

		if( readSize > 0 )
		{
			bool packetHeaderDetected = false;
//
//			//bool atLeastOnePacketReceived = naioCodec.decode( receiveBuffer, static_cast<uint>( readSize ), packetHeaderDetected );
//
//			if( atLeastOnePacketReceived == true )
//			{
//
//			}
		}



		if( now >= nextTick )
		{
			nextTick = now + duration;

			SDL_Event event;
			while ( SDL_PollEvent( &event ) )
			{
				switch(event.type)
				{
					// Cas d'une touche enfoncée
					case SDL_KEYDOWN:
						sdlKey[event.key.keysym.scancode] = 1;
						break;
						// Cas d'une touche relâchée
					case SDL_KEYUP:
						sdlKey[event.key.keysym.scancode] = 0;
						break;
				}
			}

			if( sdlKey[SDL_SCANCODE_ESCAPE] == 1)
			{
				return 0;
			}
			else if( sdlKey[SDL_SCANCODE_UP] == 1 and sdlKey[SDL_SCANCODE_LEFT] == 1 )
			{
				int sentSize = (int)write( socket_desc, leftForwardCmd, 17 );
			}
			else if( sdlKey[SDL_SCANCODE_UP] == 1 and sdlKey[SDL_SCANCODE_RIGHT] == 1 )
			{
				int sentSize = (int)write( socket_desc, rightForwardCmd, 17 );
			}
			else if( sdlKey[SDL_SCANCODE_DOWN] == 1 and sdlKey[SDL_SCANCODE_LEFT] == 1 )
			{
				int sentSize = (int)write( socket_desc, leftBackwardCmd, 17 );
			}
			else if( sdlKey[SDL_SCANCODE_DOWN] == 1 and sdlKey[SDL_SCANCODE_RIGHT] == 1 )
			{
				int sentSize = (int)write( socket_desc, rightBackwardCmd, 17 );
			}
			else if( sdlKey[SDL_SCANCODE_UP] == 1 )
			{
				int sentSize = (int)write( socket_desc, forwardCmd, 17 );
			}
			else if( sdlKey[SDL_SCANCODE_DOWN] == 1 )
			{
				int sentSize = (int)write( socket_desc, backwardCmd, 17 );
			}
			else if( sdlKey[SDL_SCANCODE_LEFT] == 1 )
			{
				int sentSize = (int)write( socket_desc, leftCmd, 17 );
			}
			else if( sdlKey[SDL_SCANCODE_RIGHT] == 1 )
			{
				int sentSize = (int)write( socket_desc, rightCmd, 17 );
			}
			else
			{
				int sentSize = (int)write( socket_desc, standCmd, 17 );

				std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 250 ) ) );
			}
		}

		ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
		now = static_cast<int64_t>( ms.count() );

		std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( duration / 2) ) );
	}

	return 0;
}
