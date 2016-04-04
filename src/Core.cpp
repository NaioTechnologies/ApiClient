#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <unistd.h>
#include <SDL2/SDL.h>
#include <ApiCodec/ApiMotorsPacket.hpp>

#include "Core.hpp"

using namespace std;
using namespace std::chrono;

// #################################################

Core::Core( ) :
		stopThreadAsked_{ false },
		threadStarted_{ false },
		hostAdress_{ "127.0.0.1" },
		hostPort_{ 5555 },
		socketConnected_{false},
		naioCodec_{ },
		sendPacketList_{ },
		controlType_{ ControlType::CONTROL_TYPE_MANUAL },
		askedApiMotorsPacketPtr_{ nullptr }
{

}

// #################################################

Core::~Core( )
{

}

// #################################################

void
Core::init( std::string hostAdress, uint16_t hostPort )
{
	hostAdress_ = hostAdress;
	hostPort_ = hostPort;

	stopThreadAsked_ = false;
	threadStarted_ = false;
	socketConnected_ = false;

	// create graphics
	SDL_Window* screen;
	SDL_Renderer* renderer;

	screen = initSDL("Api Client", 500, 500, &renderer);

	for ( int i = 0 ; i < SDL_NUM_SCANCODES ; i++ )
	{
		sdlKey_[i] = 0;
	}

	std::cout << "Connecting to : " << hostAdress << ":" <<  hostPort << std::endl;

	struct sockaddr_in server;

	//Create socket
	socket_desc_ = socket( AF_INET, SOCK_STREAM, 0 );

	if (socket_desc_ == -1)
	{
		std::cout << "Could not create socket" << std::endl;
	}

	server.sin_addr.s_addr = inet_addr( hostAdress.c_str() );
	server.sin_family = AF_INET;
	server.sin_port = htons( hostPort );

	//Connect to remote server
	if ( connect( socket_desc_, ( struct sockaddr * ) &server, sizeof( server ) ) < 0 )
	{
		puts( "connect error" );
	}
	else
	{
		puts( "Connected\n" );
		socketConnected_ = true;
	}

	mainThread_ = std::thread( &Core::call_from_thread, this );
	//mainThread_ = std::thread( [=] { call_from_thread(); } );

	//return mainThread_;
}

// #################################################

void
Core::stop( )
{
	if( threadStarted_ )
	{
		stopThreadAsked_ = true;

		mainThread_.join();

		threadStarted_ = false;
	}
}

// #################################################

void
Core::call_from_thread( )
{
	std::cout << "Starting main thread." << std::endl;

	uint8_t receiveBuffer[4000000];

	milliseconds ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );

	int64_t now = static_cast<int64_t>( ms.count() );
	int64_t duration = 50;
	int64_t nextTick = now + duration;

	threadStarted_ = true;

	while( !stopThreadAsked_ )
	{
		int readSize = (int)read( socket_desc_, receiveBuffer, 4000000 );

		if( readSize > 0 )
		{
//			bool packetHeaderDetected = false;
//
//			//bool atLeastOnePacketReceived = naioCodec.decode( receiveBuffer, static_cast<uint>( readSize ), packetHeaderDetected );
//
//			if( atLeastOnePacketReceived == true )
//			{
//
//			}
		}

		// Test keyboard input.
		// send commands related to keyboard.
		if( now >= nextTick )
		{
			nextTick = now + duration;

			readSDLKeyboard();

			manageSDLKeyboard();

			if( controlType_ == ControlType::CONTROL_TYPE_MANUAL )
			{
				if( askedApiMotorsPacketPtr_ != nullptr )
				{
					sendPacketList_.emplace_back( askedApiMotorsPacketPtr_ );
				}
				else
				{
					std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 250 ) ) );
				}
			}
		}

		// send and empty the waiting packet queue
		bool allWasOk = sendWaitingPackets();

		if( !allWasOk )
		{
			// what to do ?
		}

		// compute next tick
		ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );

		now = static_cast<int64_t>( ms.count() );

		std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( duration / 2) ) );
	}

	threadStarted_ = false;
	stopThreadAsked_ = false;

	std::cout << "Stopping main thread." << std::endl;
}

// #################################################
bool
Core::sendWaitingPackets()
{
	bool allWasOk = true;

	for( auto&& packet : sendPacketList_ )
	{
		cl::BufferUPtr buffer = packet->encode();

		int sentSize = (int)write( socket_desc_, buffer->data(), buffer->size() );

		if( sentSize <= 0 )
		{
			allWasOk = false;
		}
	}

	sendPacketList_.clear();

	return allWasOk;
}

// #################################################

SDL_Window*
Core::initSDL(const char* name, int szX, int szY, SDL_Renderer** renderer)
{
	SDL_Window *screen;

	SDL_Init(SDL_INIT_EVERYTHING);
	screen = SDL_CreateWindow(name, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, szX, szY, SDL_WINDOW_SHOWN);

	*renderer =  SDL_CreateRenderer( screen, 0, SDL_RENDERER_ACCELERATED);

	// Set render color to black ( background will be rendered in this color )
	SDL_SetRenderDrawColor( *renderer, 0, 0, 0, 255 );

	SDL_RenderClear( *renderer );
	SDL_RenderPresent(*renderer);

	return screen;
}

// #################################################
void
Core::exitSDL()
{
	SDL_Quit();
}

// #################################################
void
Core::readSDLKeyboard()
{
	SDL_Event event;

	while ( SDL_PollEvent( &event ) )
	{
		switch(event.type)
		{
			// Cas d'une touche enfoncée
			case SDL_KEYDOWN:
				sdlKey_[event.key.keysym.scancode] = 1;
				break;
				// Cas d'une touche relâchée
			case SDL_KEYUP:
				sdlKey_[event.key.keysym.scancode] = 0;
				break;
		}
	}
}

// #################################################

bool
Core::manageSDLKeyboard()
{
	bool keyPressed = false;

	int8_t left = 0;
	int8_t right = 0;

	if( sdlKey_[SDL_SCANCODE_ESCAPE] == 1)
	{
		stopThreadAsked_ = true;

		return true;
	}

	if( sdlKey_[SDL_SCANCODE_UP] == 1 and sdlKey_[SDL_SCANCODE_LEFT] == 1 )
	{
		left = 32;
		right = 63;
		keyPressed = true;
	}
	else if( sdlKey_[SDL_SCANCODE_UP] == 1 and sdlKey_[SDL_SCANCODE_RIGHT] == 1 )
	{
		left = 63;
		right = 32;
		keyPressed = true;
	}
	else if( sdlKey_[SDL_SCANCODE_DOWN] == 1 and sdlKey_[SDL_SCANCODE_LEFT] == 1 )
	{
		left = -32;
		right = -63;
		keyPressed = true;
	}
	else if( sdlKey_[SDL_SCANCODE_DOWN] == 1 and sdlKey_[SDL_SCANCODE_RIGHT] == 1 )
	{
		left = -63;
		right = -32;
		keyPressed = true;
	}
	else if( sdlKey_[SDL_SCANCODE_UP] == 1 )
	{
		left = 63;
		right = 63;
		keyPressed = true;
	}
	else if( sdlKey_[SDL_SCANCODE_DOWN] == 1 )
	{
		left = -63;
		right = -63;
		keyPressed = true;

	}
	else if( sdlKey_[SDL_SCANCODE_LEFT] == 1 )
	{
		left = -63;
		right = 63;
		keyPressed = true;
	}
	else if( sdlKey_[SDL_SCANCODE_RIGHT] == 1 )
	{
		left = 63;
		right = -63;
		keyPressed = true;
	}

	askedApiMotorsPacketPtr_ = std::make_shared<ApiMotorsPacket>( left, right );

	return keyPressed;
}