#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <unistd.h>
#include <SDL2/SDL.h>
#include <ApiCodec/ApiMotorsPacket.hpp>
#include <ApiCodec/ApiStatusPacket.hpp>

#include "Core.hpp"

using namespace std;
using namespace std::chrono;

// #################################################

Core::Core( ) :
		stopThreadAsked_{ false },
		threadStarted_{ false },
		mainThread_{ },
		hostAdress_{ "127.0.0.1" },
		hostPort_{ 5555 },
		socketConnected_{false},
		naioCodec_{ },
		sendPacketList_{ },
		askedApiMotorsPacketPtr_{ nullptr },
		lastReceivedStatusPacketPtr_{ nullptr },
		controlType_{ ControlType::CONTROL_TYPE_MANUAL }
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

	screen = initSDL("Api Client", 800, 600, &renderer);

	// ignore unused screen
	(void)screen;

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

	// creates main thread
	mainThread_ = std::thread( &Core::call_from_thread, this );
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
	uint8_t receiveBuffer[4000000];

	std::cout << "Starting main thread." << std::endl;

	// prepare timers for real time operations
	milliseconds ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );

	int64_t now = static_cast<int64_t>( ms.count() );
	int64_t duration = 50;
	int64_t nextTick = now + duration;

	threadStarted_ = true;

	while( !stopThreadAsked_ )
	{
		// any time : read incoming messages.
		int readSize = (int)read( socket_desc_, receiveBuffer, 4000000 );

		if( readSize > 0 )
		{
			bool packetHeaderDetected = false;

			bool atLeastOnePacketReceived = naioCodec_.decode( receiveBuffer, static_cast<uint>( readSize ), packetHeaderDetected );

			// manage received messages
			if( atLeastOnePacketReceived == true )
			{
				for( auto&& packetPtr : naioCodec_.currentBasePacketList )
				{
					manageReceivedPacket( packetPtr );
				}

				naioCodec_.currentBasePacketList.clear();
			}
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
					// if no input given, waits a bit more.
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

		// sleep half a duration
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
Core::initSDL( const char* name, int szX, int szY, SDL_Renderer** renderer )
{
	SDL_Window *screen;

	SDL_Init( SDL_INIT_EVERYTHING );
	screen = SDL_CreateWindow( name, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, szX, szY, SDL_WINDOW_SHOWN );

	*renderer =  SDL_CreateRenderer( screen, 0, SDL_RENDERER_ACCELERATED );

	// Set render color to black ( background will be rendered in this color )
	SDL_SetRenderDrawColor( *renderer, 0, 0, 0, 255 );

	SDL_RenderClear( *renderer );
	SDL_RenderPresent( *renderer );

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
				sdlKey_[ event.key.keysym.scancode ] = 1;
				break;
				// Cas d'une touche relâchée
			case SDL_KEYUP:
				sdlKey_[ event.key.keysym.scancode ] = 0;
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

	if( sdlKey_[ SDL_SCANCODE_ESCAPE ] == 1)
	{
		stopThreadAsked_ = true;

		return true;
	}

	if( sdlKey_[ SDL_SCANCODE_UP ] == 1 and sdlKey_[ SDL_SCANCODE_LEFT ] == 1 )
	{
		left = 32;
		right = 63;
		keyPressed = true;
	}
	else if( sdlKey_[ SDL_SCANCODE_UP ] == 1 and sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 )
	{
		left = 63;
		right = 32;
		keyPressed = true;
	}
	else if( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 and sdlKey_[ SDL_SCANCODE_LEFT ] == 1 )
	{
		left = -32;
		right = -63;
		keyPressed = true;
	}
	else if( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 and sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 )
	{
		left = -63;
		right = -32;
		keyPressed = true;
	}
	else if( sdlKey_[ SDL_SCANCODE_UP ] == 1 )
	{
		left = 63;
		right = 63;
		keyPressed = true;
	}
	else if( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 )
	{
		left = -63;
		right = -63;
		keyPressed = true;

	}
	else if( sdlKey_[ SDL_SCANCODE_LEFT ] == 1 )
	{
		left = -63;
		right = 63;
		keyPressed = true;
	}
	else if( sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 )
	{
		left = 63;
		right = -63;
		keyPressed = true;
	}

	askedApiMotorsPacketPtr_ = std::make_shared<ApiMotorsPacket>( left, right );

	return keyPressed;
}

// #################################################
void
Core::manageReceivedPacket( BaseNaio01PacketPtr packetPtr )
{
	if( std::dynamic_pointer_cast<ApiStatusPacket>( packetPtr ) )
	{
		ApiStatusPacketPtr statusPacketPtr = std::dynamic_pointer_cast<ApiStatusPacket>( packetPtr );

		lastReceivedStatusPacketPtr_ = statusPacketPtr;

		std::cout << "theta : " << statusPacketPtr->theta << std::endl;
	}
}


// #################################################
void
Core::joinMainThread()
{
	mainThread_.join();
}