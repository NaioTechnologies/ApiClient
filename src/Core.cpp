#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <unistd.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <ApiCodec/ApiMotorsPacket.hpp>
#include <ApiCodec/ApiStatusPacket.hpp>
#include <ApiIhmDisplayPacket.hpp>
#include <HaGyroPacket.hpp>
#include <HaAcceleroPacket.hpp>

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
		askedHaMotorsPacketPtr_{ nullptr },
		api_lidar_packet_ptr_{ nullptr },
		ha_lidar_packet_ptr_{ nullptr },
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
	screen_ = initSDL("Api Client", 800, 480, &renderer_);

	// ignore unused screen
	(void)screen_;

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
	int64_t duration = 15;
	int64_t nextTick = now + duration;

	threadStarted_ = true;

	ApiMotorsPacketPtr first_packet = std::make_shared<ApiMotorsPacket>( 0 , 0 );
	cl_copy::BufferUPtr first_buffer = first_packet->encode();
	write( socket_desc_, first_buffer->data(), first_buffer->size() );

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
				if( askedApiMotorsPacketPtr_ == nullptr && askedApiMotorsPacketPtr_ )
				{
					// if no input given, waits a bit more.
					std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 250 ) ) );
				}
				else
				{
					if( askedApiMotorsPacketPtr_ != nullptr )
					{
						sendPacketList_.emplace_back( askedApiMotorsPacketPtr_ );
					}

					if( askedHaMotorsPacketPtr_ != nullptr )
					{
						sendPacketList_.emplace_back( askedHaMotorsPacketPtr_ );
					}
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

		// drawing part.
		SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255); // the rect color (solid red)
		SDL_Rect background;
		background.w = 800;
		background.h = 480;
		background.y = 0;
		background.x = 0;

		SDL_RenderFillRect(renderer_, &background);

		draw_robot();

		uint16_t lidar_distance_[271];

		ha_lidar_packet_ptr_access.lock();
		api_lidar_packet_ptr_access.lock();

		if( ha_lidar_packet_ptr_ != nullptr )
		{
			for( int i = 0; i < 271 ; i++ )
			{
				lidar_distance_[i] = ha_lidar_packet_ptr_->distance[ i ];
			}
		}
		else if( api_lidar_packet_ptr_ != nullptr )
		{
			for( int i = 0; i < 271 ; i++ )
			{
				lidar_distance_[i] = 5000;
			}
		}

		api_lidar_packet_ptr_access.unlock();
		ha_lidar_packet_ptr_access.unlock();

		draw_lidar( lidar_distance_ );

		SDL_Color white = {255, 255, 255};
		TTF_Font* font = TTF_OpenFont("mono.ttf", 12);
		SDL_Surface* surfaceMessage = TTF_RenderText_Blended( font, "TEST", white );
		SDL_Texture* message = SDL_CreateTextureFromSurface( renderer_, surfaceMessage );
		SDL_Rect message_rect;
		message_rect.x = 200;
		message_rect.y = 200;
		message_rect.w = 100;
		message_rect.h = 20;
		SDL_QueryTexture( message, NULL, NULL, &message_rect.w, &message_rect.h );
		SDL_RenderCopy( renderer_, message, NULL, &message_rect );
		SDL_DestroyTexture(message);
		SDL_FreeSurface(surfaceMessage);

		static int flying_pixel_x = 0;

		if( flying_pixel_x > 800 )
		{
			flying_pixel_x = 0;
		}

		SDL_SetRenderDrawColor(renderer_, 200, 150, 125, 255); // the rect color (solid red)
		SDL_Rect flying_pixel;
		flying_pixel.w = 1;
		flying_pixel.h = 1;
		flying_pixel.y = 480 - 40;
		flying_pixel.x = flying_pixel_x;

		flying_pixel_x++;

		SDL_RenderFillRect(renderer_, &flying_pixel);

		SDL_RenderPresent( renderer_ );
	}

	threadStarted_ = false;
	stopThreadAsked_ = false;

	std::cout << "Stopping main thread." << std::endl;
}

// #################################################
void Core::draw_lidar( uint16_t lidar_distance_[271] )
{
	for( int i = 0; i < 271 ; i++ )
	{
		double dist = static_cast<double>( lidar_distance_[ i ] ) / 10.0f;

		if( dist < 3.0f )
		{
			dist = 5000.0f;
		}

		if( i > 45 )
		{
			double x_cos = dist * cos(  static_cast<double>( ( i - 45 ) * M_PI / 180. ) );
			double y_sin = dist * sin(  static_cast<double>( ( i - 45 ) * M_PI / 180. ) );

			double x = 400.0 + x_cos;
			double y = 400.0 - y_sin;

			SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
			SDL_Rect lidar_pixel;

			lidar_pixel.w = 1;
			lidar_pixel.h = 1;
			lidar_pixel.x = static_cast<int>( x );
			lidar_pixel.y = static_cast<int>( y );

			SDL_RenderFillRect(renderer_, &lidar_pixel);
		}
	}
}

// #################################################
void Core::draw_robot()
{
	SDL_SetRenderDrawColor(renderer_, 200, 200, 200, 255);
	SDL_Rect main;
	main.w = 42;
	main.h = 80;
	main.y = 480 - main.h;
	main.x = 400 - ( main.w / 2);

	SDL_RenderFillRect(renderer_, &main);

	SDL_SetRenderDrawColor(renderer_, 100, 100, 100, 255);
	SDL_Rect flw;
	flw.w = 8;
	flw.h = 20;
	flw.y = 480 - 75;
	flw.x = 400 - 21;

	SDL_RenderFillRect(renderer_, &flw);

	SDL_SetRenderDrawColor(renderer_, 100, 100, 100, 255);
	SDL_Rect frw;
	frw.w = 8;
	frw.h = 20;
	frw.y = 480 - 75;
	frw.x = 400 + 21 - 8;

	SDL_RenderFillRect(renderer_, &frw);

	SDL_SetRenderDrawColor(renderer_, 100, 100, 100, 255);
	SDL_Rect rlw;
	rlw.w = 8;
	rlw.h = 20;
	rlw.y = 480 - 5 - 20;
	rlw.x = 400 - 21;

	SDL_RenderFillRect(renderer_, &rlw);

	SDL_SetRenderDrawColor(renderer_, 100, 100, 100, 255);
	SDL_Rect rrw;
	rrw.w = 8;
	rrw.h = 20;
	rrw.y = 480 - 5 -20;
	rrw.x = 400 + 21 - 8;

	SDL_RenderFillRect(renderer_, &rrw);

	SDL_SetRenderDrawColor(renderer_, 120, 120, 120, 255);
	SDL_Rect lidar;
	lidar.w = 8;
	lidar.h = 8;
	lidar.y = 480 - 80 - 8;
	lidar.x = 400 - 4;

	SDL_RenderFillRect(renderer_, &lidar);
}

// #################################################
bool
Core::sendWaitingPackets()
{
	bool allWasOk = true;

	for( auto&& packet : sendPacketList_ )
	{
		cl_copy::BufferUPtr buffer = packet->encode();

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

	motor_packet_access_.lock();
	askedApiMotorsPacketPtr_ = std::make_shared<ApiMotorsPacket>( left, right );
	askedHaMotorsPacketPtr_ = std::make_shared<HaMotorsPacket>( left * 2, right * 2 );
	motor_packet_access_.unlock();

	return keyPressed;
}

// #################################################
void
Core::manageReceivedPacket( BaseNaio01PacketPtr packetPtr )
{
	std::cout << "Packet received id : " << static_cast<int>( packetPtr->getPacketId() ) << std::endl;

	if( std::dynamic_pointer_cast<ApiStatusPacket>( packetPtr ) )
	{
		ApiStatusPacketPtr statusPacketPtr = std::dynamic_pointer_cast<ApiStatusPacket>( packetPtr );

		//lastReceivedStatusPacketPtr_ = statusPacketPtr;

		std::cout << "theta : " << statusPacketPtr->theta << std::endl;
	}
	else if( std::dynamic_pointer_cast<HaLidarPacket>( packetPtr )  )
	{
		HaLidarPacketPtr haLidarPacketPtr = std::dynamic_pointer_cast<HaLidarPacket>( packetPtr );

		ha_lidar_packet_ptr_access.lock();
		ha_lidar_packet_ptr_ = haLidarPacketPtr;
		ha_lidar_packet_ptr_access.unlock();
	}
	else if( std::dynamic_pointer_cast<ApiLidarPacket>( packetPtr )  )
	{
		std::cout << "api lidar received." << std::endl;

		ApiLidarPacketPtr apiLidarPacketPtr = std::dynamic_pointer_cast<ApiLidarPacket>( packetPtr );

		api_lidar_packet_ptr_access.lock();
		api_lidar_packet_ptr_ = apiLidarPacketPtr;
		api_lidar_packet_ptr_access.unlock();
	}
	else if( std::dynamic_pointer_cast<HaGyroPacket>( packetPtr )  )
	{
		HaGyroPacketPtr haGyroPacketPtr = std::dynamic_pointer_cast<HaGyroPacket>( packetPtr );

		std::cout << "gyro : " << haGyroPacketPtr->x << " ; " << haGyroPacketPtr->y << " ; " << haGyroPacketPtr->z << std::endl;
	}
	else if( std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr )  )
	{
		HaAcceleroPacketPtr haAcceleroPacketPtr = std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr );

		std::cout << "accel : " << haAcceleroPacketPtr->x << " ; " << haAcceleroPacketPtr->y << " ; " << haAcceleroPacketPtr->z << std::endl;
	}
}

// #################################################
void
Core::joinMainThread()
{
	mainThread_.join();
}