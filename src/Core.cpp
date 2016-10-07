#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <unistd.h>
#include <ApiCodec/ApiMotorsPacket.hpp>
#include <HaGyroPacket.hpp>
#include <HaAcceleroPacket.hpp>
#include <ApiCommandPacket.hpp>
#include <zlib.h>
#include <ApiWatchdogPacket.hpp>
#include "Core.hpp"

using namespace std;
using namespace std::chrono;

// #################################################
//
Core::Core( ) :
		stopThreadAsked_{ false },
		threadStarted_{ false },
		graphicThread_{ },
		hostAdress_{ "10.0.1.1" },
		hostPort_{ 5555 },
		socketConnected_{false},
		naioCodec_{ },
		sendPacketList_{ },
		ha_lidar_packet_ptr_{ nullptr },
		ha_odo_packet_ptr_{ nullptr },
		api_post_packet_ptr_{nullptr },
		ha_gps_packet_ptr_{ nullptr },
		controlType_{ ControlType::CONTROL_TYPE_MANUAL },
		last_motor_time_{ 0L },
		imageNaioCodec_{ },
		last_left_motor_{ 0 },
		last_right_motor_{ 0 },
		last_image_received_time_{ 0 }
{
	uint8_t fake = 0;

	for ( int i = 0 ; i < 1000000 ; i++ )
	{
		if( fake >= 255 )
		{
			fake = 0;
		}

		last_images_buffer_[ i ] = fake;

		fake++;
	}
}

// #################################################
//
Core::~Core( )
{

}

// #################################################
//
void
Core::init( std::string hostAdress, uint16_t hostPort )
{
	hostAdress_ = hostAdress;
	hostPort_ = hostPort;

	stopThreadAsked_ = false;
	threadStarted_ = false;
	socketConnected_ = false;

	imageServerThreadStarted_ = false;
	stopImageServerThreadAsked_ = false;

	serverReadthreadStarted_ = false;
	stopServerWriteThreadAsked_ = false;

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
	graphicThread_ = std::thread( &Core::graphic_thread, this );

	serverReadThread_ = std::thread( &Core::server_read_thread, this );

	serverWriteThread_ = std::thread( &Core::server_write_thread, this );

	imageServerThread_ = std::thread( &Core::image_server_thread, this );
}

// #################################################
//
void
Core::stop( )
{
	if( threadStarted_ )
	{
		stopThreadAsked_ = true;

		graphicThread_.join();

		threadStarted_ = false;
	}
}

// #################################################
//
void Core::stopServerReadThread( )
{
	if( serverReadthreadStarted_)
	{
		stopServerReadThreadAsked_ = true;

		serverReadThread_.join();

		serverReadthreadStarted_ = false;
	}
}

// #################################################
// thread function
void Core::server_read_thread( )
{
	std::cout << "Starting server read thread !" << std::endl;

	uint8_t receiveBuffer[ 4000000 ];

	while( !stopServerReadThreadAsked_ )
	{
		// any time : read incoming messages.
		int readSize = (int) read( socket_desc_, receiveBuffer, 4000000 );

		if (readSize > 0)
		{
			bool packetHeaderDetected = false;

			bool atLeastOnePacketReceived = naioCodec_.decode( receiveBuffer, static_cast<uint>( readSize ), packetHeaderDetected );

			// manage received messages
			if ( atLeastOnePacketReceived == true )
			{
				for ( auto &&packetPtr : naioCodec_.currentBasePacketList )
				{
					manageReceivedPacket( packetPtr );
				}

				naioCodec_.currentBasePacketList.clear();
			}
		}
	}

	serverReadthreadStarted_ = false;
	stopServerReadThreadAsked_= false;
}

// #################################################
//
void
Core::graphic_thread( )
{
	std::cout << "Starting main thread." << std::endl;

    // create graphics
    screen_ = initSDL( "Api Client", 800, 730 );

	// prepare timers for real time operations
	milliseconds ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );

	int64_t now = static_cast<int64_t>( ms.count() );
	int64_t duration = MAIN_GRAPHIC_DISPLAY_RATE_MS;
	int64_t nextTick = now + duration;

	threadStarted_ = true;

	while( !stopThreadAsked_ )
	{
		ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
		now = static_cast<int64_t>( ms.count() );

		// Test keyboard input.
		// send commands related to keyboard.
		if( now >= nextTick )
		{
			nextTick = now + duration;

			if( asked_start_video_ == true )
			{
				ApiCommandPacketPtr api_command_packet_zlib_off = std::make_shared<ApiCommandPacket>( ApiCommandPacket::CommandType::TURN_OFF_IMAGE_ZLIB_COMPRESSION );
				ApiCommandPacketPtr api_command_packet_stereo_on = std::make_shared<ApiCommandPacket>( ApiCommandPacket::CommandType::TURN_ON_API_RAW_STEREO_CAMERA_PACKET );

				sendPacketListAccess_.lock();
				sendPacketList_.emplace_back( api_command_packet_zlib_off );
				sendPacketList_.emplace_back( api_command_packet_stereo_on );
				sendPacketListAccess_.unlock();

				asked_start_video_ = false;
			}

			if( asked_stop_video_ == true )
			{
				ApiCommandPacketPtr api_command_packet_stereo_off = std::make_shared<ApiCommandPacket>( ApiCommandPacket::CommandType::TURN_OFF_API_RAW_STEREO_CAMERA_PACKET );

				sendPacketListAccess_.lock();
				sendPacketList_.emplace_back( api_command_packet_stereo_off );
				sendPacketListAccess_.unlock();

				asked_stop_video_ = false;
			}
		}

		readSDLKeyboard();
		manageSDLKeyboard();

		// drawing part.
		SDL_SetRenderDrawColor( renderer_, 0, 0, 0, 255 ); // the rect color (solid red)
		SDL_Rect background;
		background.w = 800;
		background.h = 483;
		background.y = 0;
		background.x = 0;

		SDL_RenderFillRect( renderer_, &background );

		draw_robot();

		uint16_t lidar_distance_[ 271 ];

		ha_lidar_packet_ptr_access_.lock();

		if( ha_lidar_packet_ptr_ != nullptr )
		{
			for( int i = 0; i < 271 ; i++ )
			{
				lidar_distance_[ i ] = ha_lidar_packet_ptr_->distance[ i ];
			}
		}
		else
		{
			for( int i = 0; i < 271 ; i++ )
			{
				lidar_distance_[ i ] = 5000;
			}
		}

		ha_lidar_packet_ptr_access_.unlock();

		draw_lidar( lidar_distance_ );

		draw_images( );

		// ##############################################
		char gyro_buff[ 100 ];

		ha_gyro_packet_ptr_access_.lock();
		HaGyroPacketPtr ha_gyro_packet_ptr = ha_gyro_packet_ptr_;
		ha_gyro_packet_ptr_access_.unlock();

		if( ha_gyro_packet_ptr != nullptr )
		{
			snprintf( gyro_buff, sizeof( gyro_buff ), "Gyro  : %d ; %d, %d", ha_gyro_packet_ptr->x, ha_gyro_packet_ptr->y, ha_gyro_packet_ptr->z );

			//std::cout << gyro_buff << std::endl;
		}
		else
		{
			snprintf( gyro_buff, sizeof( gyro_buff ), "Gyro  : N/A ; N/A, N/A" );
		}

		ha_accel_packet_ptr_access_.lock();
		HaAcceleroPacketPtr ha_accel_packet_ptr = ha_accel_packet_ptr_;
		ha_accel_packet_ptr_access_.unlock();

		char accel_buff[100];
		if( ha_accel_packet_ptr != nullptr )
		{
			snprintf( accel_buff, sizeof( accel_buff ), "Accel : %d ; %d, %d", ha_accel_packet_ptr->x, ha_accel_packet_ptr->y, ha_accel_packet_ptr->z );

			//std::cout << accel_buff << std::endl;
		}
		else
		{
			snprintf(accel_buff, sizeof(accel_buff), "Accel : N/A ; N/A, N/A" );
		}

		ha_odo_packet_ptr_access.lock();
		HaOdoPacketPtr ha_odo_packet_ptr = ha_odo_packet_ptr_;
		ha_odo_packet_ptr_access.unlock();

		char odo_buff[100];
		if( ha_odo_packet_ptr != nullptr )
		{
			snprintf( odo_buff, sizeof( odo_buff ), "ODO -> RF : %d ; RR : %d ; RL : %d, FL : %d", ha_odo_packet_ptr->fr, ha_odo_packet_ptr->rr, ha_odo_packet_ptr->rl, ha_odo_packet_ptr->fl );

			//std::cout << odo_buff << std::endl;

		}
		else
		{
			snprintf( odo_buff, sizeof( odo_buff ), "ODO -> RF : N/A ; RR : N/A ; RL : N/A, FL : N/A" );
		}

		ha_gps_packet_ptr_access_.lock();
		HaGpsPacketPtr ha_gps_packet_ptr = ha_gps_packet_ptr_;
		ha_gps_packet_ptr_access_.unlock();

		char gps1_buff[ 100 ];
		char gps2_buff[ 100 ];
		if( ha_gps_packet_ptr_ != nullptr )
		{
			snprintf( gps1_buff, sizeof( gps1_buff ), "GPS -> lat : %lf ; lon : %lf ; alt : %lf", ha_gps_packet_ptr->lat, ha_gps_packet_ptr->lon, ha_gps_packet_ptr->alt ) ;
			snprintf( gps2_buff, sizeof( gps2_buff ), "GPS -> nbsat : %d ; fixlvl : %d ; speed : %lf ", ha_gps_packet_ptr->satUsed,ha_gps_packet_ptr->quality, ha_gps_packet_ptr->groundSpeed ) ;
		}
		else
		{
			snprintf( gps1_buff, sizeof( gps1_buff ), "GPS -> lat : N/A ; lon : N/A ; alt : N/A" );
			snprintf( gps2_buff, sizeof( gps2_buff ), "GPS -> lnbsat : N/A ; fixlvl : N/A ; speed : N/A" );
		}

		draw_text( gyro_buff, 10, 410 );
		draw_text( accel_buff, 10, 420 );
		draw_text( odo_buff, 10, 430 );
		draw_text( gps1_buff, 10, 440 );
		draw_text( gps2_buff, 10, 450 );

		// ##############################################
		ApiPostPacketPtr api_post_packet_ptr = nullptr;

		api_post_packet_ptr_access_.lock();
		api_post_packet_ptr = api_post_packet_ptr_;
		api_post_packet_ptr_access_.unlock();

		if( api_post_packet_ptr != nullptr )
		{
			for( uint i = 0 ; i < api_post_packet_ptr->postList.size() ; i++ )
			{
				if( api_post_packet_ptr->postList[ i ].postType == ApiPostPacket::PostType::RED )
				{
					draw_red_post( static_cast<int>( api_post_packet_ptr->postList[ i ].x * 100.0 ), static_cast<int>( api_post_packet_ptr->postList[ i ].y * 100.0 ) );
				}
			}
		}

		// ##############################################

		static int flying_pixel_x = 0;

		if( flying_pixel_x > 800 )
		{
			flying_pixel_x = 0;
		}

		SDL_SetRenderDrawColor( renderer_, 200, 150, 125, 255 );
		SDL_Rect flying_pixel;
		flying_pixel.w = 1;
		flying_pixel.h = 1;
		flying_pixel.y = 482;
		flying_pixel.x = flying_pixel_x;

		flying_pixel_x++;

		SDL_RenderFillRect(renderer_, &flying_pixel);

		SDL_RenderPresent( renderer_ );

		// compute wait time
		milliseconds end_ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
		int64_t end_now = static_cast<int64_t>( end_ms.count() );
		int64_t wait_time = nextTick - end_now;

		if( wait_time <= 0 )
		{
			wait_time = 10;
		}

		//std::cout << "display time took " << display_time << " ms so wait_time is " << wait_time << " ms " << std::endl;

		// repeat keyboard reading for smoother command inputs
		readSDLKeyboard();
		manageSDLKeyboard();

		std::this_thread::sleep_for( std::chrono::milliseconds( wait_time / 2 ) );

		readSDLKeyboard();
		manageSDLKeyboard();

		std::this_thread::sleep_for( std::chrono::milliseconds( wait_time / 2 ) );
	}

	threadStarted_ = false;
	stopThreadAsked_ = false;

	exitSDL();

	std::cout << "Stopping main thread." << std::endl;
}

// #################################################
//
void Core::draw_text( char buffer[100], int x, int y )
{
	SDL_Surface* surfaceMessageAccel = TTF_RenderText_Solid( ttf_font_, buffer, sdl_color_white_ );
	SDL_Texture* messageAccel = SDL_CreateTextureFromSurface( renderer_, surfaceMessageAccel );

	SDL_FreeSurface( surfaceMessageAccel );

	SDL_Rect message_rect_accel;
	message_rect_accel.x = x;
	message_rect_accel.y = y;

	SDL_QueryTexture( messageAccel, NULL, NULL, &message_rect_accel.w, &message_rect_accel.h );
	SDL_RenderCopy( renderer_, messageAccel, NULL, &message_rect_accel );

	SDL_DestroyTexture( messageAccel );
}

// #################################################
//
void Core::draw_lidar( uint16_t lidar_distance_[ 271 ] )
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

			double x = 400.0 - x_cos;
			double y = 400.0 - y_sin;

			SDL_SetRenderDrawColor( renderer_, 255, 255, 255, 255 );
			SDL_Rect lidar_pixel;

			lidar_pixel.w = 1;
			lidar_pixel.h = 1;
			lidar_pixel.x = static_cast<int>( x );
			lidar_pixel.y = static_cast<int>( y );

			SDL_RenderFillRect( renderer_, &lidar_pixel );
		}
	}
}

// #################################################
//
void Core::draw_red_post( int x, int y )
{
	SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 255);
	SDL_Rect rp;
	rp.w = 2;
	rp.h = 2;
	rp.y = 400 - x - 1;
	rp.x = 400 - y - 1;

	SDL_RenderFillRect( renderer_, &rp );
}

// #################################################
//
void Core::draw_robot()
{
	SDL_SetRenderDrawColor( renderer_, 200, 200, 200, 255 );
	SDL_Rect main;
	main.w = 42;
	main.h = 80;
	main.y = 480 - main.h;
	main.x = 400 - ( main.w / 2);

	SDL_RenderFillRect( renderer_, &main );

	SDL_SetRenderDrawColor( renderer_, 100, 100, 100, 255 );
	SDL_Rect flw;
	flw.w = 8;
	flw.h = 20;
	flw.y = 480 - 75;
	flw.x = 400 - 21;

	SDL_RenderFillRect( renderer_, &flw );

	SDL_SetRenderDrawColor( renderer_, 100, 100, 100, 255 );
	SDL_Rect frw;
	frw.w = 8;
	frw.h = 20;
	frw.y = 480 - 75;
	frw.x = 400 + 21 - 8;

	SDL_RenderFillRect( renderer_, &frw );

	SDL_SetRenderDrawColor( renderer_, 100, 100, 100, 255 );
	SDL_Rect rlw;
	rlw.w = 8;
	rlw.h = 20;
	rlw.y = 480 - 5 - 20;
	rlw.x = 400 - 21;

	SDL_RenderFillRect( renderer_, &rlw );

	SDL_SetRenderDrawColor( renderer_, 100, 100, 100, 255 );
	SDL_Rect rrw;
	rrw.w = 8;
	rrw.h = 20;
	rrw.y = 480 - 5 -20;
	rrw.x = 400 + 21 - 8;

	SDL_RenderFillRect( renderer_, &rrw );

	SDL_SetRenderDrawColor( renderer_, 120, 120, 120, 255 );
	SDL_Rect lidar;
	lidar.w = 8;
	lidar.h = 8;
	lidar.y = 480 - 80 - 8;
	lidar.x = 400 - 4;

	SDL_RenderFillRect( renderer_, &lidar );
}

// #################################################
//
void Core::draw_images( )
{
	SDL_Surface* left_image;

	SDL_Surface* right_image;

	#if SDL_BYTEORDER == SDL_BIG_ENDIAN
		Uint32 rmask = 0xff000000;
		Uint32 gmask = 0x00ff0000;
		Uint32 bmask = 0x0000ff00;
		Uint32 amask = 0x000000ff;
	#else
		Uint32 rmask = 0x000000ff;
		Uint32 gmask = 0x0000ff00;
		Uint32 bmask = 0x00ff0000;
		Uint32 amask = 0xff000000;
	#endif

	last_images_buffer_access_.lock();

	if( last_image_type_ == ApiStereoCameraPacket::ImageType::RAW_IMAGES or last_image_type_ == ApiStereoCameraPacket::ImageType::RAW_IMAGES_ZLIB )
	{
		left_image = SDL_CreateRGBSurfaceFrom( last_images_buffer_, 752, 480, 3 * 8, 752 * 3, rmask, gmask, bmask, amask );
		right_image = SDL_CreateRGBSurfaceFrom( last_images_buffer_ + ( 752 * 480 * 3 ), 752, 480, 3 * 8, 752 * 3, rmask, gmask, bmask, amask );
	}
	else
	{
		left_image = SDL_CreateRGBSurfaceFrom( last_images_buffer_, 376, 240, 3 * 8, 376 * 3, rmask, gmask, bmask, amask );
		right_image = SDL_CreateRGBSurfaceFrom( last_images_buffer_ + ( 376 * 240 * 3 ), 376, 240, 3 * 8, 376 * 3, rmask, gmask, bmask, amask );
	}

	last_images_buffer_access_.unlock();

	SDL_Rect left_rect = { 400 - 376 - 10, 485, 376, 240 };

	SDL_Rect right_rect = { 400 + 10, 485, 376, 240 };

	SDL_Texture * left_texture = SDL_CreateTextureFromSurface( renderer_, left_image );

	SDL_Texture * right_texture = SDL_CreateTextureFromSurface( renderer_, right_image );

	SDL_RenderCopy( renderer_, left_texture, NULL, &left_rect );

	SDL_RenderCopy( renderer_, right_texture, NULL, &right_rect );
}

// #################################################
//
SDL_Window*
Core::initSDL( const char* name, int szX, int szY )
{
	std::cout << "Init SDL";

	SDL_Window *screen;
	std::cout << ".";

	SDL_Init( SDL_INIT_EVERYTHING );
	std::cout << ".";

	screen = SDL_CreateWindow( name, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, szX, szY, SDL_WINDOW_SHOWN );
	std::cout << ".";

	renderer_ =  SDL_CreateRenderer( screen, 0, SDL_RENDERER_ACCELERATED );
	std::cout << ".";

	TTF_Init();
	std::cout << ".";

	// Set render color to black ( background will be rendered in this color )
	SDL_SetRenderDrawColor( renderer_, 0, 0, 0, 255 );
	std::cout << ".";

	SDL_RenderClear( renderer_ );
	std::cout << ".";

	sdl_color_red_ = { 255, 0, 0, 0 };
	sdl_color_white_ = { 255, 255, 255, 0 };
	ttf_font_ = TTF_OpenFont("mono.ttf", 12);

	if (ttf_font_ == nullptr)
	{
		std::cerr << "Failed to load SDL Font! Error: " << TTF_GetError() << '\n';
	}

	std::cout << "DONE" << std::endl;

	return screen;
}

// #################################################
//
void
Core::exitSDL()
{
	SDL_Quit();
}

// #################################################
//
void
Core::readSDLKeyboard()
{
	SDL_Event event;

	while ( SDL_PollEvent( &event ) )
	{
		switch( event.type )
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
//
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

	if( sdlKey_[ SDL_SCANCODE_O ] == 1 )
	{
		asked_start_video_ = true;
	}

	if( sdlKey_[ SDL_SCANCODE_F ] == 1 )
	{
		asked_stop_video_ = true;
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

	last_motor_access_.lock();
	last_left_motor_ = static_cast<int8_t >( left * 2 );
	last_right_motor_ = static_cast<int8_t >( right * 2 );
	last_motor_access_.unlock();

	return keyPressed;
}

// #################################################
//
void
Core::manageReceivedPacket( BaseNaio01PacketPtr packetPtr )
{
	//std::cout << "Packet received id : " << static_cast<int>( packetPtr->getPacketId() ) << std::endl;

	if( std::dynamic_pointer_cast<HaLidarPacket>( packetPtr )  )
	{
		HaLidarPacketPtr haLidarPacketPtr = std::dynamic_pointer_cast<HaLidarPacket>( packetPtr );

		ha_lidar_packet_ptr_access_.lock();
		ha_lidar_packet_ptr_ = haLidarPacketPtr;
		ha_lidar_packet_ptr_access_.unlock();
	}
	else if( std::dynamic_pointer_cast<HaGyroPacket>( packetPtr )  )
	{
		HaGyroPacketPtr haGyroPacketPtr = std::dynamic_pointer_cast<HaGyroPacket>( packetPtr );

		ha_gyro_packet_ptr_access_.lock();
		ha_gyro_packet_ptr_ = haGyroPacketPtr;
		ha_gyro_packet_ptr_access_.unlock();
	}
	else if( std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr )  )
	{
		HaAcceleroPacketPtr haAcceleroPacketPtr = std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr );

		ha_accel_packet_ptr_access_.lock();
		ha_accel_packet_ptr_ = haAcceleroPacketPtr;
		ha_accel_packet_ptr_access_.unlock();
	}
	else if( std::dynamic_pointer_cast<HaOdoPacket>( packetPtr )  )
	{
		HaOdoPacketPtr haOdoPacketPtr = std::dynamic_pointer_cast<HaOdoPacket>( packetPtr );

		ha_odo_packet_ptr_access.lock();
		ha_odo_packet_ptr_ = haOdoPacketPtr;
		ha_odo_packet_ptr_access.unlock();
	}
	else if( std::dynamic_pointer_cast<ApiPostPacket>( packetPtr )  )
	{
		ApiPostPacketPtr apiPostPacketPtr = std::dynamic_pointer_cast<ApiPostPacket>( packetPtr );

		api_post_packet_ptr_access_.lock();
		api_post_packet_ptr_ = apiPostPacketPtr;
		api_post_packet_ptr_access_.unlock();
	}
	else if( std::dynamic_pointer_cast<HaGpsPacket>( packetPtr )  )
	{
		HaGpsPacketPtr haGpsPacketPtr = std::dynamic_pointer_cast<HaGpsPacket>( packetPtr );

		ha_gps_packet_ptr_access_.lock();
		ha_gps_packet_ptr_ = haGpsPacketPtr;
		ha_gps_packet_ptr_access_.unlock();
	}
	else if( std::dynamic_pointer_cast<ApiStereoCameraPacket>( packetPtr )  )
	{
		ApiStereoCameraPacketPtr api_stereo_camera_packet_ptr = std::dynamic_pointer_cast<ApiStereoCameraPacket>( packetPtr );

		milliseconds now_ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
		last_image_received_time_ = static_cast<int64_t>( now_ms.count() );

		api_stereo_camera_packet_ptr_access_.lock();
		api_stereo_camera_packet_ptr_ = api_stereo_camera_packet_ptr;
		api_stereo_camera_packet_ptr_access_.unlock();
	}

}

// #################################################
//
void
Core::joinMainThread()
{
	graphicThread_.join();
}

// #################################################
//
void Core::joinServerReadThread()
{
	serverReadThread_.join();
}

// #################################################
//
void Core::image_server_thread( )
{
	imageServerReadthreadStarted_ = false;
	imageServerWriteThreadStarted_ = false;

	stopImageServerReadThreadAsked_ = false;
	stopImageServerWriteThreadAsked_ = false;

	stopImageServerThreadAsked_ = false;
	imageServerThreadStarted_ = true;

	struct sockaddr_in imageServer;

	//Create socket
	image_socket_desc_ = socket( AF_INET, SOCK_STREAM, 0 );

	if ( image_socket_desc_ == -1 )
	{
		std::cout << "Could not create socket" << std::endl;
	}

	imageServer.sin_addr.s_addr = inet_addr( hostAdress_.c_str() );
	imageServer.sin_family = AF_INET;
	imageServer.sin_port = htons( static_cast<uint16_t>( hostPort_ + 2 ) );

	//Connect to remote server
	if ( connect( image_socket_desc_, ( struct sockaddr * ) &imageServer, sizeof( imageServer ) ) < 0 )
	{
		puts( "image connect error" );
	}
	else
	{
		puts( "Connected image\n" );
		imageSocketConnected_ = true;
	}

	image_prepared_thread_ = std::thread( &Core::image_preparer_thread, this );

	std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 50 ) ) );

	imageServerReadThread_ = std::thread( &Core::image_server_read_thread, this );

	std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 50 ) ) );

	imageServerWriteThread_ = std::thread( &Core::image_server_write_thread, this );

	std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 50 ) ) );

	while( not stopImageServerThreadAsked_ )
	{
		std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 500 ) ) );
	}

	imageServerThreadStarted_ = false;
	stopImageServerThreadAsked_ = false;
}

// #################################################
//
void Core::image_server_read_thread( )
{
	imageServerReadthreadStarted_ = true;

	uint8_t receiveBuffer[ 4000000 ];

	while( !stopImageServerReadThreadAsked_ )
	{
		// any time : read incoming messages.
		int readSize = (int) read( image_socket_desc_, receiveBuffer, 4000000 );

		if (readSize > 0)
		{
			bool packetHeaderDetected = false;

			bool atLeastOnePacketReceived = imageNaioCodec_.decode( receiveBuffer, static_cast<uint>( readSize ), packetHeaderDetected );

			// manage received messages
			if ( atLeastOnePacketReceived == true )
			{
				for ( auto &&packetPtr : imageNaioCodec_.currentBasePacketList )
				{
					if( std::dynamic_pointer_cast<ApiStereoCameraPacket>( packetPtr )  )
					{
						ApiStereoCameraPacketPtr api_stereo_camera_packet_ptr = std::dynamic_pointer_cast<ApiStereoCameraPacket>( packetPtr );

						milliseconds now_ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
						last_image_received_time_ = static_cast<int64_t>( now_ms.count() );

						api_stereo_camera_packet_ptr_access_.lock();
						api_stereo_camera_packet_ptr_ = api_stereo_camera_packet_ptr;
						api_stereo_camera_packet_ptr_access_.unlock();
					}
				}

				imageNaioCodec_.currentBasePacketList.clear();
			}
		}

		std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( WAIT_SERVER_IMAGE_TIME_RATE_MS ) ) );
	}

	imageServerReadthreadStarted_ = false;
	stopImageServerReadThreadAsked_= false;
}

// #################################################
// use only for server socket watchdog
void Core::image_server_write_thread( )
{
	imageServerWriteThreadStarted_ = true;

	while( !stopImageServerWriteThreadAsked_ )
	{
		if( imageSocketConnected_ )
		{
			ApiWatchdogPacketPtr api_watchdog_packet_ptr = std::make_shared<ApiWatchdogPacket>( 42 );

			cl_copy::BufferUPtr buffer = api_watchdog_packet_ptr->encode();

			write( image_socket_desc_, buffer->data(), buffer->size() );
		}

		std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( IMAGE_SERVER_WATCHDOG_SENDING_RATE_MS ) ) );
	}

	imageServerWriteThreadStarted_ = false;
	stopImageServerWriteThreadAsked_ = false;
}

// #################################################
//
void Core::image_preparer_thread( )
{
	Bytef zlibUncompressedBytes[ 4000000l ];

	while ( true )
	{
		std::this_thread::sleep_for( std::chrono::milliseconds(static_cast<int64_t>( IMAGE_PREPARING_RATE_MS ) ) );

		ApiStereoCameraPacketPtr api_stereo_camera_packet_ptr = nullptr;

		api_stereo_camera_packet_ptr_access_.lock();

		if ( api_stereo_camera_packet_ptr_ != nullptr )
		{
			last_image_type_ = api_stereo_camera_packet_ptr_->imageType;

			api_stereo_camera_packet_ptr = api_stereo_camera_packet_ptr_;

			api_stereo_camera_packet_ptr_ = nullptr;
		}

		api_stereo_camera_packet_ptr_access_.unlock();

		if ( api_stereo_camera_packet_ptr != nullptr )
		{
			cl_copy::BufferUPtr bufferUPtr = std::move( api_stereo_camera_packet_ptr->dataBuffer );

			if ( last_image_type_ == ApiStereoCameraPacket::ImageType::RAW_IMAGES_ZLIB or
				 last_image_type_ == ApiStereoCameraPacket::ImageType::RECTIFIED_COLORIZED_IMAGES_ZLIB )
			{
				uLong sizeDataUncompressed = 0l;

				uncompress( (Bytef *) zlibUncompressedBytes, &sizeDataUncompressed, bufferUPtr->data(),
						   static_cast<uLong>( bufferUPtr->size() ) );

				last_images_buffer_access_.lock();

				if ( last_image_type_ == ApiStereoCameraPacket::ImageType::RAW_IMAGES_ZLIB )
				{
					// don't know how to display 8bits image with sdl...
					for ( uint i = 0; i < sizeDataUncompressed; i++ )
					{
						last_images_buffer_[ ( i * 3 ) + 0 ] = zlibUncompressedBytes[ i ];
						last_images_buffer_[ ( i * 3 ) + 1 ] = zlibUncompressedBytes[ i ];
						last_images_buffer_[ ( i * 3 ) + 2 ] = zlibUncompressedBytes[ i ];
					}
				}
				else
				{
					for ( uint i = 0; i < sizeDataUncompressed; i++ )
					{
						last_images_buffer_[ i ] = zlibUncompressedBytes[i];
					}
				}

				last_images_buffer_access_.unlock();
			}
			else
			{
				last_images_buffer_access_.lock();

				if ( last_image_type_ == ApiStereoCameraPacket::ImageType::RAW_IMAGES )
				{
					// don't know how to display 8bits image with sdl...
					for (uint i = 0; i < bufferUPtr->size(); i++)
					{
						last_images_buffer_[ ( i * 3 ) + 0 ] = bufferUPtr->at( i );
						last_images_buffer_[ ( i * 3 ) + 1 ] = bufferUPtr->at( i );
						last_images_buffer_[ ( i * 3 ) + 2 ] = bufferUPtr->at( i );
					}
				}
				else
				{
					for ( uint i = 0; i < bufferUPtr->size(); i++ )
					{
						last_images_buffer_[ i ] = bufferUPtr->at( i );
					}
				}

				last_images_buffer_access_.unlock();
			}
		}
		else
		{
			milliseconds now_ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
			int64_t now = static_cast<int64_t>( now_ms.count() );

			int64_t diff_time = now - last_image_received_time_;

			if( diff_time > TIME_BEFORE_IMAGE_LOST_MS )
			{
				last_image_received_time_ = now;

				uint8_t fake = 0;

				last_images_buffer_access_.lock();

				for ( int i = 0 ; i < 721920 * 3 ; i++ )
				{
					if( fake >= 255 )
					{
						fake = 0;
					}

					last_images_buffer_[ i ] = fake;

					fake++;
				}

				last_images_buffer_access_.unlock();
			}
		}
	}
}

// #################################################
//
void Core::server_write_thread( )
{
	stopServerWriteThreadAsked_ = false;
	serverWriteThreadStarted_ = true;

	for( int i = 0 ; i < 100 ; i++ )
	{
		ApiMotorsPacketPtr first_packet = std::make_shared<ApiMotorsPacket>( 0, 0 );
		cl_copy::BufferUPtr first_buffer = first_packet->encode();
		write( socket_desc_, first_buffer->data(), first_buffer->size() );
	}

	while( not stopServerWriteThreadAsked_ )
	{
		last_motor_access_.lock();

		HaMotorsPacketPtr haMotorsPacketPtr = std::make_shared<HaMotorsPacket>( last_left_motor_, last_right_motor_ );

		last_motor_access_.unlock();

		sendPacketListAccess_.lock();

		sendPacketList_.push_back( haMotorsPacketPtr );

		for( auto&& packet : sendPacketList_ )
		{
			cl_copy::BufferUPtr buffer = packet->encode();

			int sentSize = (int)write( socket_desc_, buffer->data(), buffer->size() );

			(void)sentSize;
		}

		sendPacketList_.clear();

		sendPacketListAccess_.unlock();

		std::this_thread::sleep_for( std::chrono::milliseconds( SERVER_SEND_COMMAND_RATE_MS ) );
	}

	stopServerWriteThreadAsked_ = false;
	serverWriteThreadStarted_ = false;
}