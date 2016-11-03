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

// com_simu
#include "DriverSerial.hpp"
#include "DriverSocket.hpp"
#include "createLidarTrame.hpp"
#include <net/if.h>
#include <sys/fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "DriverSocket.hpp"

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
		last_image_received_time_{ 0 },
		com_simu_can_connected_{ false },
		com_simu_serial_connected_{ false },
		com_simu_image_to_core_client_connected_{ false },
		com_simu_image_to_core_buffer_updated_{ false }
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

	com_simu_last_odo_ticks_[0] = false;
	com_simu_last_odo_ticks_[1] = false;
	com_simu_last_odo_ticks_[2] = false;
	com_simu_last_odo_ticks_[3] = false;

	com_simu_remote_status_.pad_down = false;
	com_simu_remote_status_.pad_up = false;
	com_simu_remote_status_.pad_left = false;
	com_simu_remote_status_.pad_right = false;
	com_simu_remote_status_.analog_x = 63;
	com_simu_remote_status_.analog_y = 63;

	com_simu_remote_status_.teleco_self_id_6 = 255;
	com_simu_remote_status_.teleco_act_7 = 253;

	for( uint i = 0 ; i < 20 ; i++ )
	{
		com_simu_ihm_line_top_[ i ] = ' ';
		com_simu_ihm_line_bottom_[ i ] = ' ';
	}

	for( uint i = 20 ; i < 100 ; i++ )
	{
		com_simu_ihm_line_top_[ i ] = '\0';
		com_simu_ihm_line_bottom_[ i ] = '\0';
	}

	com_simu_ihm_button_status_.cancel = false;
	com_simu_ihm_button_status_.validate = false;
	com_simu_ihm_button_status_.minus = false;
	com_simu_ihm_button_status_.plus = false;
	com_simu_ihm_button_status_.left = false;
	com_simu_ihm_button_status_.right = false;
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

	std::cout << "Connecting to simu : " << hostAdress << ":" <<  hostPort << std::endl;

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
		puts( "Simu connect error" );
	}
	else
	{
		puts( "Simu Connected\n" );
		socketConnected_ = true;
	}

	// creates main thread
	graphicThread_ = std::thread( &Core::graphic_thread, this );

	serverReadThread_ = std::thread( &Core::server_read_thread, this );

	serverWriteThread_ = std::thread( &Core::server_write_thread, this );

	imageServerThread_ = std::thread( &Core::image_server_thread, this );

	// com_simu
	com_simu_create_virtual_can( );

	com_simu_create_serial_thread_ = std::thread( &Core::com_simu_create_serial_thread_function, this );

	std::this_thread::sleep_for( std::chrono::milliseconds( 5000 ) );

	com_simu_read_serial_thread_ = std::thread( &Core::com_simu_read_serial_thread_function, this );

	com_simu_lidar_to_core_thread_ = std::thread( &Core::com_simu_lidar_to_core_thread_function, this );

	com_simu_connect_can( );

	com_simu_read_can_thread_ = std::thread( &Core::com_simu_read_can_thread_function, this );

	com_simu_remote_thread_= std::thread( &Core::com_simu_remote_thread_function, this );

	// bridge simulatos images
	com_simu_image_to_core_read_thread_ = std::thread( &Core::com_simu_image_to_core_read_thread_function, this );
	com_simu_image_to_core_write_thread_ = std::thread( &Core::com_simu_image_to_core_write_thread_function, this );

	(void)com_simu_image_to_core_read_thread_;
	(void)com_simu_image_to_core_write_thread_;
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

	std::cout << "Stopping server read thread !" << std::endl;
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

		draw_text( com_simu_ihm_line_top_, 500, 410 );
		draw_text( com_simu_ihm_line_bottom_, 500, 420 );

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

	(void)( system( "pkill socat" ) + 1 );
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

	uint8_t last_images_buffer_to_display[ 752 * 480 * 3 * 2 ];

	for( uint i = 0 ; i < ( 752 * 480 * 3 * 2 ) ; i++ )
	{
		last_images_buffer_to_display[ i ] = last_images_buffer_[ i ];
	}

	last_images_buffer_access_.unlock();

	if( last_image_type_ == ApiStereoCameraPacket::ImageType::RAW_IMAGES or last_image_type_ == ApiStereoCameraPacket::ImageType::RAW_IMAGES_ZLIB )
	{
		left_image = SDL_CreateRGBSurfaceFrom( last_images_buffer_to_display, 752, 480, 3 * 8, 752 * 3, rmask, gmask, bmask, amask );
		right_image = SDL_CreateRGBSurfaceFrom( last_images_buffer_to_display + ( 752 * 480 * 3 ), 752, 480, 3 * 8, 752 * 3, rmask, gmask, bmask, amask );
	}
	else
	{
		left_image = SDL_CreateRGBSurfaceFrom( last_images_buffer_to_display, 376, 240, 3 * 8, 376 * 3, rmask, gmask, bmask, amask );
		right_image = SDL_CreateRGBSurfaceFrom( last_images_buffer_to_display + ( 376 * 240 * 3 ), 376, 240, 3 * 8, 376 * 3, rmask, gmask, bmask, amask );
	}

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

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_F ] == 1 )
	{
		asked_stop_video_ = true;

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_UP ] == 1 and sdlKey_[ SDL_SCANCODE_LEFT ] == 1 )
	{
		com_simu_remote_status_.analog_x = 250;
		com_simu_remote_status_.analog_y = 5;

		left = 32;
		right = 63;
		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_UP ] == 1 and sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 )
	{
		com_simu_remote_status_.analog_x = 250;
		com_simu_remote_status_.analog_y = 250;

		left = 63;
		right = 32;
		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 and sdlKey_[ SDL_SCANCODE_LEFT ] == 1 )
	{
		com_simu_remote_status_.analog_x = 5;
		com_simu_remote_status_.analog_y = 5;

		left = -32;
		right = -63;
		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 and sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 )
	{
		com_simu_remote_status_.analog_x = 5;
		com_simu_remote_status_.analog_y = 250;

		left = -63;
		right = -32;
		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_UP ] == 1 )
	{
		com_simu_remote_status_.analog_x = 250;

		left = 63;
		right = 63;
		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 )
	{
		com_simu_remote_status_.analog_x = 5;

		left = -63;
		right = -63;
		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_LEFT ] == 1 )
	{
		com_simu_remote_status_.analog_y = 5;

		left = -63;
		right = 63;
		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 )
	{
		com_simu_remote_status_.analog_y = 250;

		left = 63;
		right = -63;
		keyPressed = true;
	}

	// ########################
	//         COM_SIMU
	// ########################

	if( sdlKey_[ SDL_SCANCODE_KP_7 ] == 1 )
	{
		com_simu_remote_status_.secu_left = true;

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_9 ] == 1 )
	{
		com_simu_remote_status_.secu_right = true;

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_1 ] == 1 )
	{
		com_simu_remote_status_.arr_left = true;

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_3 ] == 1 )
	{
		com_simu_remote_status_.arr_right = true;

		keyPressed = true;
	}


	if( sdlKey_[ SDL_SCANCODE_KP_4 ] == 1 )
	{
		com_simu_remote_status_.pad_left = true;

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_6 ] == 1 )
	{
		com_simu_remote_status_.pad_right = true;

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_8 ] == 1 )
	{
		com_simu_remote_status_.pad_up = true;

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_2 ] == 1 )
	{
		com_simu_remote_status_.pad_down = true;

		keyPressed = true;
	}

	if( sdlKey_[ SDL_SCANCODE_PAGEUP ] == 1 )
	{
		com_simu_ihm_button_status_.plus = true;

		keyPressed = true;
	}
	else
	{
		com_simu_ihm_button_status_.plus = false;
	}

	if( sdlKey_[ SDL_SCANCODE_PAGEDOWN] == 1 )
	{
		com_simu_ihm_button_status_.minus = true;

		keyPressed = true;
	}
	else
	{
		com_simu_ihm_button_status_.minus = false;
	}

	if( sdlKey_[ SDL_SCANCODE_HOME] == 1 )
	{
		com_simu_ihm_button_status_.left = true;

		keyPressed = true;
	}
	else
	{
		com_simu_ihm_button_status_.left = false;
	}

	if( sdlKey_[ SDL_SCANCODE_END] == 1 )
	{
		com_simu_ihm_button_status_.right = true;

		keyPressed = true;
	}
	else
	{
		com_simu_ihm_button_status_.right = false;
	}

	if( sdlKey_[ SDL_SCANCODE_INSERT] == 1 )
	{
		com_simu_ihm_button_status_.validate = true;

		keyPressed = true;
	}
	else
	{
		com_simu_ihm_button_status_.validate = false;
	}

	if( sdlKey_[ SDL_SCANCODE_DELETE] == 1 )
	{
		com_simu_ihm_button_status_.cancel = true;

		keyPressed = true;
	}
	else
	{
		com_simu_ihm_button_status_.cancel = false;
	}

	// #######################

	if( sdlKey_[ SDL_SCANCODE_LEFT ] == 0 and sdlKey_[ SDL_SCANCODE_RIGHT ] == 0 )
	{
		com_simu_remote_status_.analog_y = 128;
	}

	if( sdlKey_[ SDL_SCANCODE_UP ] == 0 and sdlKey_[ SDL_SCANCODE_DOWN ] == 0 )
	{
		com_simu_remote_status_.analog_x = 128;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_6 ] == 0 )
	{
		com_simu_remote_status_.pad_right = false;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_8 ] == 0 )
	{
		com_simu_remote_status_.pad_up = false;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_2 ] == 0 )
	{
		com_simu_remote_status_.pad_down = false;
	}

	if( sdlKey_[ SDL_SCANCODE_KP_4 ] == 0 )
	{
		com_simu_remote_status_.pad_left = false;
	}

	return keyPressed;
}

// #################################################
//
void
Core::manageReceivedPacket( BaseNaio01PacketPtr packetPtr )
{
	// std::cout << "Packet received id : " << static_cast<int>( packetPtr->getPacketId() ) << std::endl;

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

		com_simu_transform_and_write_to_can( packetPtr );
	}
	else if( std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr )  )
	{
		HaAcceleroPacketPtr haAcceleroPacketPtr = std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr );

		ha_accel_packet_ptr_access_.lock();
		ha_accel_packet_ptr_ = haAcceleroPacketPtr;
		ha_accel_packet_ptr_access_.unlock();

		com_simu_transform_and_write_to_can( packetPtr );
	}
	else if( std::dynamic_pointer_cast<HaOdoPacket>( packetPtr )  )
	{
		HaOdoPacketPtr haOdoPacketPtr = std::dynamic_pointer_cast<HaOdoPacket>( packetPtr );

		ha_odo_packet_ptr_access.lock();
		ha_odo_packet_ptr_ = haOdoPacketPtr;
		ha_odo_packet_ptr_access.unlock();

		com_simu_transform_and_write_to_can( packetPtr );
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
		std::cout << "Could not create image socket" << std::endl;
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
		image_socket_desc_access_.lock();

		// any time : read incoming messages.
		//int readSize = (int) read( image_socket_desc_, receiveBuffer, 4000000 );
		int readSize = (int)recv( image_socket_desc_, receiveBuffer, 16384, MSG_DONTWAIT );

		image_socket_desc_access_.unlock();

		if ( readSize > 0 )
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

						last_image_received_time_ = get_now_ms();

						com_simu_image_to_core_buffer_access_.lock();

						size_t buffer_size = api_stereo_camera_packet_ptr->dataBuffer->size();

						com_simu_image_to_core_buffer_ = cl_copy::unique_buffer( buffer_size );

						for( int i = 0 ; i < buffer_size ; i++ )
						{
							(*com_simu_image_to_core_buffer_)[ i ] = api_stereo_camera_packet_ptr->dataBuffer->at( i );
						}

						com_simu_image_to_core_buffer_updated_ = true;

						com_simu_image_to_core_buffer_access_.unlock();

						api_stereo_camera_packet_ptr_access_.lock();
						api_stereo_camera_packet_ptr_ = api_stereo_camera_packet_ptr;
						api_stereo_camera_packet_ptr_access_.unlock();
					}
				}

				imageNaioCodec_.currentBasePacketList.clear();
			}
		}
		//std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( WAIT_SERVER_IMAGE_TIME_RATE_MS ) ) );
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

			int total_written_bytes = 0;
			int sentSize = 0;
			int nb_tries = 0;
			int max_tries = 50;

			image_socket_desc_access_.lock();

			while( total_written_bytes < buffer->size() and nb_tries < max_tries )
			{
				sentSize = (int) write( image_socket_desc_, buffer->data() + total_written_bytes, buffer->size() - total_written_bytes );

				if( sentSize < 0 )
				{
					nb_tries++;
					std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 10 ) ) );
				}
				else
				{
					total_written_bytes = total_written_bytes + sentSize;
					nb_tries = 0;
				}
			}

			image_socket_desc_access_.unlock();
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
	while ( true )
	{
		api_stereo_camera_packet_ptr_access_.lock();

		if ( api_stereo_camera_packet_ptr_ == nullptr )
		{
			int64_t now = get_now_ms();

			int64_t diff_time = now - last_image_received_time_;

			if ( diff_time > TIME_BEFORE_IMAGE_LOST_MS )
			{
				last_image_received_time_ = now;

				uint8_t fake = 0;

				last_images_buffer_access_.lock();

				for ( int i = 0; i < 721920 * 3; i++ )
				{
					if ( fake >= 255 )
					{
						fake = 0;
					}

					last_images_buffer_[i] = fake;

					fake++;
				}

				last_images_buffer_access_.unlock();
			}
		}
		else
		{
			last_image_type_ = api_stereo_camera_packet_ptr_->imageType;

			size_t buffer_size = api_stereo_camera_packet_ptr_->dataBuffer->size();

			cl_copy::BufferUPtr prepared_image_buffer = cl_copy::unique_buffer(buffer_size);

			for (int i = 0; i < buffer_size; i++)
			{
				(*prepared_image_buffer)[i] = api_stereo_camera_packet_ptr_->dataBuffer->at(i);
			}

			last_images_buffer_access_.lock();

			// don't know how to display 8bits image with sdl...
			for (uint i = 0; i < prepared_image_buffer->size(); i++)
			{
				last_images_buffer_[(i * 3) + 0] = prepared_image_buffer->at(i);
				last_images_buffer_[(i * 3) + 1] = prepared_image_buffer->at(i);
				last_images_buffer_[(i * 3) + 2] = prepared_image_buffer->at(i);
			}

			last_images_buffer_access_.unlock();

			api_stereo_camera_packet_ptr_ = nullptr;
		}

		api_stereo_camera_packet_ptr_access_.unlock();

		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>( 20 )));
	}
}

// #################################################
//
void Core::server_write_thread( )
{
	std::cout << "Staring server write thread." << std::endl;

	stopServerWriteThreadAsked_ = false;
	serverWriteThreadStarted_ = true;

//	for( int i = 0 ; i < 100 ; i++ )
//	{
//		ApiMotorsPacketPtr first_packet = std::make_shared<ApiMotorsPacket>( 0, 0 );
//		cl_copy::BufferUPtr first_buffer = first_packet->encode();
//		write( socket_desc_, first_buffer->data(), first_buffer->size() );
//	}

	while( not stopServerWriteThreadAsked_ )
	{
		sendPacketListAccess_.lock();

		//std::cout << "server_write_thread com_simu_serial_connected_ : " << (int)com_simu_serial_connected_ << std::endl;

		if( socketConnected_ )
		{
			if ( not com_simu_serial_connected_ )
			{
				HaMotorsPacketPtr motor_packet = std::make_shared<HaMotorsPacket>( 0, 0 );

				sendPacketList_.push_back( motor_packet );
			}

			for ( auto &&packet : sendPacketList_ )
			{
				cl_copy::BufferUPtr buffer = packet->encode();

				int total_written_bytes = 0;
				int sentSize = 0;
				int nb_tries = 0;
				int max_tries = 50;

				while( total_written_bytes < buffer->size() and nb_tries < max_tries )
				{
					sentSize = (int) send(socket_desc_, buffer->data() + total_written_bytes, buffer->size() - total_written_bytes, 0 );

					if( sentSize < 0 )
					{
						nb_tries++;
						std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 10 ) ) );
					}
					else
					{
						total_written_bytes = total_written_bytes + sentSize;
						nb_tries = 0;
					}
				}

				//std::cout << "server_write_thread : " << sentSize << std::endl;
			}
		}

		sendPacketList_.clear();

		sendPacketListAccess_.unlock();

		std::this_thread::sleep_for( std::chrono::milliseconds( SERVER_SEND_COMMAND_RATE_MS ) );
	}

	stopServerWriteThreadAsked_ = false;
	serverWriteThreadStarted_ = false;

	std::cout << "Stopping server write thread." << std::endl;
}

// #################################################
//
void Core::com_simu_create_virtual_can( )
{
	(void)( system( "modprobe can" ) + 1 );
	(void)( system( "modprobe can_raw" ) + 1 );
	(void)( system( "modprobe vcan" ) + 1 );
	(void)( system( "ip link add dev can0 type vcan" ) + 1 );
	(void)( system( "ip link set up can0" ) + 1 );
}

// #################################################
//
void Core::com_simu_create_serial_thread_function( )
{
	(void)( system( "socat PTY,link=/dev/ttyS0,raw,echo=0 PTY,link=/tmp/ttyS1,raw,echo=0" ) + 1 );
}

// #################################################
//
void Core::com_simu_read_serial_thread_function( )
{
	std::cout << "Staring read serial thread." << std::endl;

	unsigned char b[200];

	int motorNumber;
	int posInEntete = 0;
	char motors[ 3 ] = { 0 };

	int serialPort = serialport_init( "/tmp/ttyS1", 9600 );

	if( serialPort == -1 )
	{
		std::cout << "/tmp/ttyS1 failed too !" << std::endl;
	}
	else
	{
		std::cout << "connected to /tmp/ttyS1" << std::endl;
	}

	while ( 1 )
	{
		ssize_t serial_read_size = read( serialPort, b, 1 );

		if ( serial_read_size > 0 )
		{

			// std::cout << "serial_read_size : " << (int)serial_read_size << std::endl;

			com_simu_serial_connected_ = true;

			if ( posInEntete == 2 )
			{
				motors[ motorNumber ] = ( ( ( char ) b[ 0 ] ) * 2 ) - 128;

				if ( motorNumber == 2 )
				{
					// std::cout << "ha motors created : " << static_cast<int>( motors[ 2 ] ) << " " << static_cast<int>( motors[ 1 ] ) << std::endl;

					HaMotorsPacketPtr haMotorsPacketPtr = std::make_shared<HaMotorsPacket>( motors[ 2 ], motors[ 1 ] );

					sendPacketListAccess_.lock();

					sendPacketList_.push_back( haMotorsPacketPtr );

					sendPacketListAccess_.unlock();
				}

				posInEntete = 0;
			}

			if ( posInEntete == 1 )
			{
				if ( b[0] == 6 )
				{
					motorNumber = 1;
					posInEntete = 2;
				}
				else if ( b[0] == 7 )
				{
					motorNumber = 2;
					posInEntete = 2;
				}
				else
				{
					posInEntete = 0;
				}
			}

			if ( ( posInEntete == 0 ) && ( b[ 0 ] == 128 ) )
			{
				posInEntete = 1;
			}
		}
	}

	std::cout << "Stopping read serial thread." << std::endl;
}

// #################################################
//
void Core::com_simu_lidar_to_core_thread_function( )
{
	SOCKET sockLidarRobot = openSocketServer( COM_SIMU_PORT_CORE_LIDAR );

	sockLidarRobot = waitConnect( sockLidarRobot );

	printf( "Lidar connected to Core\n" );

	struct timespec timeInit;

	clock_gettime( CLOCK_MONOTONIC_RAW, &timeInit );

	unsigned char buffer[ 1000 ];
	char trame[ 10000 ];

	int lidar[271];
	int albedo[271];

	uint16_t nbMesures = 1;//à incrémenter à chaque mesure
	uint16_t nbTelegrammes = 1;//à incrémenter à chaque fois qu'on envoie une donnée

	printf( "listening Lidar\n" );

	while ( 1 )
	{
		memset( buffer, '\0', 1000 );

		int size = read( sockLidarRobot, buffer, 1000 );

		if ( size > 0 )
		{
			buffer[ size ] = '\0';

			if ( strncmp( "\x02sRN LMDscandata 1\x03", ( char* )buffer, strlen( "\x02sRN LMDscandata 1\x03" ) ) == 0 )
			{
				ha_lidar_packet_ptr_access_.lock( );

				if( ha_lidar_packet_ptr_ != nullptr )
				{
					for ( int i = 0 ; i < 271 ; i++ )
					{
						// 2 bytes
						lidar[ i ] = ha_lidar_packet_ptr_->distance[ i ]; // ( buffer[ 11 + ( 2 * i ) ] * 256 ) + ( buffer[ 11 + ( 2 * i ) + 1 ] );
					}
					//albedo
					for ( int i = 0 ; i < 271 ; i++ )
					{
						// 1 byte
						albedo[ i ] = ha_lidar_packet_ptr_->albedo[ i ]; // buffer[ 11 + ( 271 * 2 ) + i ];
					}

					nbMesures++;

					// buffer to socket;
					nbTelegrammes++;

					createTrame( lidar, albedo, trame, nbMesures, nbTelegrammes, timeInit );

					(void)( write( sockLidarRobot, trame, strlen( trame ) ) + 1 );
				}

				ha_lidar_packet_ptr_access_.unlock( );
			}
		}
		else
		{
			usleep( 1000 );
		}
	}
}

// #################################################
//
int Core::com_simu_connect_can( )
{
	struct sockaddr_can addr;
	struct ifreq ifr;

	const char *ifname = "can0";

	printf( "Connecting Can\n" );

	// Create the CAN socket
	com_simu_can_socket_ = socket( PF_CAN, SOCK_RAW, CAN_RAW );
	printf( "Can sock : %d\n", com_simu_can_socket_ );

	strcpy( ifr.ifr_name, ifname );
	ioctl( com_simu_can_socket_, SIOCGIFINDEX, &ifr );

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf( "Can : %s at index %d\n", ifname, ifr.ifr_ifindex );

	if( bind( com_simu_can_socket_, ( struct sockaddr * ) &addr, sizeof( addr ) ) < 0 )
	{
		perror( "Error in can socket bind" );
		return -2;
	}

	com_simu_can_connected_ = true;

	printf( "Can Connected\n" );

	return 1;
}


// #################################################
//
void Core::com_simu_transform_and_write_to_can( BaseNaio01PacketPtr packetPtr )
{
	if( std::dynamic_pointer_cast<HaOdoPacket>( packetPtr )  )
	{
		HaOdoPacketPtr haOdoPacketPtr = std::dynamic_pointer_cast<HaOdoPacket>( packetPtr );

		if( com_simu_last_ha_odo_packet_ptr_ != nullptr )
		{
			if( com_simu_last_ha_odo_packet_ptr_->fr != haOdoPacketPtr->fr )
			{
				com_simu_last_odo_ticks_[ 0 ] = not com_simu_last_odo_ticks_[ 0 ];
			}

			if( com_simu_last_ha_odo_packet_ptr_->fl != haOdoPacketPtr->fl )
			{
				com_simu_last_odo_ticks_[ 1 ] = not com_simu_last_odo_ticks_[ 1 ];
			}

			if( com_simu_last_ha_odo_packet_ptr_->rr != haOdoPacketPtr->rr )
			{
				com_simu_last_odo_ticks_[ 2 ] = not com_simu_last_odo_ticks_[ 2 ];
			}

			if( com_simu_last_ha_odo_packet_ptr_->rl != haOdoPacketPtr->rl )
			{
				com_simu_last_odo_ticks_[ 3 ] = not com_simu_last_odo_ticks_[ 3 ];
			}
		}

		uint8_t data[ 1 ];

		data[ 0 ] = 0x00;

		if( com_simu_last_odo_ticks_[ 0 ] )
		{
			data[ 0 ] = ( data[ 0 ] | ( 0x01 << 0 ) );
		}

		if( com_simu_last_odo_ticks_[ 1 ] )
		{
			data[ 0 ] = ( data[ 0 ] | ( 0x01 << 1 ) );
		}

		if( com_simu_last_odo_ticks_[ 2 ] )
		{
			data[ 0 ] = ( data[ 0 ] | ( 0x01 << 2 ) );
		}

		if( com_simu_last_odo_ticks_[ 3 ] )
		{
			data[ 0 ] = ( data[ 0 ] | ( 0x01 << 3 ) );
		}

		com_simu_send_can_packet( ComSimuCanMessageId::CAN_ID_GEN, ComSimuCanMessageType::CAN_MOT_CONS, data, 1 );

		com_simu_last_ha_odo_packet_ptr_ = haOdoPacketPtr;
	}
	else if( std::dynamic_pointer_cast<HaGyroPacket>( packetPtr )  )
	{
		HaGyroPacketPtr haGyroPacketPtr = std::dynamic_pointer_cast<HaGyroPacket>( packetPtr );

		uint8_t data[ 6 ];

		data[ 0 ] = ( haGyroPacketPtr->x >> 8 ) & 0xFF;
		data[ 1 ] = ( haGyroPacketPtr->x >> 0 ) & 0xFF;

		data[ 2 ] = ( haGyroPacketPtr->y >> 8 ) & 0xFF;
		data[ 3 ] = ( haGyroPacketPtr->y >> 0 ) & 0xFF;

		data[ 4 ] = ( haGyroPacketPtr->z >> 8 ) & 0xFF;
		data[ 5 ] = ( haGyroPacketPtr->z >> 0 ) & 0xFF;

		com_simu_send_can_packet( ComSimuCanMessageId::CAN_ID_IMU, ComSimuCanMessageType::CAN_IMU_GYRO, data, 6 );
	}
	else if( std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr )  )
	{
		HaAcceleroPacketPtr haAcceleroPacketPtr = std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr );

		uint8_t data[ 6 ];

		data[ 0 ] = ( haAcceleroPacketPtr->x >> 8 ) & 0xFF;
		data[ 1 ] = ( haAcceleroPacketPtr->x >> 0 ) & 0xFF;

		data[ 2 ] = ( haAcceleroPacketPtr->y >> 8 ) & 0xFF;
		data[ 3 ] = ( haAcceleroPacketPtr->y >> 0 ) & 0xFF;

		data[ 4 ] = ( haAcceleroPacketPtr->z >> 8 ) & 0xFF;
		data[ 5 ] = ( haAcceleroPacketPtr->z >> 0 ) & 0xFF;

		com_simu_send_can_packet( ComSimuCanMessageId::CAN_ID_IMU, ComSimuCanMessageType::CAN_IMU_ACC, data, 6 );
	}
}

// #################################################
//
void Core::com_simu_send_can_packet( ComSimuCanMessageId id, ComSimuCanMessageType id_msg, uint8_t data[], uint8_t len )
{
	struct can_frame frame;
	ssize_t nbytes = -1;
	int nbTests = 0;

	if ( not com_simu_can_connected_ )
	{
		return;
	}

	frame.can_id  = ( unsigned int )( id * 128 + id_msg );
	frame.can_dlc = len;

	for ( uint8_t i = 0 ; i < len ; i++ )
	{
		frame.data[i] = data[i];
	}

	while ( nbytes <= 0 && nbTests < 10 )
	{
		com_simu_can_socket_access_.lock();

		nbytes = write( com_simu_can_socket_, &frame, sizeof( struct can_frame ) );

		com_simu_can_socket_access_.unlock();

		nbTests++;
		//usleep( 200 );
	}

	if ( nbytes <= 0 )
	{
		std::cout << "Can write error." << std::endl;
	}
}

// #################################################
//
void Core::com_simu_remote_thread_function( )
{
	while( true )
	{
		send_remote_can_packet( CAN_TELECO_KEYS );

		std::this_thread::sleep_for( std::chrono::milliseconds( COM_SIMU_REMOTE_SEND_RATE_MS / 2 ) );

		send_keypad_can_packet( );

		std::this_thread::sleep_for( std::chrono::milliseconds( COM_SIMU_REMOTE_SEND_RATE_MS / 2 ) );
	}
}

// #################################################
//
void Core::com_simu_read_can_thread_function( )
{
	ssize_t bytesRead;

	struct can_frame frame;

	memset( &frame, 0, sizeof( frame ) );

	while( true )
	{
		if( com_simu_can_connected_ )
		{
			bytesRead = recv( com_simu_can_socket_, &frame, sizeof( frame ), 0 );

			if ( bytesRead > 0 )
			{
				if( ( ( frame.can_id ) >> 7 ) == CAN_ID_IHM )
				{
					if( ( ( frame.can_id ) % 16 ) == CAN_IHM_LCD )
					{
						uint8_t car_position = frame.data[ 1 ];
						char car = static_cast<char>( frame.data[ 2 ] );

						if( car_position < 16 )
						{
							com_simu_ihm_line_top_[ car_position ] = car;
						}
						else
						{
							com_simu_ihm_line_bottom_[ car_position - 40 ] = car;
						}
					}
				}
				else if( ( ( frame.can_id ) >> 7 ) == CAN_ID_TELECO )
				{
					// std::cout << "CAN_ID_TELECO" << std::endl;

					if( ( ( frame.can_id ) % 16 ) == CAN_TELECO_NUM_VERSION )
					{
						std::cout << "setting teleco act : " << static_cast<int>( frame.data[ 6 ] ) << " self_id : " << static_cast<int>(  frame.data[ 7 ] ) << std::endl;

						com_simu_remote_status_.teleco_self_id_6 = frame.data[ 7 ];

						send_remote_can_packet( CAN_TELECO_NUM_VERSION );
					}
				}
			}
		}
		else
		{
			std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
		}
	}
}


// #################################################
//
void Core::send_remote_can_packet( ComSimuCanMessageType message_type )
{
	uint8_t remote_data[ 8 ];

	if( message_type == ComSimuCanMessageType::CAN_TELECO_KEYS )
	{
		// B
		//		fillClick( (report[3]>>(TN_UP              -   9))%2 , tnData.btn.dpad_UP       );
		//		fillClick( (report[3]>>(TN_RIGHT           -   9))%2 , tnData.btn.dpad_RIGHT    );
		//		fillClick( (report[3]>>(TN_DOWN            -   9))%2 , tnData.btn.dpad_DOWN     );
		//		fillClick( (report[3]>>(TN_LEFT            -   9))%2 , tnData.btn.dpad_LEFT     );
		//
		//		fillClick( (report[3]>>(TN_ON_OFF          -   9))%2 , tnData.btn.on_off        );
		//		fillClick( (report[3]>>(TN_STOP            -   9))%2 , tnData.btn.stop          );
		//		fillClick( (report[3]>>(TN_GO              -   9))%2 , tnData.btn.go            );
		//
		//		fillClick( (report[2]>>(TN_ARR_LEFT        -   1))%2 , tnData.btn.arrLeft       );
		//		fillClick( (report[2]>>(TN_ARR_RIGHT       -   1))%2 , tnData.btn.arrRight      );
		//		fillClick( (report[2]>>(TN_SECU_LEFT       -   1))%2 , tnData.btn.secuLeft      );
		//		fillClick( (report[2]>>(TN_SECU_RIGHT      -   1))%2 , tnData.btn.secuRight     );
		//		fillClick( (report[2]>>(TN_TOOL_DOWN_LEFT  -   1))%2 , tnData.btn.toolDownLeft  );
		//		fillClick( (report[2]>>(TN_TOOL_DOWN_RIGHT -   1))%2 , tnData.btn.toolDownRight );
		//		fillClick( (report[2]>>(TN_TOOL_UP_LEFT    -   1))%2 , tnData.btn.toolUpLeft    );
		//		fillClick( (report[2]>>(TN_TOOL_UP_RIGHT   -   1))%2 , tnData.btn.toolUpRight   );

		uint8_t directional_cross = 0x00;
		uint8_t buttons1 = 0x00;

		// std::cout << "com_simu_remote_status_.teleco_act_7 : " << static_cast< int >( com_simu_remote_status_.teleco_act_7 ) << std::endl;

		if( com_simu_remote_status_.pad_up )
		{
			// std::cout << "com_simu_remote_status_.pad_up" << std::endl;

			directional_cross = ( directional_cross | ( 0x01 << 3 ) );
		}

		if( com_simu_remote_status_.pad_left )
		{
			// std::cout << "com_simu_remote_status_.pad_left" << std::endl;

			directional_cross = ( directional_cross | ( 0x01 << 4 ) );
		}

		if( com_simu_remote_status_.pad_right )
		{
			// std::cout << "com_simu_remote_status_.pad_right" << std::endl;

			directional_cross = ( directional_cross | ( 0x01 << 5 ) );
		}

		if( com_simu_remote_status_.pad_down )
		{
			// std::cout << "com_simu_remote_status_.pad_down" << std::endl;

			directional_cross = ( directional_cross | ( 0x01 << 6 ) );
		}

		if( com_simu_remote_status_.secu_left )
		{
			buttons1 = ( buttons1 | ( 0x01 << 0 ) );
		}

		if( com_simu_remote_status_.secu_right )
		{
			buttons1 = ( buttons1 | ( 0x01 << 1 ) );
		}

		if( com_simu_remote_status_.arr_left )
		{
			buttons1 = ( buttons1 | ( 0x01 << 2 ) );
		}

		if( com_simu_remote_status_.arr_right )
		{
			buttons1 = ( buttons1 | ( 0x01 << 3 ) );
		}

		remote_data[ 0 ] = com_simu_remote_status_.analog_x;
		remote_data[ 1 ] = com_simu_remote_status_.analog_y;

		remote_data[ 2 ] = buttons1;
		remote_data[ 3 ] = directional_cross;

		remote_data[ 4 ] = 0x00;
		remote_data[ 5 ] = 0x00;

		remote_data[ 6 ] = com_simu_remote_status_.teleco_self_id_6;
		remote_data[ 7 ] = com_simu_remote_status_.teleco_act_7;

		com_simu_send_can_packet( ComSimuCanMessageId::CAN_ID_TELECO, ComSimuCanMessageType::CAN_TELECO_KEYS, remote_data, 8 );
	}
	else if( message_type == ComSimuCanMessageType::CAN_TELECO_NUM_VERSION )
	{
		remote_data[ 0 ] = 0x10;
		remote_data[ 1 ] = 0x08;
		remote_data[ 2 ] = 0x00;
		remote_data[ 3 ] = 0x00;
		remote_data[ 4 ] = 0x00;
		remote_data[ 5 ] = 0x00;
		remote_data[ 6 ] = com_simu_remote_status_.teleco_self_id_6;
		remote_data[ 7 ] = com_simu_remote_status_.teleco_act_7;

		if( com_simu_remote_status_.teleco_act_7 < 10 )
		{
			remote_data[ 2 ] = ( 4 + 128 );
		}
		else if( com_simu_remote_status_.teleco_act_7 > 10 )
		{
			remote_data[ 2 ] = ( 16 + 32 );
		}

		com_simu_send_can_packet( ComSimuCanMessageId::CAN_ID_TELECO, ComSimuCanMessageType::CAN_TELECO_NUM_VERSION, remote_data, 8 );
	}
}

// #################################################
//
void Core::send_keypad_can_packet( )
{
	uint8_t keypad_data[ 1 ];
	uint8_t buttons = 0;

	if( com_simu_ihm_button_status_.cancel )
	{
		buttons = ( buttons | ( 0x01 << 0 ) );
	}

	if( com_simu_ihm_button_status_.validate )
	{
		buttons = ( buttons | ( 0x01 << 1 ) );
	}

	if( com_simu_ihm_button_status_.plus )
	{
		buttons = ( buttons | ( 0x01 << 2 ) );
	}

	if( com_simu_ihm_button_status_.minus )
	{
		buttons = ( buttons | ( 0x01 << 3 ) );
	}

	if( com_simu_ihm_button_status_.right )
	{
		buttons = ( buttons | ( 0x01 << 4 ) );
	}

	if( com_simu_ihm_button_status_.left )
	{
		buttons = ( buttons | ( 0x01 << 5 ) );
	}

	keypad_data[ 0 ] = buttons;

	com_simu_send_can_packet( ComSimuCanMessageId::CAN_ID_IHM, ComSimuCanMessageType::CAN_IHM_BUT, keypad_data, 1 );
}

// #################################################
//
int64_t Core::get_now_ms( )
{
	milliseconds now_ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
	int64_t now = static_cast<int64_t>( now_ms.count() );

	return  now;
}


// #################################################
//
void Core::com_simu_image_to_core_client_disconnected()
{
	close( com_simu_image_to_core_client_socket_ );

	com_simu_image_to_core_client_connected_ = false;
}

// #################################################
//
void Core::com_simu_image_to_core_read_thread_function( )
{
	unsigned char receivedBuffer[4096];

	while( true )
	{
		if( com_simu_image_to_core_client_connected_ )
		{
			com_simu_image_to_core_socket_access_.lock();

			int size = static_cast<int>( read( com_simu_image_to_core_client_socket_, receivedBuffer, 4096 ) );

			com_simu_image_to_core_socket_access_.unlock();

			if ( size > 0 )
			{
				com_simu_image_to_core_last_activity_ = get_now_ms();
			}
		}

		std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 100 ) ) );
	}
}

// #################################################
//
void Core::com_simu_image_to_core_write_thread_function( )
{
	std::cout << "core image listening on port 5558." << std::endl;

	com_simu_image_to_core_server_socket_ = openSocketServer( 5558 );

	while( true )
	{
		if( not com_simu_image_to_core_client_connected_ and com_simu_image_to_core_server_socket_ > 0 )
		{
			com_simu_image_to_core_socket_access_.lock();

			com_simu_image_to_core_client_socket_ = waitConnect( com_simu_image_to_core_server_socket_ );

			com_simu_image_to_core_socket_access_.unlock();
		}

		if( com_simu_image_to_core_client_socket_ > 0 and not com_simu_image_to_core_client_connected_ )
		{
			com_simu_image_to_core_client_connected_ = true;

			std::cout << "core image socket connected." << std::endl;

			com_simu_image_to_core_last_activity_ = get_now_ms();
		}

		if( com_simu_image_to_core_client_connected_ )
		{
			if ( ( get_now_ms() - com_simu_image_to_core_last_activity_ ) > 5000 )
			{
				std::cout << "core image client disconnected." << std::endl;

				com_simu_image_to_core_client_disconnected();
			}
		}

		if( com_simu_image_to_core_client_connected_ )
		{
			com_simu_image_to_core_buffer_access_.lock();

			if( com_simu_image_to_core_buffer_updated_ )
			{
				com_simu_image_to_core_buffer_updated_ = false;

				com_simu_image_to_core_socket_access_.lock();

				int total_written_bytes = 0;
				int sentSize = 0;
				int nb_tries = 0;
				int max_tries = 50;

				while( total_written_bytes < com_simu_image_to_core_buffer_->size() and nb_tries < max_tries )
				{
					sentSize = (int) send( com_simu_image_to_core_client_socket_, com_simu_image_to_core_buffer_->data() + total_written_bytes, com_simu_image_to_core_buffer_->size() - total_written_bytes, 0 );

					if( sentSize < 0 )
					{
						nb_tries++;
						std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 10 ) ) );
					}
					else
					{
						total_written_bytes = total_written_bytes + sentSize;
						nb_tries = 0;
					}
				}

				com_simu_image_to_core_socket_access_.unlock();
			}

			com_simu_image_to_core_buffer_access_.unlock();
		}

		//std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 1 ) ) );
	}
}


