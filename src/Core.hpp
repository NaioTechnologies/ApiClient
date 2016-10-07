//=============================================================================
//
//  Copyright (C)  2014  Naio Technologies
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//=============================================================================

#ifndef CORE_HPP
#define CORE_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <SDL2/SDL_video.h>
#include <SDL2/SDL_system.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <HaLidarPacket.hpp>
#include <ApiLidarPacket.hpp>
#include <HaOdoPacket.hpp>
#include <ApiPostPacket.hpp>
#include <ApiGpsPacket.hpp>
#include <HaGpsPacket.hpp>
#include <ApiStereoCameraPacket.hpp>

#include "ApiCodec/Naio01Codec.hpp"
#include "ApiCodec/ApiMotorsPacket.hpp"
#include "ApiCodec/ApiStatusPacket.hpp"
#include "ApiCodec/HaMotorsPacket.hpp"
#include "ApiCodec/HaGyroPacket.hpp"
#include "ApiCodec/HaAcceleroPacket.hpp"


class Core
{
public:
	enum ControlType : uint8_t
	{
		CONTROL_TYPE_MANUAL = 0x01,
	};

	const int64_t MAIN_GRAPHIC_DISPLAY_RATE_MS = 100;
public:

	Core( );
	~Core( );

	// launch core
	void init( std::string hostAdress_, uint16_t hostPort_ );

	// thread management
	void stop( );
	void stopServerReadThread( );
	void joinMainThread();
	void joinServerReadThread();

private:
	// thread function
	void graphic_thread( );

	// main server 5555 thread function
	void server_read_thread( );
	void server_write_thread( );
	void image_preparer_thread( );

	// images server 5557 thread function
	void image_server_thread( );
	void image_server_read_thread( );
	void image_server_write_thread( );

	// graph
	SDL_Window *initSDL(const char* name, int szX, int szY );

	void exitSDL();
	void readSDLKeyboard();
	bool manageSDLKeyboard();

	// communications
	void manageReceivedPacket( BaseNaio01PacketPtr packetPtr );
	void draw_robot();
	void draw_lidar( uint16_t lidar_distance_[271] );
	void draw_text( char gyro_buff[100], int x, int y );
	void draw_red_post( int x, int y );
	void draw_images( );
private:
	// thread part
	bool stopThreadAsked_;
	bool threadStarted_;
	std::thread graphicThread_;

	bool stopServerReadThreadAsked_;
	bool serverReadthreadStarted_;
	std::thread serverReadThread_;

	bool stopServerWriteThreadAsked_;
	bool serverWriteThreadStarted_;
	std::thread serverWriteThread_;

	// socket part
	std::string hostAdress_;
	uint16_t hostPort_;
	int socket_desc_;
	bool socketConnected_;

	// sdl part
	int sdlKey_[SDL_NUM_SCANCODES];

	// codec part
	Naio01Codec naioCodec_;
	std::mutex sendPacketListAccess_;
	std::vector< BaseNaio01PacketPtr > sendPacketList_;
	std::mutex motor_packet_access_;

	HaMotorsPacketPtr askedHaMotorsPacketPtr_;

	std::mutex ha_lidar_packet_ptr_access_;
	HaLidarPacketPtr ha_lidar_packet_ptr_;

	std::mutex ha_gyro_packet_ptr_access_;
	HaGyroPacketPtr ha_gyro_packet_ptr_;

	std::mutex ha_accel_packet_ptr_access_;
	HaAcceleroPacketPtr ha_accel_packet_ptr_;

	std::mutex ha_odo_packet_ptr_access;
	HaOdoPacketPtr ha_odo_packet_ptr_;

	std::mutex api_post_packet_ptr_access_;
	ApiPostPacketPtr api_post_packet_ptr_;

	std::mutex ha_gps_packet_ptr_access_;
	HaGpsPacketPtr ha_gps_packet_ptr_;

	std::mutex api_stereo_camera_packet_ptr_access_;
	ApiStereoCameraPacketPtr api_stereo_camera_packet_ptr_;
	std::mutex last_images_buffer_access_;
	uint8_t last_images_buffer_[ 4000000 ];
	ApiStereoCameraPacket::ImageType last_image_type_;

	// ia part
	ControlType controlType_;

	SDL_Window* screen_;
	SDL_Renderer* renderer_;

	SDL_Color sdl_color_red_;
	SDL_Color sdl_color_white_;
	TTF_Font* ttf_font_;

	bool asked_start_video_;
	bool asked_stop_video_;

	std::thread image_prepared_thread_;

	uint64_t last_motor_time_;

	int image_socket_desc_;
	bool imageSocketConnected_;
	Naio01Codec imageNaioCodec_;




	bool stopImageServerThreadAsked_;
	bool imageServerThreadStarted_;
	std::thread imageServerThread_;

	bool stopImageServerReadThreadAsked_;
	bool imageServerReadthreadStarted_;
	std::thread imageServerReadThread_;

	bool stopImageServerWriteThreadAsked_;
	bool imageServerWriteThreadStarted_;
	std::thread imageServerWriteThread_;
};

#endif