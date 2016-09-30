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
#include <HaLidarPacket.hpp>
#include <ApiLidarPacket.hpp>

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
		//CONTROL_TYPE_AUTO_1 = 0x02,
	};

public:

	Core( );
	~Core( );

	// launch core
	void init( std::string hostAdress_, uint16_t hostPort_ );

	// ask stop core.
	void stop( );

	// wait for core end.
	void joinMainThread();

private:

	// thread function
	void call_from_thread( );

	// graph function
	SDL_Window *initSDL(const char* name, int szX, int szY, SDL_Renderer** renderer);
//	void afficherTexte(SDL_Renderer *renderer, char texte[200], int x, int y, int monospace);

	void exitSDL();
	void readSDLKeyboard();
	bool manageSDLKeyboard();

	// communications
	bool sendWaitingPackets();
	void manageReceivedPacket( BaseNaio01PacketPtr packetPtr );
	void draw_robot();
	void draw_lidar( uint16_t lidar_distance_[271] );

public:

private:
	// thread part
	bool stopThreadAsked_;
	bool threadStarted_;
	std::thread mainThread_;

	// socket part
	std::string hostAdress_;
	uint16_t hostPort_;
	int socket_desc_;
	bool socketConnected_;

	// sdl part
	int sdlKey_[SDL_NUM_SCANCODES];

	// codec part
	Naio01Codec naioCodec_;
	std::vector< BaseNaio01PacketPtr > sendPacketList_;
	std::mutex motor_packet_access_;
	ApiMotorsPacketPtr askedApiMotorsPacketPtr_;
	HaMotorsPacketPtr askedHaMotorsPacketPtr_;

	std::mutex api_lidar_packet_ptr_access;
	ApiLidarPacketPtr api_lidar_packet_ptr_;

	std::mutex ha_lidar_packet_ptr_access;
	HaLidarPacketPtr ha_lidar_packet_ptr_;

	std::mutex ha_gyro_packet_ptr_access_;
	HaGyroPacketPtr ha_gyro_packet_ptr_;

	std::mutex ha_accel_packet_ptr_access;
	HaAcceleroPacketPtr ha_accel_packet_ptr_;

	// ia part
	ControlType controlType_;

	SDL_Window* screen_;
	SDL_Renderer* renderer_;
};

#endif