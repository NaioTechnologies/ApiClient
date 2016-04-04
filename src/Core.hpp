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
#include <SDL2/SDL_video.h>
#include <SDL2/SDL_system.h>

#include "ApiCodec/Naio01Codec.hpp"
#include "ApiCodec/ApiMotorsPacket.hpp"

class Core
{
public:
	enum ControlType : uint8_t
	{
		CONTROL_TYPE_MANUAL = 0x01,
		CONTROL_TYPE_AUTO_1 = 0x02,
	};

public:

	Core( );
	~Core( );

	void init( std::string hostAdress_, uint16_t hostPort_ );

	void stop( );

private:

	void call_from_thread( );

	SDL_Window *initSDL(const char* name, int szX, int szY, SDL_Renderer** renderer);

	void exitSDL();

	void readSDLKeyboard();

	bool manageSDLKeyboard();

	bool sendWaitingPackets();

public:
	std::thread mainThread_;
private:
	bool stopThreadAsked_;
	bool threadStarted_;


	std::string hostAdress_;
	uint16_t hostPort_;

	int sdlKey_[SDL_NUM_SCANCODES];

	int socket_desc_;

	bool socketConnected_;

	Naio01Codec naioCodec_;

	std::vector< BaseNaio01PacketPtr > sendPacketList_;

	ControlType controlType_;

	ApiMotorsPacketPtr askedApiMotorsPacketPtr_;

};

#endif