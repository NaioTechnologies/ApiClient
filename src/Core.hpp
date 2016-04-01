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

#include <ApiCodec/Naio01Codec.hpp>
#include <iostream>
#include <thread>

class Core
{
public:

	Core( );
	Core( std::string hostAdress_;
	uint16_t hostPort_; );

	~Core( );

	void init( );

	void stop( );

private:

	void call_from_thread( );

private:
	bool stopThreadAsked_;
	bool threadStarted_;
	std::thread mainThread_;

	std::string hostAdress_;
	uint16_t hostPort_;

//	std::vector< BaseNaio01PacketPtr > sendPacketList;
//	std::vector< BaseNaio01PacketPtr > receivedPacketList;

};

#endif