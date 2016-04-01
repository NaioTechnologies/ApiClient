#include "Core.hpp"

// #################################################
Core::Core( ) :
		stopThreadAsked_{ false },
		threadStarted_{ false },
		hostAdress_{ "127.0.0.1" },
		hostPort_{ 5555 }
{

}

// #################################################
Core::~Core( )
{

}

// #################################################
void
Core::init( )
{
	stopThreadAsked_ = false;
	threadStarted_ = false;

	mainThread_ = std::thread{ &Core::call_from_thread, this };
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
	threadStarted_ = true;

	while( !stopThreadAsked_ )
	{

	}

	threadStarted_ = false;
	stopThreadAsked_ = false;
}