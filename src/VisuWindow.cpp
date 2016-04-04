#include <algorithm>
#include <chrono>
#include <thread>

#include "VisuWindow.hpp"

VisuWindow::VisuWindow(std::string name, unsigned short width, unsigned short height) {
	SDL_Init(SDL_INIT_EVERYTHING);
	//	TTF_Init();
	screen_ = SDL_CreateWindow(name.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN);

	SDL_Renderer* renderer =  SDL_CreateRenderer( screen_, 0, SDL_RENDERER_ACCELERATED);

	// Set render color to black ( background will be rendered in this color )
	SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );

	SDL_RenderClear( renderer );
	SDL_RenderPresent( renderer );

	for ( int i = 0; i< SDL_NUM_SCANCODES; i++)
	{
		sdl_keys[i] = 0;
	}
}

VisuWindow::~VisuWindow() {
	SDL_Quit();
}

void VisuWindow::set_default_callback(CallbackFunc f) {
	defaultAction = f;
}

void VisuWindow::set_callback(CallbackFunc f, std::initializer_list<int> args) {
	std::vector<int> keys;
	for(int k: args)
		keys.push_back(k);
	sort(keys.begin(), keys.end());

	bool found = false;
	for(std::pair<std::vector<int>, CallbackFunc>& p : callbacks) {
		if(keys == p.first) {
			found = true;
			p.second = f;
			break;
		}
	}

	if(!found)
		callbacks.push_back(std::make_pair(keys, f));
}

void VisuWindow::go () {
	using namespace std::chrono;

	milliseconds ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
	int64_t now = static_cast<int64_t>( ms.count() );
	int64_t duration = 50;
	int64_t nextTick = now + duration;

	while ( true )
	{
		if( now >= nextTick )
		{
			nextTick = now + duration;

			SDL_Event event;
			while ( SDL_PollEvent( &event ) )
			{
				switch(event.type)
				{
					// Cas d'une touche enfoncée
					case SDL_KEYDOWN:
						sdl_keys[event.key.keysym.scancode] = 1;
						break;
						// Cas d'une touche relâchée
					case SDL_KEYUP:
						sdl_keys[event.key.keysym.scancode] = 0;
						break;
				}
			}

			if( sdl_keys[SDL_SCANCODE_ESCAPE] == 1)
			{
				return;
			}

			bool has_been_triggered = false;
			for(std::pair<std::vector<int>, CallbackFunc>& p : callbacks) {
				bool trigger = true;
				for(int key: p.first) {
					if(sdl_keys[key] != 1) {
						trigger = false;
						break;
					}
				}

				if(trigger)
				{
					has_been_triggered = true;
					p.second();
					break;
				}
			}

			if(!has_been_triggered)
				defaultAction();

			std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( 250 ) ) );
		}

		ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
		now = static_cast<int64_t>( ms.count() );

		std::this_thread::sleep_for( std::chrono::milliseconds( static_cast<int64_t>( duration / 2) ) );
	}
}

