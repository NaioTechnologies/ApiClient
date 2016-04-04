#ifndef VISU_WINDOW
#define VISU_WINDOW

#include <string>
#include <vector>
#include <functional>

#include <SDL2/SDL.h>

class VisuWindow {
public:
	typedef std::function<void ()> CallbackFunc;

	VisuWindow(std::string name, unsigned short width, unsigned short height);
  virtual ~VisuWindow();

	void set_default_callback(CallbackFunc f);
	void set_callback(CallbackFunc f, std::initializer_list<int> args);
	void go();

private:
	SDL_Window* screen_;
	SDL_Renderer* renderer_;
	int sdl_keys[SDL_NUM_SCANCODES];
	std::vector<std::pair<std::vector<int>, CallbackFunc>> callbacks;
	CallbackFunc defaultAction;
};

#endif
