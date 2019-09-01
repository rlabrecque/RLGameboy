#include "main.h"
#include "cartridge.h"
#include "cpu.h"
#include "gpu.h"
#include "key.h"
#include "mmu.h"
#include "timer.h"
#include "util.h"


#include "SDL.h"

#include <stdint.h>
#include <conio.h>
#include <stdio.h>
#include <iostream>
#include <Windows.h>

//#define HWTESTS

static SDL_Window *sdlWindow;
static SDL_Renderer *sdlRenderer;
static SDL_Texture *sdlTexture;
static int run_interval;

#if defined(HWTESTS)
#include <dirent.h>
void RunTests() {
	DIR *dir;
	struct dirent *ent;
	SDL_Event e;
	bool bEscapeHit = false;
	bool bSpaceHit = false;
	if ((dir = opendir("D:/Code/RLGameboy/hwtests/")) != NULL) {
		char filepath[MAX_PATH];
		while ((ent = readdir(dir)) != NULL) {
			if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) {
				continue;
			}
			bSpaceHit = false;
			strlcpy(filepath, "D:/Code/RLGameboy/hwtests/", MAX_PATH);
			strlcat(filepath, ent->d_name, MAX_PATH);

			printf("%s\n", filepath);
			reset();
			GetCartridge()->LoadFile(filepath);
			run();

			while (true) {
				if (SDL_PollEvent(&e)) {
					if (e.type == SDL_QUIT) {
						bEscapeHit = true;
					}
					else if (e.type == SDL_KEYDOWN) {
						switch (e.key.keysym.sym) {
						case SDLK_ESCAPE:
							bEscapeHit = true;
							break;
						case SDLK_SPACE:
							bSpaceHit = true;
							break;
						}
					}
				}

				if (bEscapeHit || bSpaceHit) {
					break;
				}

				if (run_interval) {
					frame();
				}
				Sleep(run_interval);
			}

			if (bEscapeHit) {
				break;
			}
		}
		closedir(dir);
	}
	else {
		// could not open directory 
	}
}
#endif

void frame() {

	//uint64_t fclock = Z80.cycleCounter_ + 70224;

	//uint32_t cycles = Z80.cycleCounter_;

	Z80.status.clock = 0;

	Z80.interrupt_test();
	Z80.Process();

	//cycles = Z80.cycleCounter_ - cycles;


	GPU.step(Z80.status.clock);
}

void reset() {
	//printf("[MAIN] Reset.\n");

	MMU.Reset();
	Z80.Reset();
	GPU.Reset();
	KEY.reset();
	//TIMER.reset();
}

int main(int argc, char *argv[]) {
	if (SDL_Init(SDL_INIT_EVERYTHING) == -1)
		return 1;

	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetSwapInterval(1);
	SDL_ShowCursor(SDL_DISABLE);

	sdlWindow = SDL_CreateWindow("RLGameboy", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 160, 144, 0);
	if (sdlWindow == NULL)
		return 1;

	sdlRenderer = SDL_CreateRenderer(sdlWindow, -1, SDL_RENDERER_SOFTWARE);
	if (sdlRenderer == NULL)
		return 1;

	sdlTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, 160, 144);
	if (sdlTexture == NULL)
		return 1;

#if defined(HWTESTS)
	RunTests();

	SDL_DestroyTexture(sdlTexture);
	SDL_DestroyRenderer(sdlRenderer);
	SDL_DestroyWindow(sdlWindow);
	return 0;
#endif

	reset();

	//MMU.load("D:\\Riley\\Desktop\\sram.gbc");
	//MMU.load("D:\\Riley\\Desktop\\sml.gb");
	//GetCartridge()->LoadFile("D:/Riley/Desktop/cpu_instrs.gb");
	//GetCartridge()->LoadFile("D:/Riley/Desktop/pkmn.gb");
	//GetCartridge()->LoadFile("D:/Riley/Desktop/yellow.gbc");
	//GetCartridge()->LoadFile("D:/Riley/Desktop/tetris.gb");
	GetCartridge()->LoadFile("D:/Riley/Desktop/ttt.gb");

	SDL_Event e;
	bool bEscapeHit = false;
	while (true) {
		if (SDL_PollEvent(&e)) {
			if (e.type == SDL_QUIT) {
				bEscapeHit = true;
			}
			else if (e.type == SDL_KEYDOWN) {
				switch (e.key.keysym.sym) {
				case SDLK_ESCAPE: // Escape
					bEscapeHit = true;
					break;

				case SDLK_UP:
					KEY._keys[1] &= 0xB;
					break;
				case SDLK_DOWN:
					KEY._keys[1] &= 0x7;
					break;
				case SDLK_LEFT:
					KEY._keys[1] &= 0xD;
					break;
				case SDLK_RIGHT:
					KEY._keys[1] &= 0xE;
					break;
				case SDLK_z:
					KEY._keys[0] &= 0xD;
					break;
				case SDLK_x:
					KEY._keys[0] &= 0xE;
					break;
				case SDLK_RETURN:
					KEY._keys[0] &= 0x7;
					break;
				case SDLK_RSHIFT:
					KEY._keys[0] &= 0xB;
					break;
				}
			}
			else if (e.type == SDL_KEYUP) {
				switch (e.key.keysym.sym) {
				case SDLK_UP:
					KEY._keys[1] |= 0x4;
					break;
				case SDLK_DOWN:
					KEY._keys[1] |= 0x8;
					break;
				case SDLK_LEFT:
					KEY._keys[1] |= 0x2;
					break;
				case SDLK_RIGHT:
					KEY._keys[1] |= 0x1;
					break;
				case SDLK_z:
					KEY._keys[0] |= 0x2;
					break;
				case SDLK_x:
					KEY._keys[0] |= 0x1;
					break;
				case SDLK_RETURN:
					KEY._keys[0] |= 0x8;
					break;
				case SDLK_RSHIFT:
					KEY._keys[0] |= 0x4;
					break;
				}
			}
		}

		if (_kbhit()) {
			int key = _getch();

			switch (key) {
			case 27: // Escape
				bEscapeHit = true;
				break;
			case 'r':
				reset();
				GetCartridge()->LoadFile("D:/Riley/Desktop/ttt.gb");
			default:
				//printf("%c", key);
				break;
			}
		}

		if (bEscapeHit) {
			break;
		}

		frame();
		//Sleep(1);
	}

	SDL_DestroyTexture(sdlTexture);
	SDL_DestroyRenderer(sdlRenderer);
	SDL_DestroyWindow(sdlWindow);

	SDL_Quit();

	return 0;
}

#include <time.h>
void videoRefresh(uint32_t* screen) {
	SDL_UpdateTexture(sdlTexture, NULL, screen, 160 * sizeof(uint32_t));
	SDL_RenderClear(sdlRenderer);
	SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, NULL);
	SDL_RenderPresent(sdlRenderer);
	//printf("videoRefresh\n");
	/*static unsigned frameCounter = 0;
	static time_t previous, current;
	frameCounter++;

	time(&current);
	if (current != previous) {
		previous = current;
		SDL_SetWindowTitle(sdlWindow, "FPS:"); //frameCounter
		frameCounter = 0;
	}*/
}