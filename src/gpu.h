#pragma once

#include "timer.h"
#include "mmu.h"

#include <string>
#include <map>
#include <vector>
#include "SDL.h"

struct gb_sprite
{
	uint32_t raw_data[0x80];
	uint32_t custom_data[0x80];
	uint8_t x;
	int y; //TODO: Find a better way to handle off-screen coordinates
	uint8_t tile_number;
	uint8_t options;
	std::string hash;
	bool custom_data_loaded;
};

struct gb_tile
{
	uint32_t raw_data[0x40];
	uint32_t custom_data[0x40];
	std::string hash;
	bool custom_data_loaded;
};

struct gbc_tile
{
	uint32_t raw_data[0x40];
};

class gpu : MMIO {
public:
	void Reset();

	uint8_t mmio_read(uint16_t addr) {
		__debugbreak();
		return 0;
	};
	void mmio_write(uint16_t addr, uint8_t data) {
		__debugbreak();
	};

	void step(int cpu_clock);

	uint8_t gpu_mode;
	uint8_t gpu_mode_change;
	int gpu_clock;

	int frame_start_time;
	int frame_current_time;

	bool lcd_enabled;

	//Tile set data
	gb_tile tile_set_1[0x100];
	gb_tile tile_set_0[0x100];

	gbc_tile gbc_tile_set_1[0x100][2];
	gbc_tile gbc_tile_set_0[0x100][2];

	gbc_tile win_tile;
	gbc_tile bg_tile;

	//Pixel data
	uint32_t scanline_pixel_data[0x100];
	uint32_t final_pixel_data[0x10000];

	//Palettes
	uint8_t bgp[4];
	uint8_t obp[4][2];

	uint16_t sprite_colors_raw[4][8];
	uint16_t background_colors_raw[4][8];

	uint32_t sprite_colors_final[4][8];
	uint32_t background_colors_final[4][8];

	gb_sprite sprites[40];

	//HDMA
	uint8_t current_hdma_line;

	//Sprite Hash Data
	std::vector<std::string> sprite_hash_list;
	std::vector<uint8_t> tile_set_0_updates;
	std::vector<uint8_t> tile_set_1_updates;
	std::map<std::string, SDL_Surface*> custom_sprite_list;
	std::map<std::string, SDL_Surface*>::iterator custom_sprite_list_itr;

	void render_screen();
	void scanline_compare();
	void update_bg_tile();
	void update_gbc_bg_tile();

	void generate_scanline();
	void generate_sprites();

	void horizontal_flip(uint16_t width, uint16_t height, uint32_t pixel_data[]);
	void vertical_flip(uint16_t width, uint16_t height, uint32_t pixel_data[]);
	uint8_t signed_tile(uint8_t tile_number);

	//Custom graphics functions and variables
	void dump_sprites();
	void dump_bg_tileset_1();
	void dump_bg_tileset_0();
	void dump_bg_window();

	void load_sprites();
	void load_bg_tileset_1();
	void load_bg_tileset_0();

	uint32_t dump_mode;

	uint32_t dump_tile_0;
	uint32_t dump_tile_1;
	uint32_t dump_tile_win;

	uint8_t last_bgp;

	void opengl_blit();

	uint32_t DMG_PAL_BG[4];
	uint32_t DMG_PAL_OBJ[4][2];
};


/* ROM Header */

//DMG or GBC Support
const uint16_t ROM_COLOR = 0x143;

//Memory Bank Controller Type
const uint16_t ROM_MBC = 0x147;

//ROM Size
const uint16_t ROM_ROMSIZE = 0x148;

//RAM Size
const uint16_t ROM_RAMSIZE = 0x149;

/* Special Memory Addresses */

//Object Attribute Memory
const uint16_t OAM = 0xFE00;

/* Special Memory Registers */

//Joypad
const uint16_t REG_P1 = 0xFF00;

//DIV Timer
const uint16_t REG_DIV = 0xFF04;

//TIMA Timer
const uint16_t REG_TIMA = 0xFF05;

//Timer Modulo
const uint16_t REG_TMA = 0xFF06;

//TAC
const uint16_t REG_TAC = 0xFF07;

//Interrupt Flag
const uint16_t REG_IF = 0xFF0F;

//Enabled Interrupts
const uint16_t REG_IE = 0xFFFF;

//NR52
const uint16_t REG_NR52 = 0xFF26;

//LCD Control
const uint16_t REG_LCDC = 0xFF40;

//LCD Status
const uint16_t REG_STAT = 0xFF41;

//Scroll-Y
const uint16_t REG_SY = 0xFF42;

//Scroll-X
const uint16_t REG_SX = 0xFF43;

//Scanline
const uint16_t REG_LY = 0xFF44;

//LY Coincidence
const uint16_t REG_LYC = 0xFF45;

//DMA
const uint16_t REG_DMA = 0xFF46;

//BG Palette
const uint16_t REG_BGP = 0xFF47;

//Sprite Palette 0
const uint16_t REG_OBP0 = 0xFF48;

//Sprite Palette 1
const uint16_t REG_OBP1 = 0xFF49;

//Window-Y
const uint16_t REG_WY = 0xFF4A;

//Window-X
const uint16_t REG_WX = 0xFF4B;

//Double Speed Control
const uint16_t REG_KEY1 = 0xFF4D;

//HDMA Source High
const uint16_t REG_HDMA1 = 0xFF51;

//HDMA Source Low
const uint16_t REG_HDMA2 = 0xFF52;

//HDMA Destination High
const uint16_t REG_HDMA3 = 0xFF53;

//HDMA Destination Low
const uint16_t REG_HDMA4 = 0xFF54;

//HDMA Control
const uint16_t REG_HDMA5 = 0xFF55;

//Video RAM Bank
const uint16_t REG_VBK = 0xFF4F;

//Background Color Palette Select
const uint16_t REG_BCPS = 0xFF68;

//Background Color Palette Data
const uint16_t REG_BCPD = 0xFF69;

//Object Color Palette Select
const uint16_t REG_OCPS = 0xFF6A;

//Object Color Palette Data
const uint16_t REG_OCPD = 0xFF6B;

//Working RAM Bank
const uint16_t REG_SVBK = 0xFF70;

extern gpu GPU;