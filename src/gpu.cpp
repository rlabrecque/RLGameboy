#include "main.h"
#include "cpu.h"
#include "gpu.h"
#include "mmu.h"

#include <algorithm>
#include <stdio.h>

gpu GPU;

void gpu::Reset() {
	for (unsigned n = 0x8000; n <= 0x9fff; n++) MMU.mmio[n] = this;  //VRAM
	for (unsigned n = 0xfe00; n <= 0xfe9f; n++) MMU.mmio[n] = this;  //OAM

	MMU.mmio[0xff40] = this;  //LCDC
	MMU.mmio[0xff41] = this;  //STAT
	MMU.mmio[0xff42] = this;  //SCY
	MMU.mmio[0xff43] = this;  //SCX
	MMU.mmio[0xff44] = this;  //LY
	MMU.mmio[0xff45] = this;  //LYC
	MMU.mmio[0xff47] = this;  //BGP
	MMU.mmio[0xff48] = this;  //OBP0
	MMU.mmio[0xff49] = this;  //OBP1
	MMU.mmio[0xff4a] = this;  //WY
	MMU.mmio[0xff4b] = this;  //WX

	gpu_mode = 2;
	gpu_mode_change = 0;
	gpu_clock = 0;
	frame_start_time = 0;
	frame_current_time = 0;
	lcd_enabled = false;

	/*if (config::use_scaling)
	{
		temp_screen = SDL_CreateRGBSurface(SDL_SWSURFACE, (256 * config::scaling_factor), (144 * config::scaling_factor), 32, 0, 0, 0, 0);
	}

	src_screen = SDL_CreateRGBSurface(SDL_SWSURFACE, 256, 144, 32, 0, 0, 0, 0);*/

	//Initialize a bunch of data to 0 - Let's avoid segfaults...
	memset(scanline_pixel_data, 0xFFFFFFFF, sizeof(scanline_pixel_data));
	memset(final_pixel_data, 0xFFFFFFFF, sizeof(final_pixel_data));

	sprite_hash_list.push_back(" ");

	for (int x = 0; x < 40; x++)
	{
		memset(sprites[x].raw_data, 0, sizeof(sprites[x].raw_data));
		memset(sprites[x].custom_data, 0, sizeof(sprites[x].custom_data));
		sprites[x].custom_data_loaded = false;
	}

	for (int x = 0; x < 0x100; x++)
	{
		memset(tile_set_1[x].raw_data, 0, sizeof(tile_set_1[x].raw_data));
		memset(tile_set_1[x].raw_data, 0, sizeof(tile_set_1[x].custom_data));
		memset(tile_set_0[x].raw_data, 0, sizeof(tile_set_0[x].raw_data));
		memset(tile_set_0[x].raw_data, 0, sizeof(tile_set_0[x].custom_data));
		memset(gbc_tile_set_1[x][0].raw_data, 0, sizeof(gbc_tile_set_1[x][0].raw_data));
		memset(gbc_tile_set_1[x][1].raw_data, 0, sizeof(gbc_tile_set_1[x][1].raw_data));
		memset(gbc_tile_set_0[x][0].raw_data, 0, sizeof(gbc_tile_set_1[x][0].raw_data));
		memset(gbc_tile_set_0[x][1].raw_data, 0, sizeof(gbc_tile_set_1[x][1].raw_data));
		tile_set_1[x].custom_data_loaded = false;
		tile_set_0[x].custom_data_loaded = false;
	}

	dump_tile_0 = 0xFEEDBACC;
	dump_tile_1 = 0xFEEDBACC;
	dump_tile_win = 0xFEEDBACC;
	dump_mode = 4;

	/*if (config::use_opengl)
	{
		glGenTextures(1, &gpu_texture);
		glBindTexture(GL_TEXTURE_2D, gpu_texture);
	}*/


	DMG_PAL_BG[0] = DMG_PAL_OBJ[0][0] = DMG_PAL_OBJ[0][1] = 0xFFFFFFFF;
	DMG_PAL_BG[1] = DMG_PAL_OBJ[1][0] = DMG_PAL_OBJ[1][1] = 0xFFC0C0C0;
	DMG_PAL_BG[2] = DMG_PAL_OBJ[2][0] = DMG_PAL_OBJ[2][1] = 0xFF606060;
	DMG_PAL_BG[3] = DMG_PAL_OBJ[3][0] = DMG_PAL_OBJ[3][1] = 0xFF000000;
}
/****** Flip pixel data horizontally - For sprites only ******/
void gpu::horizontal_flip(uint16_t width, uint16_t height, uint32_t pixel_data[])
{
	uint32_t target, origin = 0;

	for (uint16_t x = 0; x < height; x++)
	{
		for (uint16_t y = 0; y < width / 2; y++)
		{
			target = (((x *width) + width) - y) - 1;
			origin = (x * width) + y;
			std::swap(pixel_data[target], pixel_data[origin]);
		}
	}
}

/****** Flip pixel data vertically - For sprites only ******/
void gpu::vertical_flip(uint16_t width, uint16_t height, uint32_t pixel_data[])
{
	uint32_t target, origin = 0;

	for (uint16_t x = 0; x < height / 2; x++)
	{
		for (uint16_t y = 0; y < width; y++)
		{
			target = ((height - 1 - x) * width) + y;
			origin = (x * width) + y;
			std::swap(pixel_data[target], pixel_data[origin]);
		}
	}
}

/****** Converts signed tile numbers to regular tile numbers (0-255) ******/
uint8_t gpu::signed_tile(uint8_t tile_number)
{
	if (tile_number <= 127)
	{
		tile_number += 128;
		return tile_number;
	}

	else
	{
		tile_number -= 128;
		return tile_number;
	}
}

/****** Compares LY and LYC - Generates STAT Interrupt ******/
void gpu::scanline_compare()
{
	if (MMU.memory_map[REG_LY] == MMU.memory_map[REG_LYC])
	{
		MMU.memory_map[REG_STAT] |= 0x4;
		if (MMU.memory_map[REG_STAT] & 0x40) { MMU.memory_map[REG_IF] |= 2; }
	}
	else { MMU.memory_map[REG_STAT] &= ~0x4; }
}

/****** Updates a specific tile - DMG Mode ******/
void gpu::update_bg_tile()
{
	uint8_t high_byte, low_byte = 0;
	uint8_t high_bit, low_bit = 0;
	uint8_t final_byte = 0;

	uint16_t tile_addr = 0;
	uint8_t tile_number = 0;
	uint8_t tile_row = 0;

	uint16_t pixel_counter = 0;

	//Update tiles for every address that was written to since last update
	for (int x = 0; x < MMU.gpu_update_addr.size(); x++)
	{
		//Update a tile row for Tile Set #1
		if ((MMU.gpu_update_addr[x] >= 0x8000) && (MMU.gpu_update_addr[x] <= 0x8FFF))
		{
			tile_number = (MMU.gpu_update_addr[x] - 0x8000) / 16;
			tile_addr = 0x8000 + (tile_number * 16);
			tile_row = (MMU.gpu_update_addr[x] - tile_addr) / 2;
			tile_addr += (tile_row * 2);
			pixel_counter = (tile_row * 8);

			//Grab High and Low Bytes for Background Tile
			high_byte = MMU.rb(tile_addr);
			low_byte = MMU.rb(tile_addr + 1);

			//Cycle through High and Low bytes
			for (int y = 7; y >= 0; y--)
			{
				high_bit = (high_byte >> y) & 0x01;
				low_bit = (low_byte >> y) & 0x01;
				final_byte = high_bit + (low_bit * 2);

				tile_set_1[tile_number].raw_data[pixel_counter] = final_byte;
				pixel_counter++;
			}

			//Add tile to the list of updated tiles to see if custom tiles can be loaded
			bool add_tile = true;

			for (int y = 0; y < tile_set_1_updates.size(); y++)
			{
				if (tile_set_1_updates[y] == tile_number) { add_tile = false; }
			}

			if (add_tile) { tile_set_1_updates.push_back(tile_number); }
		}

		//Update a tile row for Tile Set #1
		if ((MMU.gpu_update_addr[x] >= 0x8800) && (MMU.gpu_update_addr[x] <= 0x97FF))
		{
			tile_number = (MMU.gpu_update_addr[x] - 0x8800) / 16;
			tile_addr = 0x8800 + (tile_number * 16);
			tile_row = (MMU.gpu_update_addr[x] - tile_addr) / 2;
			tile_addr += (tile_row * 2);
			pixel_counter = (tile_row * 8);

			//Grab High and Low Bytes for Background Tile
			high_byte = MMU.rb(tile_addr);
			low_byte = MMU.rb(tile_addr + 1);

			//Cycle through High and Low bytes
			for (int y = 7; y >= 0; y--)
			{
				high_bit = (high_byte >> y) & 0x01;
				low_bit = (low_byte >> y) & 0x01;
				final_byte = high_bit + (low_bit * 2);

				tile_set_0[tile_number].raw_data[pixel_counter] = final_byte;
				pixel_counter++;
			}

			//Add tile to the list of updated tiles to see if custom tiles can be loaded
			bool add_tile = true;

			for (int y = 0; y < tile_set_0_updates.size(); y++)
			{
				if (tile_set_0_updates[y] == tile_number) { add_tile = false; }
			}

			if (add_tile) { tile_set_0_updates.push_back(tile_number); }
		}

		//When a new background palette is used, update the tile checklist to include all tiles
		//Don't repeat once a new background palette has been established!
		if ((MMU.gpu_update_addr[x] == REG_BGP) && (last_bgp != MMU.memory_map[REG_BGP]))
		{
			tile_set_0_updates.clear();
			tile_set_1_updates.clear();
			for (int y = 0; y < 0x100; y++)
			{
				tile_set_0_updates.push_back(y);
				tile_set_1_updates.push_back(y);
			}

			last_bgp = MMU.memory_map[REG_BGP];
		}
	}

	MMU.gpu_update_addr.clear();
}

/****** Updates a specific tile - GBC Mode ******/
void gpu::update_gbc_bg_tile()
{
	uint8_t high_byte, low_byte = 0;
	uint8_t high_bit, low_bit = 0;
	uint8_t final_byte = 0;

	uint16_t tile_addr = 0;
	uint8_t tile_number = 0;
	uint8_t tile_row = 0;

	uint16_t pixel_counter = 0;

	//Update tiles for every address that was written to since last update
	for (int x = 0; x < MMU.gpu_update_addr.size(); x++)
	{
		//Update a tile row for Tile Set #1
		if ((MMU.gpu_update_addr[x] >= 0x8000) && (MMU.gpu_update_addr[x] <= 0x8FFF))
		{
			tile_number = (MMU.gpu_update_addr[x] - 0x8000) / 16;
			tile_addr = 0x8000 + (tile_number * 16);
			tile_row = (MMU.gpu_update_addr[x] - tile_addr) / 2;
			tile_addr += (tile_row * 2);
			pixel_counter = (tile_row * 8);

			//Grab High and Low Bytes for Background Tile
			high_byte = MMU.rb(tile_addr);
			low_byte = MMU.rb(tile_addr + 1);

			//Cycle through High and Low bytes
			for (int y = 7; y >= 0; y--)
			{
				high_bit = (high_byte >> y) & 0x01;
				low_bit = (low_byte >> y) & 0x01;
				final_byte = high_bit + (low_bit * 2);

				gbc_tile_set_1[tile_number][MMU.vram_bank].raw_data[pixel_counter] = final_byte;
				pixel_counter++;
			}
		}

		//Update a tile row for Tile Set #1
		if ((MMU.gpu_update_addr[x] >= 0x8800) && (MMU.gpu_update_addr[x] <= 0x97FF))
		{
			tile_number = (MMU.gpu_update_addr[x] - 0x8800) / 16;
			tile_addr = 0x8800 + (tile_number * 16);
			tile_row = (MMU.gpu_update_addr[x] - tile_addr) / 2;
			tile_addr += (tile_row * 2);
			pixel_counter = (tile_row * 8);

			//Grab High and Low Bytes for Background Tile
			high_byte = MMU.rb(tile_addr);
			low_byte = MMU.rb(tile_addr + 1);

			//Cycle through High and Low bytes
			for (int y = 7; y >= 0; y--)
			{
				high_bit = (high_byte >> y) & 0x01;
				low_bit = (low_byte >> y) & 0x01;
				final_byte = high_bit + (low_bit * 2);

				gbc_tile_set_0[tile_number][MMU.vram_bank].raw_data[pixel_counter] = final_byte;
				pixel_counter++;
			}
		}
	}

	MMU.gpu_update_addr.clear();
}

/****** Prepares scanline for rendering - Pulls data from BG, Window, and Sprites ******/
void gpu::generate_scanline()
{
	uint8_t current_scanline = MMU.memory_map[REG_LY] + MMU.memory_map[REG_SY];
	uint8_t current_bgp = MMU.memory_map[REG_BGP];
	uint8_t current_pixel = 0x100 - MMU.memory_map[REG_SX];

	uint32_t bg_win_raw_data[0x100];
	uint32_t bg_priority[0x100];

	uint16_t map_addr = 0;
	uint16_t tile_set_addr = 0;
	uint8_t map_entry = 0;
	uint8_t tile_pixel = 0;

	//Determine Tile Map Address
	if (MMU.memory_map[REG_LCDC] & 0x08) { map_addr = 0x9C00; }
	else { map_addr = 0x9800; }

	//Determine Background/Window Palette - From lightest to darkest
	bgp[0] = current_bgp & 0x3;
	bgp[1] = (current_bgp >> 2) & 0x3;
	bgp[2] = (current_bgp >> 4) & 0x3;
	bgp[3] = (current_bgp >> 6) & 0x3;

	//Determine which tiles we should generate to get the scanline data - integer division ftw :p
	uint16_t tile_lower_range = (current_scanline / 8) * 32;
	uint16_t tile_upper_range = tile_lower_range + 32;

	//Render Background Pixel Data
	if (MMU.memory_map[REG_LCDC] & 0x01)
	{
		//Determine which line of the tiles we should generate pixels for this scanline
		uint8_t tile_line = current_scanline % 8;

		//Generate background pixel data for selected tiles
		for (int x = tile_lower_range; x < tile_upper_range; x++)
		{
			bool highlight_tile = false;
			uint8_t bg_palette = 0;
			uint8_t bg_tile_bank = 0;
			uint8_t bg_map_attribute = 0;

			//Check if tile can be highlighted - For BG tile dumping
			/*if (config::dump_sprites)
			{
				uint32_t line_bound = MMU.memory_map[REG_LY];
				uint32_t left_bound = current_pixel;
				uint32_t right_bound = current_pixel + 8;

				if (((config::mouse_x / config::scaling_factor) > left_bound) && ((config::mouse_x / config::scaling_factor) < right_bound)
					&& ((config::mouse_y / config::scaling_factor) == line_bound)) {
					highlight_tile = true;
				}
			}*/

			for (int y = (tile_line * 8); y < ((tile_line * 8) + 8); y++)
			{
				map_entry = MMU.rb(map_addr + x);

				//Choose from the correct Tile Set
				if (MMU.memory_map[REG_LCDC] & 0x10)
				{
					tile_pixel = tile_set_1[map_entry].raw_data[y];
					if (highlight_tile) { dump_tile_1 = map_entry; dump_mode = 1; }
				}
				else
				{
					map_entry = signed_tile(map_entry);
					tile_pixel = tile_set_0[map_entry].raw_data[y];
					if (highlight_tile) { dump_tile_0 = map_entry; dump_mode = 0; }
				}

				bg_win_raw_data[current_pixel] = tile_pixel;

				/*if ((config::load_sprites) && (MMU.memory_map[REG_LCDC] & 0x10) && (tile_set_1[map_entry].custom_data_loaded))
				{
					scanline_pixel_data[current_pixel] = tile_set_1[map_entry].custom_data[y];
				}

				else if ((config::load_sprites) && ((MMU.memory_map[REG_LCDC] & 0x10) == 0) && (tile_set_0[map_entry].custom_data_loaded))
				{
					scanline_pixel_data[current_pixel] = tile_set_0[map_entry].custom_data[y];
				}

				else*/
				{
					//Output Scanline data to RGBA - DMG Mode
					if (/*config::gb_type != 2*/true)
					{
						//Output Scanline data to RGBA
						switch (bgp[tile_pixel])
						{
						case 0:
							scanline_pixel_data[current_pixel] = DMG_PAL_BG[0];
							break;

						case 1:
							scanline_pixel_data[current_pixel] = DMG_PAL_BG[1];
							break;

						case 2:
							scanline_pixel_data[current_pixel] = DMG_PAL_BG[2];
							break;

						case 3:
							scanline_pixel_data[current_pixel] = DMG_PAL_BG[3];
							break;
						}
					}

					//Output Scanline data to RGBA - GBC Mode
					else
					{
						if ((y % 8) == 0)
						{
							uint8_t old_vram_bank = MMU.vram_bank;

							//Always read CHR data from Bank 0
							MMU.vram_bank = 0;
							map_entry = MMU.rb(map_addr + x);

							//Read BG Map attributes from Bank 1
							MMU.vram_bank = 1;
							bg_map_attribute = MMU.rb(map_addr + x);
							bg_palette = bg_map_attribute & 0x7;
							bg_tile_bank = (bg_map_attribute & 0x8) ? 1 : 0;

							if (MMU.memory_map[REG_LCDC] & 0x10) { bg_tile = gbc_tile_set_1[map_entry][bg_tile_bank]; }
							else { map_entry = signed_tile(map_entry); bg_tile = gbc_tile_set_0[map_entry][bg_tile_bank]; }

							if (bg_map_attribute & 0x20) { horizontal_flip(8, 8, bg_tile.raw_data); }
							if (bg_map_attribute & 0x40) { vertical_flip(8, 8, bg_tile.raw_data); }

							MMU.vram_bank = old_vram_bank;
						}

						bg_priority[current_pixel] = (bg_map_attribute & 0x80) ? 1 : 0;
						bg_win_raw_data[current_pixel] = bg_tile.raw_data[y];
						scanline_pixel_data[current_pixel] = background_colors_final[bg_tile.raw_data[y]][bg_palette];
					}
				}

				//Highlight tiles on mouseover - For BG tile dumping
				/*if (config::dump_sprites)
				{
					if ((MMU.memory_map[REG_LCDC] & 0x10) && (map_entry == dump_tile_1)) { scanline_pixel_data[current_pixel] += 0x00700000; }
					else if (((MMU.memory_map[REG_LCDC] & 0x10) == 0) && (map_entry == dump_tile_0)) { scanline_pixel_data[current_pixel] += 0x00700000; }
				}*/

				current_pixel++;
			}
		}
	}

	//Render Window Pixel Data
	if ((MMU.memory_map[REG_LY] - MMU.memory_map[REG_WY] >= 0) && (MMU.memory_map[REG_LCDC] & 0x20))
	{
		//Determine Tile Map Address
		if (MMU.memory_map[REG_LCDC] & 0x40) { map_addr = 0x9C00; }
		else { map_addr = 0x9800; }

		current_pixel = MMU.memory_map[REG_WX] - 7;
		uint8_t window_line = (MMU.memory_map[REG_LY] - MMU.memory_map[REG_WY]) % 8;

		//Determine which tiles we should generate to get the scanline data
		tile_lower_range = ((MMU.memory_map[REG_LY] - MMU.memory_map[REG_WY]) / 8) * 32;
		tile_upper_range = tile_lower_range + 32;

		//Generate window pixel data for selected tiles
		for (int x = tile_lower_range; x < tile_upper_range; x++)
		{
			bool highlight_tile = false;
			uint8_t bg_tile_bank = 0;
			uint8_t bg_palette = 0;
			uint8_t bg_map_attribute = 0;

			//Check if tile can be highlighted - For BG tile dumping
			/*if (config::dump_sprites)
			{
				uint32_t line_bound = MMU.memory_map[REG_LY];
				uint32_t left_bound = current_pixel;
				uint32_t right_bound = current_pixel + 8;

				if (((config::mouse_x / config::scaling_factor) > left_bound) && ((config::mouse_x / config::scaling_factor) < right_bound)
					&& ((config::mouse_y / config::scaling_factor) == line_bound)) {
					highlight_tile = true;
				}
			}*/

			for (int y = (window_line * 8); y < ((window_line * 8) + 8); y++)
			{
				map_entry = MMU.rb(map_addr + x);

				//Choose from the correct Tile Set
				if (MMU.memory_map[REG_LCDC] & 0x10)
				{
					tile_pixel = tile_set_1[map_entry].raw_data[y];
					if (highlight_tile) { dump_tile_win = map_entry; dump_mode = 2; }
				}
				else
				{
					map_entry = signed_tile(map_entry);
					tile_pixel = tile_set_0[map_entry].raw_data[y];
					if (highlight_tile) { dump_tile_win = map_entry; dump_mode = 3; }
				}

				bg_win_raw_data[current_pixel] = tile_pixel;

				/*if ((config::load_sprites) && (MMU.memory_map[REG_LCDC] & 0x10) && (tile_set_1[map_entry].custom_data_loaded))
				{
					scanline_pixel_data[current_pixel] = tile_set_1[map_entry].custom_data[y];
				}

				else if ((config::load_sprites) && ((MMU.memory_map[REG_LCDC] & 0x10) == 0) && (tile_set_0[map_entry].custom_data_loaded))
				{
					scanline_pixel_data[current_pixel] = tile_set_0[map_entry].custom_data[y];
				}

				else*/
				{
					//Output Scanline data to RGBA - DMG Mode
					if (/*config::gb_type != 2*/true)
					{
						//Output Scanline data to RGBA
						switch (bgp[tile_pixel])
						{
						case 0:
							scanline_pixel_data[current_pixel] = DMG_PAL_BG[0];
							break;

						case 1:
							scanline_pixel_data[current_pixel] = DMG_PAL_BG[1];
							break;

						case 2:
							scanline_pixel_data[current_pixel] = DMG_PAL_BG[2];
							break;

						case 3:
							scanline_pixel_data[current_pixel] = DMG_PAL_BG[3];
							break;
						}
					}

					//Output Scanline data to RGBA - GBC Mode
					else
					{
						if ((y % 8) == 0)
						{
							uint8_t old_vram_bank = MMU.vram_bank;

							//Always read CHR data from Bank 0
							MMU.vram_bank = 0;
							map_entry = MMU.rb(map_addr + x);

							//Read BG Map attributes from Bank 1
							MMU.vram_bank = 1;
							bg_map_attribute = MMU.rb(map_addr + x);
							bg_palette = bg_map_attribute & 0x7;
							bg_tile_bank = (bg_map_attribute & 0x8) ? 1 : 0;

							if (MMU.memory_map[REG_LCDC] & 0x10) { win_tile = gbc_tile_set_1[map_entry][bg_tile_bank]; }
							else { map_entry = signed_tile(map_entry); win_tile = gbc_tile_set_0[map_entry][bg_tile_bank]; }

							if (bg_map_attribute & 0x20) { horizontal_flip(8, 8, win_tile.raw_data); }
							if (bg_map_attribute & 0x40) { vertical_flip(8, 8, win_tile.raw_data); }

							MMU.vram_bank = old_vram_bank;
						}

						bg_priority[current_pixel] = (bg_map_attribute & 0x80) ? 1 : 0;
						bg_win_raw_data[current_pixel] = win_tile.raw_data[y];
						scanline_pixel_data[current_pixel] = background_colors_final[win_tile.raw_data[y]][bg_palette];
					}
				}

				/*if ((config::dump_sprites) && (map_entry == dump_tile_win)) { scanline_pixel_data[current_pixel] += 0x00700000; }*/

				current_pixel++;
				if (current_pixel == 0) { x = tile_upper_range; break; }
			}
		}
	}

	//Render Sprite Pixel Data
	if (MMU.memory_map[REG_LCDC] & 0x02)
	{
		//Simple list to keep track of which sprites (10 max) to render for scanline
		uint8_t sprite_render_list[10];
		uint8_t sprite_render_line[10];
		uint16_t sprite_height = 0;
		int sprite_counter = 0;

		current_pixel = 0;

		//Determine if in 8x8 or 8x16 mode
		if (MMU.memory_map[REG_LCDC] & 0x04) { sprite_height = 16; }
		else { sprite_height = 8; }

		//Cycle through all sprites to see if they're within height-range
		for (int x = 0; x < 40; x++)
		{
			//Check to see if X coordinate is on screen
			if (sprite_counter < 10)
			{
				//Check to see if Y coordinate is within rendering range
				for (int y = 0; y < sprite_height; y++)
				{
					if ((sprites[x].y + y) == MMU.memory_map[REG_LY])
					{
						sprite_render_list[sprite_counter] = x;
						sprite_render_line[sprite_counter] = y;
						sprite_counter++;
					}
				}
			}
		}

		if (sprite_counter != 0)
		{
			//Cycle through sprite list
			for (int x = (sprite_counter - 1); x >= 0; x--)
			{
				uint8_t current_sprite = sprite_render_list[x];
				uint8_t sprite_line = sprite_render_line[x];
				uint8_t priority = sprites[current_sprite].options & 0x80 ? 1 : 0;
				uint8_t pal = sprites[current_sprite].options & 0x10 ? 1 : 0;
				uint8_t gbc_pal = sprites[current_sprite].options & 0x7;

				current_pixel = sprites[current_sprite].x;

				//Draw each sprite
				for (int y = (sprite_line * 8); y < (sprite_line * 8) + 8; y++)
				{
					bool draw_sprite_pixel = false;

					//Draw custom sprite data
					/*if (sprites[current_sprite].custom_data_loaded)
					{
						//Only draw if pixel color is not equal to the transparency value
						if (sprites[current_sprite].custom_data[y] != config::custom_sprite_transparency)
						{
							scanline_pixel_data[current_pixel] = sprites[current_sprite].custom_data[y];
						}
					}

					//Draw original sprite data
					else*/
					{
						//Output Scanline data to RGBA - DMG Mode
						if (/*config::gb_type != 2*/true)
						{
							//If raw data is 0, that's the sprites transparency
							//In this case, we leave scanline data untouched
							if ((sprites[current_sprite].raw_data[y] != 0) && (priority == 0)) { draw_sprite_pixel = true; }
							else if ((sprites[current_sprite].raw_data[y] != 0) && (priority == 1) && (bg_win_raw_data[current_pixel] == 0)) { draw_sprite_pixel = true; }

							if (draw_sprite_pixel)
							{
								switch (obp[sprites[current_sprite].raw_data[y]][pal])
								{
								case 0:
									scanline_pixel_data[current_pixel] = DMG_PAL_OBJ[0][pal];
									break;

								case 1:
									scanline_pixel_data[current_pixel] = DMG_PAL_OBJ[1][pal];
									break;

								case 2:
									scanline_pixel_data[current_pixel] = DMG_PAL_OBJ[2][pal];
									break;

								case 3:
									scanline_pixel_data[current_pixel] = DMG_PAL_OBJ[3][pal];
									break;
								}
							}

						}

						//Output Scanline data to RGBA - DMG Mode
						if (/*config::gb_type == 2*/false)
						{
							if ((bg_priority[current_pixel] == 0) && (priority == 0) && (sprites[current_sprite].raw_data[y] != 0)) { draw_sprite_pixel = true; }
							if ((bg_priority[current_pixel] == 0) && (priority == 1) && (sprites[current_sprite].raw_data[y] != 0) && (bg_win_raw_data[current_pixel] == 0)) { draw_sprite_pixel = true; }
							if ((bg_priority[current_pixel] == 1) && (sprites[current_sprite].raw_data[y] != 0) && (bg_win_raw_data[current_pixel] == 0)) { draw_sprite_pixel = true; }

							if (draw_sprite_pixel)
							{
								scanline_pixel_data[current_pixel] = sprite_colors_final[sprites[current_sprite].raw_data[y]][gbc_pal];
							}
						}
					}

					current_pixel++;
				}
			}
		}
	}

	//Copy scanline data to final buffer
	for (int x = 0; x < 0x100; x++)
	{
		final_pixel_data[(MMU.memory_map[REG_LY] * 0x100) + x] = scanline_pixel_data[x];
	}
}

/****** Prepares sprites for rendering - Pulls data from OAM, sets sprite palettes, etc ******/
void gpu::generate_sprites()
{
	uint8_t high_byte, low_byte = 0;
	uint8_t high_bit, low_bit = 0;
	uint8_t final_byte = 0;

	uint16_t sprite_tile_addr = 0;
	uint16_t sprite_height = 0;

	//Read sprite attributes from OAM
	for (int x = 0, y = 0; x < 40; x++, y += 4)
	{
		sprites[x].y = MMU.memory_map[OAM + y] - 16;
		sprites[x].x = MMU.memory_map[OAM + y + 1] - 8;
		sprites[x].tile_number = MMU.memory_map[OAM + y + 2];
		sprites[x].options = MMU.memory_map[OAM + y + 3];
	}

	//Determine if in 8x8 or 8x16 mode
	if (MMU.memory_map[REG_LCDC] & 0x04)
	{
		sprite_height = 16;

		//Set LSB of tile number to 0
		for (int x = 0; x < 40; x++) { sprites[x].tile_number &= ~0x1; }
	}

	else { sprite_height = 8; }

	uint8_t sp_zero = MMU.memory_map[REG_OBP0];
	uint8_t sp_one = MMU.memory_map[REG_OBP1];

	//Determine Sprite Palettes - From lightest to darkest
	obp[0][0] = sp_zero & 0x3;
	obp[1][0] = (sp_zero >> 2) & 0x3;
	obp[2][0] = (sp_zero >> 4) & 0x3;
	obp[3][0] = (sp_zero >> 6) & 0x3;

	obp[0][1] = sp_one & 0x3;
	obp[1][1] = (sp_one >> 2) & 0x3;
	obp[2][1] = (sp_one >> 4) & 0x3;
	obp[3][1] = (sp_one >> 6) & 0x3;

	//Load custom sprite data
	/*if (config::load_sprites) { load_sprites(); }

	//Read sprite pixel data normally	
	else*/
	{
		for (int x = 0; x < 40; x++)
		{
			sprite_tile_addr = (sprites[x].tile_number * 16) + 0x8000;
			uint8_t pixel_counter = 0;

			//Read Sprite Options
			uint8_t pal = sprites[x].options & 0x10 ? 1 : 0;

			//Cycles through tile
			for (int y = 0; y < sprite_height; y++)
			{
				//Grab High and Low Bytes for Tile - DMG mode
				if (/*config::gb_type != 2*/true)
				{
					high_byte = MMU.rb(sprite_tile_addr);
					low_byte = MMU.rb(sprite_tile_addr + 1);
				}

				//Grab High and Low Bytes for Tile - GBC mode
				else
				{
					uint8_t old_vram_bank = MMU.vram_bank;
					uint8_t temp_vram_bank = (sprites[x].options & 0x8) ? 1 : 0;
					MMU.vram_bank = temp_vram_bank;

					high_byte = MMU.rb(sprite_tile_addr);
					low_byte = MMU.rb(sprite_tile_addr + 1);

					MMU.vram_bank = old_vram_bank;
				}

				//Cycle through High and Low bytes
				for (int z = 7; z >= 0; z--)
				{
					high_bit = (high_byte >> z) & 0x01;
					low_bit = (low_byte >> z) & 0x01;
					final_byte = high_bit + (low_bit * 2);

					sprites[x].raw_data[pixel_counter] = final_byte;
					pixel_counter++;
				}

				sprite_tile_addr += 2;
			}
		}
	}

	//Handle horizontal and vertical flipping
	for (int x = 0; x < 40; x++)
	{
		uint8_t h_flip = sprites[x].options & 0x20 ? 1 : 0;
		uint8_t v_flip = sprites[x].options & 0x40 ? 1 : 0;

		if (h_flip == 1)
		{
			horizontal_flip(8, sprite_height, sprites[x].raw_data);
			horizontal_flip(8, sprite_height, sprites[x].custom_data);
		}

		if (v_flip == 1)
		{
			vertical_flip(8, sprite_height, sprites[x].raw_data);
			vertical_flip(8, sprite_height, sprites[x].custom_data);
		}
	}
}


/****** Render final frame ******/
void gpu::render_screen()
{
	//Lock source surface
	//if (SDL_MUSTLOCK(src_screen)){ SDL_LockSurface(src_screen); }
	//uint32_t* out_pixel_data = (uint32_t*)src_screen->pixels;

	uint32_t out_pixel_data[0x9000];
	//LCD On - Draw background to framebuffer
	if (MMU.memory_map[REG_LCDC] & 0x80)
	{
		//Technically, it's only necessary to copy scanlines 0-143
		//Scanlines 144+ aren't even rendered by generate_scanline()
		for (int a = 0; a < 0x9000; a++) { out_pixel_data[a] = final_pixel_data[a]; }
	}

	//LCD Off - Draw white pixels to framebuffer
	else
	{
		memset(out_pixel_data, 0xFFFFFFFF, sizeof(out_pixel_data));
	}

	videoRefresh(out_pixel_data);

	//Unlock source surface
	/*if (SDL_MUSTLOCK(src_screen)){ SDL_UnlockSurface(src_screen); }

	//Scale the source image...
	if ((config::use_scaling) && (!config::use_opengl))
	{
		apply_scaling(src_screen, temp_screen);
		SDL_BlitSurface(temp_screen, 0, gpu_screen, 0);
	}

	//Or just blit to unscaled image to screen
	else { SDL_BlitSurface(src_screen, 0, gpu_screen, 0); }

	//Blit via SDL
	if (!config::use_opengl)
	{
		if (SDL_Flip(gpu_screen) == -1) { std::cout << "Could not blit? \n"; }
	}

	//Blit via OpenGL
	else { opengl_blit(); }(*/

	//Limit FPS to 60
	/*frame_current_time = SDL_GetTicks();
	if ((frame_current_time - frame_start_time) < (1000 / 60)) { SDL_Delay((1000 / 60) - (frame_current_time - frame_start_time)); }
	else { printf("GPU : Late Blit\n"); }*/

	//Clear pixel data after frame draw
	memset(scanline_pixel_data, 0xFFFFFFFF, sizeof(scanline_pixel_data));
	memset(final_pixel_data, 0xFFFFFFFF, sizeof(final_pixel_data));
}

/****** Execute GPU Operations ******/
void gpu::step(int cpu_clock)
{
	if (MMU.gpu_reset_ticks) { gpu_clock = 0; MMU.gpu_reset_ticks = false; }

	//Enable LCD
	if ((MMU.memory_map[REG_LCDC] & 0x80) && (!lcd_enabled))
	{
		lcd_enabled = true;
		gpu_mode = 2;
	}

	//Update background tile
	if (MMU.gpu_update_bg_tile)
	{
		if (/*config::gb_type != 2*/true) { update_bg_tile(); }
		else { update_gbc_bg_tile(); }
		MMU.gpu_update_bg_tile = false;
	}

	//Update sprites
	if (MMU.gpu_update_sprite)
	{
		generate_sprites();
		MMU.gpu_update_sprite = false;
	}

	//Update background color palettes on the GBC
	if ((MMU.gpu_update_bg_colors) && (/*config::gb_type == 2*/false))
	{
		uint8_t hi_lo = (MMU.memory_map[REG_BCPS] & 0x1);
		uint8_t color = (MMU.memory_map[REG_BCPS] >> 1) & 0x3;
		uint8_t palette = (MMU.memory_map[REG_BCPS] >> 3) & 0x7;
		uint8_t auto_increment = (MMU.memory_map[REG_BCPS]) & 0x80;

		//Update lower-nibble of color
		if (hi_lo == 0)
		{
			background_colors_raw[color][palette] &= 0xFF00;
			background_colors_raw[color][palette] |= MMU.memory_map[REG_BCPD];
		}

		//Update upper-nibble of color
		else
		{
			background_colors_raw[color][palette] &= 0xFF;
			background_colors_raw[color][palette] |= (MMU.memory_map[REG_BCPD] << 8);
		}

		//Auto update palette index
		if (MMU.memory_map[REG_BCPS] & 0x80)
		{
			uint8_t new_index = MMU.memory_map[REG_BCPS] & 0x3F;
			new_index = (new_index + 1) & 0x3F;
			MMU.memory_map[REG_BCPS] = (0x80 | new_index);
		}

		//Convert RGB5 to 32-bit ARGB
		uint16_t color_bytes = background_colors_raw[color][palette];

		uint8_t red = ((color_bytes & 0x1F) * 8);
		color_bytes >>= 5;

		uint8_t green = ((color_bytes & 0x1F) * 8);
		color_bytes >>= 5;

		uint8_t blue = ((color_bytes & 0x1F) * 8);

		background_colors_final[color][palette] = 0xFF000000 | (red << 16) | (green << 8) | (blue);
		MMU.background_colors_raw[color][palette] = background_colors_raw[color][palette];

		//Update DMG BG palette when using GBC BIOS
		if (MMU.in_bios)
		{
			DMG_PAL_BG[0] = background_colors_final[0][0];
			DMG_PAL_BG[1] = background_colors_final[1][0];
			DMG_PAL_BG[2] = background_colors_final[2][0];
			DMG_PAL_BG[3] = background_colors_final[3][0];
		}

		MMU.gpu_update_bg_colors = false;
	}

	//Update sprite color palettes on the GBC
	if ((MMU.gpu_update_sprite_colors) && (/*config::gb_type == 2*/false))
	{
		uint8_t hi_lo = (MMU.memory_map[REG_OCPS] & 0x1);
		uint8_t color = (MMU.memory_map[REG_OCPS] >> 1) & 0x3;
		uint8_t palette = (MMU.memory_map[REG_OCPS] >> 3) & 0x7;
		uint8_t auto_increment = (MMU.memory_map[REG_OCPS]) & 0x80;

		//Update lower-nibble of color
		if (hi_lo == 0)
		{
			sprite_colors_raw[color][palette] &= 0xFF00;
			sprite_colors_raw[color][palette] |= MMU.memory_map[REG_OCPD];
		}

		//Update upper-nibble of color
		else
		{
			sprite_colors_raw[color][palette] &= 0xFF;
			sprite_colors_raw[color][palette] |= (MMU.memory_map[REG_OCPD] << 8);
		}

		//Auto update palette index
		if (MMU.memory_map[REG_OCPS] & 0x80)
		{
			uint8_t new_index = MMU.memory_map[REG_OCPS] & 0x3F;
			new_index = (new_index + 1) & 0x3F;
			MMU.memory_map[REG_OCPS] = (0x80 | new_index);
		}

		//Convert RGB5 to 32-bit ARGB
		uint16_t color_bytes = sprite_colors_raw[color][palette];

		uint8_t red = ((color_bytes & 0x1F) * 8);
		color_bytes >>= 5;

		uint8_t green = ((color_bytes & 0x1F) * 8);
		color_bytes >>= 5;

		uint8_t blue = ((color_bytes & 0x1F) * 8);

		sprite_colors_final[color][palette] = 0xFF000000 | (red << 16) | (green << 8) | (blue);
		MMU.sprite_colors_raw[color][palette] = sprite_colors_raw[color][palette];

		//Update DMG OBJ palettes when using GBC BIOS
		if (MMU.in_bios)
		{
			DMG_PAL_OBJ[0][0] = sprite_colors_final[0][0];
			DMG_PAL_OBJ[1][0] = sprite_colors_final[1][0];
			DMG_PAL_OBJ[2][0] = sprite_colors_final[2][0];
			DMG_PAL_OBJ[3][0] = sprite_colors_final[3][0];

			DMG_PAL_OBJ[0][1] = sprite_colors_final[0][1];
			DMG_PAL_OBJ[1][1] = sprite_colors_final[1][1];
			DMG_PAL_OBJ[2][1] = sprite_colors_final[2][1];
			DMG_PAL_OBJ[3][1] = sprite_colors_final[3][1];
		}

		MMU.gpu_update_sprite_colors = false;
	}

	//General HDMA
	if (/*(config::gb_type == 2)*/false && (MMU.gpu_hdma_in_progress) && (MMU.gpu_hdma_type == 0))
	{
		uint16_t start_addr = (MMU.memory_map[REG_HDMA1] << 8) | MMU.memory_map[REG_HDMA2];
		uint16_t dest_addr = (MMU.memory_map[REG_HDMA3] << 8) | MMU.memory_map[REG_HDMA4];

		//Ignore bottom 4 bits of start address
		start_addr &= 0xFFF0;

		//Ignore top 3 bits and bottom 4 bits of destination address
		dest_addr &= 0x1FF0;

		//Destination is ALWAYS in VRAM
		dest_addr |= 0x8000;

		uint8_t transfer_byte_count = (MMU.memory_map[REG_HDMA5] & 0x7F) + 1;

		for (uint16_t x = 0; x < (transfer_byte_count * 16); x++)
		{
			MMU.wb(dest_addr++, MMU.rb(start_addr++));
		}

		MMU.gpu_hdma_in_progress = false;
		MMU.memory_map[REG_HDMA5] = 0xFF;
	}

	//Perform LCD operations only when LCD is enabled
	if (lcd_enabled)
	{
		gpu_clock += cpu_clock;

		//Handle GPU Modes
		switch (gpu_mode)
		{
			//HBlank - Mode 0
		case 0:
			//Render scanline when 1st entering Mode 0
			if (gpu_mode_change != 0)
			{
				//Horizontal blanking DMA
				if ((/*config::gb_type == 2*/false) && (MMU.gpu_hdma_in_progress) && (MMU.gpu_hdma_type == 1))
				{
					uint16_t start_addr = (MMU.memory_map[REG_HDMA1] << 8) | MMU.memory_map[REG_HDMA2];
					uint16_t dest_addr = (MMU.memory_map[REG_HDMA3] << 8) | MMU.memory_map[REG_HDMA4];
					uint8_t line_transfer_count = (MMU.memory_map[REG_HDMA5] & 0x7F);

					start_addr += (MMU.gpu_hdma_current_line * 16);
					dest_addr += (MMU.gpu_hdma_current_line * 16);

					//Ignore bottom 4 bits of start address
					start_addr &= 0xFFF0;

					//Ignore top 3 bits and bottom 4 bits of destination address
					dest_addr &= 0x1FF0;

					//Destination is ALWAYS in VRAM
					dest_addr |= 0x8000;

					for (uint16_t x = 0; x < 16; x++)
					{
						MMU.wb(dest_addr++, MMU.rb(start_addr++));
					}

					MMU.gpu_hdma_current_line++;

					if (line_transfer_count == 0)
					{
						MMU.gpu_hdma_in_progress = false;
						MMU.memory_map[REG_HDMA5] = 0xFF;
						MMU.gpu_hdma_current_line = 0;
					}

					else { line_transfer_count--; MMU.memory_map[REG_HDMA5] = line_transfer_count; }
				}

				generate_scanline();
				gpu_mode_change = 0;

				//HBlank STAT INT
				if (MMU.memory_map[REG_STAT] & 0x08) { MMU.memory_map[REG_IF] |= 2; }
			}

			if (gpu_clock >= 456)
			{
				gpu_clock -= 456;
				gpu_mode = 2;
				MMU.memory_map[REG_LY]++;
				scanline_compare();

				//Render Screen after 144th line
				if (MMU.memory_map[REG_LY] == 144)
				{
					MMU.memory_map[REG_IF] |= 1;
					gpu_mode = 1;
					render_screen();
					frame_start_time = SDL_GetTicks();
				}
			}
			break;

			//VBlank - Mode 1
		case 1:
			//Disable LCD - Must be done during VBlank only
			if (!(MMU.memory_map[REG_LCDC] & 0x80))
			{
				lcd_enabled = false;
				MMU.memory_map[REG_LY] = 0;
				scanline_compare();
				gpu_clock = 0;
				gpu_mode = 0;
				break;
			}

			if (gpu_mode_change != 1)
			{
				gpu_mode_change = 1;

				//VBlank STAT INT
				if (MMU.memory_map[REG_STAT] & 0x10) { MMU.memory_map[REG_IF] |= 2; }

				//Dump sprites and BG Tiles every VBlank
				/*if (config::dump_sprites)
				{
					dump_sprites();
					if ((config::mouse_click) && (dump_tile_0 < 0x100) && (dump_mode == 0)) { dump_bg_tileset_0(); }
					else if ((config::mouse_click) && (dump_tile_1 < 0x100) && (dump_mode == 1)) { dump_bg_tileset_1(); }
					else if ((config::mouse_click) && (dump_tile_win < 0x100)) { dump_bg_window(); }
					config::mouse_click = false;
				}

				//Load custom BG tiles every VBlank
				if (config::load_sprites)
				{
					load_bg_tileset_1();
					load_bg_tileset_0();
				}*/
			}

			if (gpu_clock >= 456)
			{
				gpu_clock -= 456;
				MMU.memory_map[REG_LY]++;
				scanline_compare();

				//After 10 lines, VBlank is done, returns to top screen in Mode 2
				if (MMU.memory_map[REG_LY] == 154)
				{
					gpu_mode = 2;
					MMU.memory_map[REG_LY] = 0;
					scanline_compare();
				}
			}
			break;

			//OAM Read - Mode 2
		case 2:
			if (gpu_mode_change != 2)
			{
				gpu_mode_change = 2;

				//OAM STAT INT
				if (MMU.memory_map[REG_STAT] & 0x20) { MMU.memory_map[REG_IF] |= 2; }
			}

			if (gpu_clock >= 80) { gpu_mode = 3; }
			break;

			//VRAM Read - Mode 3
		case 3:
			if (gpu_mode_change != 3) { gpu_mode_change = 3; }
			if (gpu_clock >= 252) { gpu_mode = 0; }
			break;
		}
	}

	MMU.memory_map[REG_STAT] = (MMU.memory_map[REG_STAT] & ~0x3) + gpu_mode;
}