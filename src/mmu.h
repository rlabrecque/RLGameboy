#pragma once

#include <stdint.h>
#include <vector>

struct MMIO {
	virtual uint8_t mmio_read(uint16_t addr) = 0;
	virtual void mmio_write(uint16_t addr, uint8_t data) = 0;
};

struct Unmapped : MMIO {
	uint8_t mmio_read(uint16_t) { return 0x00; }
	void mmio_write(uint16_t, uint8_t) {  }
};

struct AudioMMIO : MMIO {
	uint8_t mmio_read(uint16_t) { return 0x00; }
	void mmio_write(uint16_t, uint8_t) { }
};

class mmu {
public:
	MMIO* mmio[65536];
	void Reset();

	/* Read 8-bit byte from a given address */
	uint8_t rb(uint16_t addr);
	
	/* Write 8-bit byte to a given address */
	void wb(uint16_t addr, uint8_t data);


	uint8_t* memory_map;

	//Memory Banks
	std::vector< std::vector<uint8_t> > read_only_bank;
	std::vector< std::vector<uint8_t> > random_access_bank;

	//Working RAM Banks - GBC only
	std::vector< std::vector<uint8_t> > working_ram_bank;

	std::vector< std::vector<uint8_t> > video_ram;

	const uint16_t rom_bank = 1;
	const uint8_t ram_bank = 0;
	uint8_t wram_bank = 1;
	uint8_t vram_bank = 0;
	const uint8_t bank_bits = 0;
	const uint8_t bank_mode = 0;

	const bool cart_battery = false;
	const bool cart_ram = false;
	const bool cart_rtc = false;
	const bool rtc_enabled = false;

	uint16_t sprite_colors_raw[4][8];
	uint16_t background_colors_raw[4][8];

	bool in_bios = false;
	uint8_t bios_type;
	uint32_t bios_size;

	bool gpu_update_bg_tile;
	bool gpu_update_sprite;
	bool gpu_reset_ticks;
	bool gpu_hdma_in_progress;
	uint8_t gpu_hdma_type;
	uint8_t gpu_hdma_current_line;
	bool gpu_update_sprite_colors;
	bool gpu_update_bg_colors;
	std::vector<uint16_t> gpu_update_addr;

	bool apu_update_channel;
	uint16_t apu_update_addr;
};

extern mmu MMU;
