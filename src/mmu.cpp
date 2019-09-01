#include "cartridge.h"
#include "cpu.h"
#include "gpu.h"
#include "key.h"
#include "mmu.h"


mmu MMU;

Unmapped unmapped;
AudioMMIO audiommio;

void mmu::Reset() {
	for (unsigned n = 0x0000; n <= 0xffff; n++) mmio[n] = &unmapped;
	for (unsigned n = 0xff10; n <= 0xff3f; n++) mmio[n] = &audiommio;
	//printf("[MMU] Reset\n");


	read_only_bank.resize(0x200);
	for (int x = 0; x < 0x200; x++) { read_only_bank[x].resize(0x4000, 0); }

	random_access_bank.resize(0x10);
	for (int x = 0; x < 0x10; x++) { random_access_bank[x].resize(0x2000, 0); }

	working_ram_bank.resize(0x8);
	for (int x = 0; x < 0x8; x++) { working_ram_bank[x].resize(0x1000, 0); }

	video_ram.resize(0x2);
	for (int x = 0; x < 0x2; x++) { video_ram[x].resize(0x2000, 0); }
}

static uint8_t mbc_read(uint16_t) {
	__debugbreak();
	return 0;
}

static void mbc_write(uint16_t, uint8_t) {
	__debugbreak();
}

uint8_t mmu::rb(uint16_t address) {
	//Read from BIOS
	if (in_bios)
	{
		//GBC BIOS reads from 0x00 to 0xFF and 0x200 to 0x900
		//0x100 - 0x1FF is reserved for the Nintendo logo + checksum + first lines of game code
		//For the latter, just read from the cartridge ROM
		if ((bios_size == 0x900) && (address > 0x100) && (address < 0x200)) { return memory_map[address]; }

		else if (address == 0x100)
		{
			in_bios = false;
			printf("MMU : Exiting BIOS \n");

			//For DMG on GBC games, we switch back to DMG Mode (we just take the colors the BIOS gives us)
			if ((bios_size == 0x900) && (memory_map[ROM_COLOR] == 0)) { /*config::gb_type = 1;*/ }
		}

		else if (address < bios_size) { __debugbreak(); return 0;/*bios[address];*/ }
	}

	//Read using ROM Banking
	if ((address >= 0x4000) && (address <= 0x7FFF) && (GetCartridge()->GetCartridgeType() != ECartridgeType::ROMONLY))
	{
		return mbc_read(address);
	}

	//Read using RAM Banking
	if ((address >= 0xA000) && (address <= 0xBFFF) && (cart_ram) && (GetCartridge()->GetCartridgeType() != ECartridgeType::ROMONLY))
	{
		return mbc_read(address);
	}

	//Read from VRAM, GBC uses banking
	if ((address >= 0x8000) && (address <= 0x9FFF))
	{
		//GBC read from VRAM Bank 1
		if ((vram_bank == 1) && (/*config::gb_type == 2*/false)) { return video_ram[1][address - 0x8000]; }

		//GBC read from VRAM Bank 0 - DMG read normally, also from Bank 0, though it doesn't use banking technically
		else { return video_ram[0][address - 0x8000]; }
	}

	//In GBC mode, read from Working RAM using Banking
	if ((address >= 0xC000) && (address <= 0xDFFF) && (/*config::gb_type == 2*/false))
	{
		//Read from Bank 0 always when address is within 0xC000 - 0xCFFF
		if ((address >= 0xC000) && (address <= 0xCFFF)) { return working_ram_bank[0][address - 0xC000]; }

		//Read from selected Bank when address is within 0xD000 - 0xDFFF
		else if ((address >= 0xD000) && (address <= 0xDFFF)) { return working_ram_bank[wram_bank][address - 0xD000]; }
	}

	//Read background color palette data
	if (address == REG_BCPD)
	{
		uint8_t hi_lo = (memory_map[REG_BCPS] & 0x1);
		uint8_t color = (memory_map[REG_BCPS] >> 1) & 0x3;
		uint8_t palette = (memory_map[REG_BCPS] >> 3) & 0x7;

		//Read lower-nibble of color
		if (hi_lo == 0)
		{
			return (background_colors_raw[color][palette] & 0xFF);
		}

		//Read upper-nibble of color
		else
		{
			return (background_colors_raw[color][palette] >> 8);
		}
	}


	//Read sprite color palette data
	if (address == REG_OCPD)
	{
		uint8_t hi_lo = (memory_map[REG_OCPS] & 0x1);
		uint8_t color = (memory_map[REG_OCPS] >> 1) & 0x3;
		uint8_t palette = (memory_map[REG_OCPS] >> 3) & 0x7;

		//Read lower-nibble of color
		if (hi_lo == 0)
		{
			return (sprite_colors_raw[color][palette] & 0xFF);
		}

		//Read upper-nibble of color
		else
		{
			return (sprite_colors_raw[color][palette] >> 8);
		}
	}

	//Read from P1
	else if (address == 0xFF00) { return 0;/* pad.read();*/ }

	//Read normally
	return memory_map[address];
	//return mmio[addr]->mmio_read(addr);
}

void mmu::wb(uint16_t address, uint8_t value) {
	if (GetCartridge()->GetCartridgeType() != ECartridgeType::ROMONLY) { mbc_write(address, value); }

	//Read from VRAM, GBC uses banking
	if ((address >= 0x8000) && (address <= 0x9FFF))
	{
		//GBC read from VRAM Bank 1
		if ((vram_bank == 1) && (/*config::gb_type == 2*/false)) { video_ram[1][address - 0x8000] = value; }

		//GBC read from VRAM Bank 0 - DMG read normally, also from Bank 0, though it doesn't use banking technically
		else { video_ram[0][address - 0x8000] = value; }

		//VRAM - Background tiles update
		if ((address >= 0x8000) && (address <= 0x97FF))
		{
			gpu_update_bg_tile = true;
			gpu_update_addr.push_back(address);
			if (address <= 0x8FFF) { gpu_update_sprite = true; }
		}
	}

	//BGP
	else if (address == REG_BGP)
	{
		gpu_update_bg_tile = true;
		gpu_update_addr.push_back(address);
		memory_map[address] = value;
	}

	//OBP0 and OBP1
	else if ((address == REG_OBP0) || (address == REG_OBP1))
	{
		gpu_update_sprite = true;
		memory_map[address] = value;
	}

	//Current scanline
	else if (address == REG_LY)
	{
		memory_map[0xFF44] = 0;
	}

	//LCDC - Sprite mode changes
	else if (address == REG_LCDC)
	{
		uint8_t current_bit = (memory_map[REG_LCDC] & 0x04) ? 1 : 0;
		uint8_t new_bit = (value & 0x04) ? 1 : 0;

		//We're switching sprite modes, so update all sprites)
		if (current_bit != new_bit) { gpu_update_sprite = true; }

		memory_map[address] = value;
	}

	//DMA transfer
	else if (address == REG_DMA)
	{
		uint16_t dma_orig = value << 8;
		uint16_t dma_dest = 0xFE00;
		while (dma_dest < 0xFEA0) { wb(dma_dest++, rb(dma_orig++)); }
		gpu_update_sprite = true;
	}

	//Internal RAM - Write to ECHO RAM as well
	else if ((address >= 0xC000) && (address <= 0xDFFF))
	{
		//DMG mode - Normal writes
		if (/*config::gb_type != 2*/true)
		{
			memory_map[address] = value;
			if (address + 0x2000 < 0xFDFF) { memory_map[address + 0x2000] = value; }
		}

		//GBC mode - Use banks
		else if (/*config::gb_type == 2*/false)
		{
			//Write to Bank 0 always when address is within 0xC000 - 0xCFFF
			if ((address >= 0xC000) && (address <= 0xCFFF)) { working_ram_bank[0][address - 0xC000] = value; }

			//Write to selected Bank when address is within 0xD000 - 0xDFFF
			else if ((address >= 0xD000) && (address <= 0xDFFF)) { working_ram_bank[wram_bank][address - 0xD000] = value; }
		}
	}

	//ECHO RAM - Write to Internal RAM as well
	else if ((address >= 0xE000) && (address <= 0xFDFF))
	{
		memory_map[address] = value;
		memory_map[address - 0x2000] = value;
	}

	//OAM - Direct writes
	else if ((address >= 0xFE00) && (address <= 0xFEA0))
	{
		memory_map[address] = value;
		gpu_update_sprite = true;
	}

	//P1 - Joypad register
	else if (address == REG_P1) { /*pad.column_id = (value & 0x30); memory_map[REG_P1] = pad.read();*/ }

	//Update Sound Channels
	else if ((address >= 0xFF10) && (address <= 0xFF25))
	{
		memory_map[address] = value;
		apu_update_channel = true;
		apu_update_addr = address;
	}

	//HDMA transfer
	else if (address == REG_HDMA5)
	{
		//Halt Horizontal DMA transfer if one is already in progress and 0 is now written to Bit 7
		if (((value & 0x80) == 0) && (gpu_hdma_in_progress))
		{
			gpu_hdma_in_progress = false;
			gpu_hdma_current_line = 0;
			value = 0x80;
		}

		//If not halting a current HDMA transfer, start a new one, determine its type
		else
		{
			gpu_hdma_in_progress = true;
			gpu_hdma_current_line = 0;
			gpu_hdma_type = (value & 0x80) ? 1 : 0;
			value &= ~0x80;
		}

		memory_map[address] = value;
	}

	//NR52
	else if (address == REG_NR52)
	{
		//Only bit 7 is writable
		if (value & 0x80) { memory_map[address] |= 0x80; }

		//When Bit 7 is cleared, so are Bits 0-3, Bits 6-4 are ALWAYS set to 1
		else { memory_map[address] = 0x70; }
	}

	//VBK - Update VRAM bank
	else if (address == REG_VBK)
	{
		vram_bank = value & 0x1;
		memory_map[address] = value;
	}

	//KEY1 - Double-Normal speed switch
	else if (address == REG_KEY1)
	{
		value &= 0x1;
		if (value == 1) { memory_map[address] |= 0x1; }
		else { memory_map[address] &= ~0x1; }
	}

	//BCPD - Update background color palettes
	else if (address == REG_BCPD)
	{
		gpu_update_bg_colors = true;
		memory_map[address] = value;
	}

	//OCPD - Update sprite color palettes
	else if (address == REG_OCPD)
	{
		gpu_update_sprite_colors = true;
		memory_map[address] = value;
	}

	//SVBK - Update Working RAM bank
	else if (address == REG_SVBK)
	{
		wram_bank = value & 0x7;
		if (wram_bank == 0) { wram_bank = 1; }
		memory_map[address] = value;
	}

	else if (address > 0x7FFF) { memory_map[address] = value; }
	//mmio[addr]->mmio_write(addr, data);
}
