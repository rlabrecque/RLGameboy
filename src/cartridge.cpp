#include "cartridge.h"
#include "mmu.h"

#include <fstream>

static const Cartridge cartridgeLocal;
Cartridge*  GetCartridge() {
	return const_cast<Cartridge*>(&cartridgeLocal);
}

Cartridge::~Cartridge() {
	/*if (m_Rom != nullptr) {
		delete[] m_Rom;
		m_Rom = nullptr;
	}*/
}

void Cartridge::LoadFile(const char * const filename) {
	/*if (m_Rom != nullptr) {
		delete[] m_Rom;
		m_Rom = nullptr;
	}*/

	for (unsigned n = 0x0000; n <= 0x7fff; n++) MMU.mmio[n] = this;
	for (unsigned n = 0xa000; n <= 0xbfff; n++) MMU.mmio[n] = this;
	MMU.mmio[0xff50] = this;
	

	std::fstream file;
	file.open(filename, std::ios::in | std::ios::binary);
	if (file) {
		file.seekg(0, std::ios::end);
		m_Size = file.tellg();
		file.seekg(0, std::ios::beg);
		//m_Rom = new uint8_t[static_cast<uint32_t>(m_Size)];
		file.read((char*)m_Rom, m_Size);
		file.close();
	}
	else {
		printf("Cartridge::LoadFile failed opening %s\n", filename);
		__debugbreak();
	}

	memcpy(m_Title, &m_Rom[0x0134], MAX_TITLE_LENGTH);
	m_Title[MAX_TITLE_LENGTH] = '\0';

	if (GetCartridgeType() != ECartridgeType::ROMONLY) {
		__debugbreak();
	}

	MMU.memory_map = m_Rom;
	//printf("[Cartridge] LoadFile - Size: %lld | Title: %s | LicenseeCode: 0x%X | CartridgeType: 0x%X | Rom Size: 0x%X | Ram Size: 0x%X | IsCGB: %s | IsSGB: %s\n",
	//	GetSize(), GetTitle(), GetLicenseeCode(), GetCartridgeType(), GetRomSize(), GetRamSize(), IsCGB() ? "true" : "false", IsSGB() ? "true" : "false");
}

uint8_t Cartridge::mmio_read(uint16_t addr) {
	if (addr == 0xff50) return 0x00;

	/*if (bootrom_enable) {
		const uint8* data = nullptr;
		switch (system.revision) {
		default:
		case System::Revision::GameBoy: data = system.bootROM.dmg; break;
		case System::Revision::SuperGameBoy: data = system.bootROM.sgb; break;
		case System::Revision::GameBoyColor: data = system.bootROM.cgb; break;
		}
		if (addr >= 0x0000 && addr <= 0x00ff) return data[addr];
		if (addr >= 0x0200 && addr <= 0x08ff && system.cgb()) return data[addr - 256];
	}*/

	//return mapper->mmio_read(addr);
	return mbc0.mmio_read(addr);
}

void Cartridge::mmio_write(uint16_t addr, uint8_t data) {
	/*if (bootrom_enable && addr == 0xff50) {
		bootrom_enable = false;
		return;
	}*/

	//mapper->mmio_write(addr, data);
	return mbc0.mmio_write(addr, data);
}

uint8_t Cartridge::MBC0::mmio_read(uint16_t addr) {
	if ((addr & 0x8000) == 0x0000) {  //$0000-7fff
		return GetCartridge()->rom_read(addr);
	}

	if ((addr & 0xe000) == 0xa000) {  //$a000-bfff
		return GetCartridge()->ram_read(addr & 0x1fff);
	}

	return 0x00;
}

void Cartridge::MBC0::mmio_write(uint16_t addr, uint8_t data) {
	if ((addr & 0xe000) == 0xa000) {  //$a000-bfff
		GetCartridge()->ram_write(addr & 0x1fff, data);
		return;
	}
}

uint8_t Cartridge::rom_read(unsigned addr) {
	//if (addr >= GetRomSize()) addr %= GetRomSize();
	//return romdata[addr];
	return m_Rom[addr];
}

void Cartridge::rom_write(unsigned addr, uint8_t data) {
	//if (addr >= romsize) addr %= romsize;
	//romdata[addr] = data;
	m_Rom[addr] = data;
}

uint8_t Cartridge::ram_read(unsigned addr) {
	//if (ramsize == 0) return 0x00;
	//if (addr >= ramsize) addr %= ramsize;
	//return ramdata[addr];
	return m_Rom[addr];
}

void Cartridge::ram_write(unsigned addr, uint8_t data) {
	//if (ramsize == 0) return;
	//if (addr >= ramsize) addr %= ramsize;
	//ramdata[addr] = data;
	m_Rom[addr] = data;
}