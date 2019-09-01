#pragma once

#include <memory>
#include "mmu.h"
struct ECartridgeType {
	enum Enum : uint8_t {
		ROMONLY					= 0x00,
		MBC1					= 0x01,
		MBC1_RAM				= 0x02,
		MBC1_RAM_BATTERY		= 0x03,
		MBC2					= 0x05,
		MBC2_BATTERY			= 0x06,
		ROM_RAM					= 0x08,
		ROM_RAM_BATTERY			= 0x09,
		MMM01					= 0x0B,
		MMM01_RAM				= 0x0C,
		MMM01_RAM_BATTERY		= 0x0D,
		MBC3_TIMER_BATTERY		= 0x0F,
		MBC3_TIMER_RAM_BATTERY	= 0x10,
		MBC3					= 0x11,
		MBC3_RAM				= 0x12,
		MBC3_RAM_BATTERY		= 0x13,
		MBC4					= 0x15,
		MBC4_RAM				= 0x16,
		MBC4_RAM_BATTERY		= 0x17,
		MBC5					= 0x19,
		MBC5_RAM				= 0x1A,
		MBC5_RAM_BATTERY		= 0x1B,
		MBC5_RUMBLE				= 0x1C,
		MBC5_RUMBLE_RAM			= 0x1D,
		MBC5_RUMBLE_RAM_BATTERY	= 0x1E,
		POCKET_CAMERA			= 0xFC,
		BANDAI_TAMA5			= 0xFD,
		HuC3					= 0xFE,
		HuC1_RAM_BATTERY		= 0xFF,
	};
};

class Cartridge : MMIO {
private:
	struct ERomOffsets {
		enum Enum : uint16_t {
			// Header:
			ENTRY_POINT				= 0x0100,
			NINTENDO_LOGO			= 0x0104,
			TITLE					= 0x0134,
			MANUFACTURER_CODE		= 0x013F,
			CGB_FLAG				= 0x0143,
			NEW_LICENSEE_CODE		= 0x0144,
			SGB_FLAG				= 0x0146,
			CARTRIDGE_TYPE			= 0x0147,
			ROM_SIZE				= 0x0148,
			RAM_SIZE				= 0x0149,
			DESTINATION_CODE		= 0x014A,
			OLD_LICENSEE_CODE		= 0x014B,
			MASK_ROM_VERSION_NUMBER	= 0x014C,
			HEADER_CHECKSUM			= 0x014D,
			GLOBAL_CHECKSUM			= 0x014E,

			//
		};
	};

	const static int MAX_TITLE_LENGTH = 16;
public:
	Cartridge() = default;
	~Cartridge();
private:
	Cartridge(const Cartridge&) = delete;
	Cartridge(Cartridge&&) = delete;
	Cartridge& operator=(const Cartridge&) = delete;
	Cartridge& operator=(Cartridge&&) = delete;

public:
	void LoadFile(const char * const filename);
	
	inline uint8_t* GetRom() { return m_Rom; }
	inline int64_t GetSize() { return m_Size; }
	inline const char * const GetTitle() { return m_Title; }
	inline const uint16_t GetLicenseeCode() { return m_Rom[ERomOffsets::OLD_LICENSEE_CODE] != 0x33 ? m_Rom[ERomOffsets::OLD_LICENSEE_CODE] : (m_Rom[ERomOffsets::NEW_LICENSEE_CODE] << 8) | m_Rom[ERomOffsets::NEW_LICENSEE_CODE + 1]; }
	inline ECartridgeType::Enum GetCartridgeType() { return static_cast<ECartridgeType::Enum>(m_Rom[ERomOffsets::CARTRIDGE_TYPE]); }
	inline uint8_t GetRomSize() { return m_Rom[ERomOffsets::ROM_SIZE]; }
	inline uint8_t GetRamSize() { return m_Rom[ERomOffsets::RAM_SIZE]; }
	inline const bool IsSGB() { return m_Rom[ERomOffsets::SGB_FLAG] == 0x03; }
	inline const bool IsCGB() { return (m_Rom[ERomOffsets::CGB_FLAG] >> 7) & 1; }

	uint8_t mmio_read(uint16_t addr) override;
	void mmio_write(uint16_t addr, uint8_t data) override;

private:
	uint8_t m_Rom[0x10000];
	int64_t m_Size;
	char m_Title[MAX_TITLE_LENGTH + 1];

	uint8_t rom_read(unsigned addr);
	void rom_write(unsigned addr, uint8_t data);
	uint8_t ram_read(unsigned addr);
	void ram_write(unsigned addr, uint8_t data);

	struct MBC0 : MMIO {
		uint8_t mmio_read(uint16_t addr) override;
		void mmio_write(uint16_t addr, uint8_t data) override;
		void power();
	} mbc0;
};

extern Cartridge*  GetCartridge();