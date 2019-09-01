#pragma once

#include <stdint.h>
#include "mmu.h"

class z80 : MMIO {
public:
	enum class Interrupt : unsigned {
		Vblank,
		Stat,
		Timer,
		Serial,
		Joypad,
	};

private:
	// Instruction Helpers"
	inline void InvalidOpCode(const uint16_t OpCode);

	inline uint8_t READ(const uint16_t addr);
	inline uint8_t PC_READ();
	inline uint8_t FF_READ(const uint8_t ffoffset);

	inline void WRITE(const uint16_t addr, const uint8_t data);
	inline void FF_WRITE(const uint8_t ffoffset, const uint8_t data);

	inline void PC_MOD(const uint16_t data);

	inline void PUSH(const uint8_t r1, const uint8_t r2);

	// 16-BIT LOADS:
	inline void ld_rr_nn(uint8_t& r1, uint8_t& r2);
	inline void push_rr(const uint8_t r1, const uint8_t r2);
	inline void pop_rr(uint8_t& r1, uint8_t& r2);

	// 8-BIT ALU:
	inline void add_a_u8(const uint8_t u8);
	inline void adc_a_u8(const uint8_t u8);
	inline void sub_a_u8(const uint8_t u8);
	inline void sbc_a_u8(const uint8_t u8);
	inline void and_a_u8(const uint8_t u8);
	inline void or_a_u8(const uint8_t u8);
	inline void xor_a_u8(const uint8_t u8);
	inline void cp_a_u8(const uint8_t u8);
	inline void inc_r(uint8_t& r);
	inline void dec_r(uint8_t& r);

	// 16-BIT ARITHMETIC:
	inline void add_hl_rr(const uint8_t rh, const uint8_t rl);
	inline void inc_rr(uint8_t& rh, uint8_t& rl);
	inline void dec_rr(uint8_t& rh, uint8_t& rl);
	inline void sp_plus_n(uint16_t& sumout);

	// JUMPS:
	inline void jp_nn();
	inline void rst_n(const uint8_t n);
	inline void jr_disp();

	// CALLS, RESTARTS AND RETURNS:
	inline void call_nn();
	inline void ret();

	// CB OPCODES (Shifts, rotates and bits):
	inline void swap_r(uint8_t& r);
	inline void rlc_r(uint8_t& r);
	inline void rl_r(uint8_t& r);
	inline void rrc_r(uint8_t& r);
	inline void rr_r(uint8_t& r);
	inline void sla_r(uint8_t& r);
	inline void sra_r(uint8_t& r);
	inline void srl_r(uint8_t& r);
	inline void bitn_u8(const uint8_t bitmask, const uint8_t u8);
	inline void z80::setn_mem_hl(const uint8_t n);
	inline void resn_mem_hl(const uint8_t n);

	inline void ProcessCB();

public:
	void Reset();
	void interrupt_raise(Interrupt id);
	void interrupt_test();
	void Process();

	// memory
	unsigned wram_addr(uint16_t addr) const;
	uint8_t mmio_read(uint16_t addr) override;
	void mmio_write(uint16_t addr, uint8_t data) override;
	uint8_t dma_read(uint16_t addr);
	void dma_write(uint16_t addr, uint8_t data);

	// timing
	void add_clocks(unsigned clocks);
	void timer_262144hz();
	void timer_65536hz();
	void timer_16384hz();
	void timer_8192hz();
	void timer_4096hz();

private:
	// Registers
	uint8_t a_;		// Accumulator
	uint8_t f_;		// Flags

	uint8_t b_;
	uint8_t c_;
	uint8_t d_;
	uint8_t e_;
	uint8_t h_;
	uint8_t l_;
public:
	uint16_t sp;	// Stack Pointer
	uint16_t pc;	// Program Counter/Pointer

	//uint32_t cycleCounter_;
	
	struct Status {
		unsigned clock;

		//$ff00  JOYP
		bool p15;
		bool p14;
		uint8_t joyp;
		uint8_t mlt_req;

		//$ff01  SB
		uint8_t serial_data;
		unsigned serial_bits;

		//$ff02  SC
		bool serial_transfer;
		bool serial_clock;

		//$ff04  DIV
		uint8_t div;

		//$ff05  TIMA
		uint8_t tima;

		//$ff06  TMA
		uint8_t tma;

		//$ff07  TAC
		bool timer_enable;
		unsigned timer_clock;

		//$ff0f  IF
		bool interrupt_request_joypad;
		bool interrupt_request_serial;
		bool interrupt_request_timer;
		bool interrupt_request_stat;
		bool interrupt_request_vblank;

		//$ff4d  KEY1
		bool speed_double;
		bool speed_switch;

		//$ff51,$ff52  HDMA1,HDMA2
		uint16_t dma_source;

		//$ff53,$ff54  HDMA3,HDMA4
		uint16_t dma_target;

		//$ff55  HDMA5
		bool dma_mode;
		uint16_t dma_length;
		bool dma_completed;

		//$ff6c  ???
		uint8_t ff6c;

		//$ff70  SVBK
		uint8_t wram_bank;

		//$ff72-$ff75  ???
		uint8_t ff72;
		uint8_t ff73;
		uint8_t ff74;
		uint8_t ff75;

		//$ffff  IE
		bool interrupt_enable_joypad;
		bool interrupt_enable_serial;
		bool interrupt_enable_timer;
		bool interrupt_enable_stat;
		bool interrupt_enable_vblank;
	} status;

	struct OAMDMA {
		bool active;
		uint8_t bank;
		uint8_t offset;
	} oamdma;

	uint8_t wram[32768];  //GB=8192, GBC=32768
	uint8_t hram[128];

	bool halt;
	bool stop;
	bool ei;
	bool ime;
};

extern z80 Z80;