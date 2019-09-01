#include "cpu.h"
#include "mmu.h"
#include "gpu.h"

#include <stdio.h>

enum {
	FLAG_Z = 0x80,
	FLAG_N = 0x40,
	FLAG_H = 0x20,
	FLAG_C = 0x10,
};

#define setZFlag()		(f_ |= FLAG_Z)
#define zeroSet()		(f_ & FLAG_Z)
#define clearZFlag()	(f_ &= 0x7F)

#define setNFlag()		(f_ |= FLAG_N)
#define negativeSet()	(f_ & FLAG_N)
#define clearNFlag()	(f_ &= 0xBF)

#define setHFlag()		(f_ |= FLAG_H)
#define halfSet()		(f_ & FLAG_H)
#define clearHFlag()	(f_ &= 0xDF)

#define setCFlag()		(f_ |= FLAG_C)
#define carrySet()		(f_ & FLAG_C)
#define clearCFlag()	(f_ &= 0xEF) // TODO: Should be ~FLAG_C probably

#define carryBit() 	(f_ & FLAG_C ? 1 : 0)

#define bc() (b_ << 8 | c_)
#define de() (d_ << 8 | e_)
#define hl() (h_ << 8 | l_)

z80 Z80;

void z80::Reset() {
	//printf("[Z80] Reset\n");
	//cycleCounter_ = 0;

	{
		for (unsigned n = 0xc000; n <= 0xdfff; n++) MMU.mmio[n] = this;  //WRAM
		for (unsigned n = 0xe000; n <= 0xfdff; n++) MMU.mmio[n] = this;  //WRAM (mirror)
		for (unsigned n = 0xff80; n <= 0xfffe; n++) MMU.mmio[n] = this;  //HRAM

		MMU.mmio[0xff00] = this;  //JOYP
		MMU.mmio[0xff01] = this;  //SB
		MMU.mmio[0xff02] = this;  //SC
		MMU.mmio[0xff04] = this;  //DIV
		MMU.mmio[0xff05] = this;  //TIMA
		MMU.mmio[0xff06] = this;  //TMA
		MMU.mmio[0xff07] = this;  //TAC
		MMU.mmio[0xff0f] = this;  //IF
		MMU.mmio[0xff46] = this;  //DMA
		MMU.mmio[0xffff] = this;  //IE
	}

	{
		a_ = 0x01;
		f_ = 0xB0;

		b_ = 0x00;
		c_ = 0x13;
		d_ = 0x00;
		e_ = 0xD8;
		h_ = 0x01;
		l_ = 0x4D;

		sp = 0xFFFE;
		pc = 0x100; // bios skip
	}


	{
		status.clock = 0;

		status.p15 = 0;
		status.p14 = 0;
		status.joyp = 0;
		status.mlt_req = 0;

		status.serial_data = 0;
		status.serial_bits = 0;

		status.serial_transfer = 0;
		status.serial_clock = 0;

		status.div = 0;

		status.tima = 0;

		status.tma = 0;

		status.timer_enable = 0;
		status.timer_clock = 0;

		status.interrupt_request_joypad = 0;
		status.interrupt_request_serial = 0;
		status.interrupt_request_timer = 0;
		status.interrupt_request_stat = 0;
		status.interrupt_request_vblank = 0;

		status.speed_double = 0;
		status.speed_switch = 0;

		status.dma_source = 0;
		status.dma_target = 0;

		status.dma_mode = 0;
		status.dma_length = 0;
		status.dma_completed = true;

		status.ff6c = 0;
		status.ff72 = 0;
		status.ff73 = 0;
		status.ff74 = 0;
		status.ff75 = 0;

		status.wram_bank = 1;

		status.interrupt_enable_joypad = 0;
		status.interrupt_enable_serial = 0;
		status.interrupt_enable_timer = 0;
		status.interrupt_enable_stat = 0;
		status.interrupt_enable_vblank = 0;

		oamdma.active = false;
		oamdma.bank = 0;
		oamdma.offset = 0;
	}

	{
		for (auto& n : wram) n = 0x00;
		for (auto& n : hram) n = 0x00;
	}

	{
		halt = false;
		stop = false;
		ei = false;
		ime = false;
	}
}

unsigned z80::wram_addr(uint16_t addr) const {
	addr &= 0x1fff;
	if (addr < 0x1000) return addr;
	auto bank = status.wram_bank + (status.wram_bank == 0);
	return (bank * 0x1000) + (addr & 0x0fff);
}

uint8_t z80::mmio_read(uint16_t addr) {
	if (addr >= 0xc000 && addr <= 0xfdff) return wram[wram_addr(addr)];
	if (addr >= 0xff80 && addr <= 0xfffe) return hram[addr & 0x7f];

	if (addr == 0xff00) {  //JOYP
		//mmio_joyp_poll();
		return (status.p15 << 5)
			| (status.p14 << 4)
			| (status.joyp << 0);
	}

	if (addr == 0xff01) {  //SB
		return 0xff;
	}

	if (addr == 0xff02) {  //SC
		return (status.serial_transfer << 7)
			| (status.serial_clock << 0);
	}

	if (addr == 0xff04) {  //DIV
		return status.div;
	}

	if (addr == 0xff05) {  //TIMA
		return status.tima;
	}

	if (addr == 0xff06) {  //TMA
		return status.tma;
	}

	if (addr == 0xff07) {  //TAC
		return (status.timer_enable << 2)
			| (status.timer_clock << 0);
	}

	if (addr == 0xff0f) {  //IF
		return (status.interrupt_request_joypad << 4)
			| (status.interrupt_request_serial << 3)
			| (status.interrupt_request_timer << 2)
			| (status.interrupt_request_stat << 1)
			| (status.interrupt_request_vblank << 0);
	}

	if (addr == 0xff4d) {  //KEY1
		return (status.speed_double << 7);
	}

	if (addr == 0xff55) {  //HDMA5
		return (status.dma_completed << 7)
			| (((status.dma_length / 16) - 1) & 0x7f);
	}

	if (addr == 0xff56) {  //RP
		return 0x02;
	}

	if (addr == 0xff6c) {  //???
		return 0xfe | status.ff6c;
	}

	if (addr == 0xff70) {  //SVBK
		return status.wram_bank;
	}

	if (addr == 0xff72) {  //???
		return status.ff72;
	}

	if (addr == 0xff73) {  //???
		return status.ff73;
	}

	if (addr == 0xff74) {  //???
		return status.ff74;
	}

	if (addr == 0xff75) {  //???
		return 0x8f | status.ff75;
	}

	if (addr == 0xff76) {  //???
		return 0x00;
	}

	if (addr == 0xff77) {  //???
		return 0x00;
	}

	if (addr == 0xffff) {  //IE
		return (status.interrupt_enable_joypad << 4)
			| (status.interrupt_enable_serial << 3)
			| (status.interrupt_enable_timer << 2)
			| (status.interrupt_enable_stat << 1)
			| (status.interrupt_enable_vblank << 0);
	}

	return 0x00;
}

void z80::mmio_write(uint16_t addr, uint8_t data) {
	if (addr >= 0xc000 && addr <= 0xfdff) { wram[wram_addr(addr)] = data; return; }
	if (addr >= 0xff80 && addr <= 0xfffe) { hram[addr & 0x7f] = data; return; }

	if (addr == 0xff00) {  //JOYP
		status.p15 = data & 0x20;
		status.p14 = data & 0x10;
		//interface->joypWrite(status.p15, status.p14);
		return;
	}

	if (addr == 0xff01) {  //SB
		status.serial_data = data;
		return;
	}

	if (addr == 0xff02) {  //SC
		status.serial_transfer = data & 0x80;
		status.serial_clock = data & 0x01;
		if (status.serial_transfer) status.serial_bits = 8;
		return;
	}

	if (addr == 0xff04) {  //DIV
		status.div = 0;
		return;
	}

	if (addr == 0xff05) {  //TIMA
		status.tima = data;
		return;
	}

	if (addr == 0xff06) {  //TMA
		status.tma = data;
		return;
	}

	if (addr == 0xff07) {  //TAC
		status.timer_enable = data & 0x04;
		status.timer_clock = data & 0x03;
		return;
	}

	if (addr == 0xff0f) {  //IF
		status.interrupt_request_joypad = data & 0x10;
		status.interrupt_request_serial = data & 0x08;
		status.interrupt_request_timer = data & 0x04;
		status.interrupt_request_stat = data & 0x02;
		status.interrupt_request_vblank = data & 0x01;
		return;
	}

	if (addr == 0xff46) {  //DMA
		oamdma.active = true;
		oamdma.bank = data;
		oamdma.offset = 0;
		return;
	}

	if (addr == 0xff4d) {  //KEY1
		status.speed_switch = data & 0x01;
		return;
	}

	if (addr == 0xff51) {  //HDMA1
		status.dma_source = (status.dma_source & 0x00ff) | (data << 8);
		return;
	}

	if (addr == 0xff52) {  //HDMA2
		status.dma_source = (status.dma_source & 0xff00) | (data & 0xf0);
		return;
	}

	if (addr == 0xff53) {  //HDMA3
		status.dma_target = (status.dma_target & 0x00ff) | (data << 8);
		return;
	}

	if (addr == 0xff54) {  //HDMA4
		status.dma_target = (status.dma_target & 0xff00) | (data & 0xf0);
		return;
	}

	if (addr == 0xff55) {  //HDMA5
		status.dma_mode = data & 0x80;
		status.dma_length = ((data & 0x7f) + 1) * 16;
		status.dma_completed = !status.dma_mode;

		if (status.dma_mode == 0) {
			do {
				for (unsigned n = 0; n < 16; n++) {
					dma_write(status.dma_target++, dma_read(status.dma_source++));
				}
				add_clocks(8 << status.speed_double);
				status.dma_length -= 16;
			} while (status.dma_length);
		}
		return;
	}

	if (addr == 0xff56) {  //RP
		return;
	}

	if (addr == 0xff6c) {  //???
		status.ff6c = data & 0x01;
		return;
	}

	if (addr == 0xff72) {  //???
		status.ff72 = data;
		return;
	}

	if (addr == 0xff73) {  //???
		status.ff73 = data;
		return;
	}

	if (addr == 0xff74) {  //???
		status.ff74 = data;
		return;
	}

	if (addr == 0xff75) {  //???
		status.ff75 = data & 0x70;
		return;
	}

	if (addr == 0xff70) {  //SVBK
		status.wram_bank = data & 0x07;
		return;
	}

	if (addr == 0xffff) {  //IE
		status.interrupt_enable_joypad = data & 0x10;
		status.interrupt_enable_serial = data & 0x08;
		status.interrupt_enable_timer = data & 0x04;
		status.interrupt_enable_stat = data & 0x02;
		status.interrupt_enable_vblank = data & 0x01;
		return;
	}
}

//VRAM DMA source can only be ROM or RAM
uint8_t z80::dma_read(uint16_t addr) {
	if (addr < 0x8000) return MMU.rb(addr);  //0000-7fff
	if (addr < 0xa000) return 0x00;          //8000-9fff
	if (addr < 0xe000) return MMU.rb(addr);  //a000-dfff
	return 0x00;                             //e000-ffff
}

//VRAM DMA target is always VRAM
void z80::dma_write(uint16_t addr, uint8_t data) {
	addr = 0x8000 | (addr & 0x1fff);  //8000-9fff
	return MMU.wb(addr, data);
}

// TIMING:
void z80::add_clocks(unsigned clocks) {
	if (oamdma.active) {
		for (unsigned n = 0; n < 4 * clocks; n++) {
			MMU.wb(0xfe00 + oamdma.offset, MMU.rb((oamdma.bank << 8) + oamdma.offset));
			if (++oamdma.offset == 160) {
				oamdma.active = false;
				break;
			}
		}
	}

	//system.clocks_executed += clocks;
	//if (system.sgb()) scheduler.exit(Scheduler::ExitReason::StepEvent);

	status.clock += clocks;
	if (status.clock >= 4 * 1024 * 1024) {
		status.clock -= 4 * 1024 * 1024;
		//cartridge.mbc3.second();
	}

	//4MHz / N(hz) - 1 = mask
	if ((status.clock & 15) == 0) timer_262144hz();
	if ((status.clock & 63) == 0)  timer_65536hz();
	if ((status.clock & 255) == 0)  timer_16384hz();
	if ((status.clock & 511) == 0)   timer_8192hz();
	if ((status.clock & 1023) == 0)   timer_4096hz();

	/*GPU.clock -= clocks * GPU.frequency;
	if (ppu.clock < 0) co_switch(scheduler.active_thread = ppu.thread);

	apu.clock -= clocks * apu.frequency;
	if (apu.clock < 0) co_switch(scheduler.active_thread = apu.thread);*/
}

void z80::timer_262144hz() {
	if (status.timer_enable && status.timer_clock == 1) {
		if (++status.tima == 0) {
			status.tima = status.tma;
			interrupt_raise(Interrupt::Timer);
		}
	}
}

void z80::timer_65536hz() {
	if (status.timer_enable && status.timer_clock == 2) {
		if (++status.tima == 0) {
			status.tima = status.tma;
			interrupt_raise(Interrupt::Timer);
		}
	}
}

void z80::timer_16384hz() {
	if (status.timer_enable && status.timer_clock == 3) {
		if (++status.tima == 0) {
			status.tima = status.tma;
			interrupt_raise(Interrupt::Timer);
		}
	}

	status.div++;
}

void z80::timer_8192hz() {
	if (status.serial_transfer && status.serial_clock) {
		if (--status.serial_bits == 0) {
			status.serial_transfer = 0;
			interrupt_raise(Interrupt::Serial);
		}
	}
}

void z80::timer_4096hz() {
	if (status.timer_enable && status.timer_clock == 0) {
		if (++status.tima == 0) {
			status.tima = status.tma;
			interrupt_raise(Interrupt::Timer);
		}
	}
}

// ****************************************************************************
// Instruction Helpers:
// ****************************************************************************
inline void z80::InvalidOpCode(uint16_t OpCode) {
	printf("InvalidOpCode: %X at pc: %X\n", OpCode, pc);
}

inline uint8_t z80::READ(const uint16_t addr) {
	add_clocks(4);
	if (oamdma.active && (addr < 0xff80 || addr == 0xffff)) return 0x00;
	return MMU.rb(addr);
}

inline uint8_t z80::PC_READ() {
	return READ(pc++);
}

inline uint8_t z80::FF_READ(const uint8_t ffoffset) {
	return READ(0xFF00 | ffoffset);
}

inline void z80::WRITE(const uint16_t addr, const uint8_t data) {
	add_clocks(4);
	if (oamdma.active && (addr < 0xff80 || addr == 0xffff)) return;
	MMU.wb(addr, data);
}

inline void z80::FF_WRITE(const uint8_t ffoffset, const uint8_t data) {
	WRITE(0xFF00 + ffoffset, data);
}

inline void z80::PC_MOD(const uint16_t data) {
	pc = data;
	add_clocks(4);
}

inline void z80::PUSH(const uint8_t r1, const uint8_t r2) {
	sp = (sp - 1) & 0xFFFF;
	WRITE(sp, (r1));
	sp = (sp - 1) & 0xFFFF;
	WRITE(sp, (r2));
}

// ****************************************************************************
// 16-BIT LOADS:
// ****************************************************************************

// ld rr,nn (12 cycles)
// set rr to 16-bit value of next 2 bytes in memory
inline void z80::ld_rr_nn(uint8_t& r1, uint8_t& r2) {
	r2 = PC_READ();
	r1 = PC_READ();
}

// push rr (16 cycles):
// Push value of register pair onto stack:
inline void z80::push_rr(const uint8_t r1, const uint8_t r2) {
	PUSH(r1, r2);
	add_clocks(4);
} 

// pop rr (12 cycles):
// Pop two bytes off stack into register pair:
inline void z80::pop_rr(uint8_t& r1, uint8_t& r2) {
	r2 = READ(sp);
	sp = (sp + 1) & 0xFFFF;
	r1 = READ(sp);
	sp = (sp + 1) & 0xFFFF;
}

// 8-BIT ALU:

// add a,r (4 cycles):
// add a,(addr) (8 cycles):
// Add 8-bit value to A, check flags:
inline void z80::add_a_u8(const uint8_t u8) {
	f_ = 0;
	if (a_ + u8 > 0xFF) {
		setCFlag();
	}
	if ((a_ & 0xF) + (u8 & 0xF) > 0xF) {
		setHFlag();
	}
	a_ += u8;
	if (a_ == 0) {
		setZFlag();
	}
}

// adc a,r (4 cycles):
// adc a,(addr) (8 cycles):
// Add 8-bit value+CF to A, check flags:
inline void z80::adc_a_u8(const uint8_t u8) {
	int val = carryBit();
	f_ = 0;
	if (a_ + u8 + val > 0xFF) {
		setCFlag();
	}
	if ((a_ & 0xF) + (u8 & 0xF) + val > 0xF) {
		setHFlag();
	}
	a_ += u8 + val;
	if (a_ == 0) {
		setZFlag();
	}
}

// sub a,r (4 cycles):
// sub a,(addr) (8 cycles):
// Subtract 8-bit value from A, check flags:
inline void z80::sub_a_u8(const uint8_t u8) {
	f_ = FLAG_N;
	if (a_ < u8) {
		setCFlag();
	}
	if ((a_ & 0xF) < (u8 & 0xF)) {
		setHFlag();
	}
	a_ -= u8;
	if (a_ == 0) {
		setZFlag();
	}
}

// sbc a,r (4 cycles):
// sbc a,(addr) (8 cycles):
// Subtract CF and 8-bit value from A, check flags:
inline void z80::sbc_a_u8(const uint8_t u8) {
	int val2 = carryBit();
	f_ = FLAG_N;
	if (a_ < u8 + val2) {
		setCFlag();
	}
	if ((a_ & 0xF) < (u8 & 0xF) + val2) {
		setHFlag();
	}
	a_ -= (u8 + val2);
	if (a_ == 0) {
		setZFlag();
	}
}

// and a,r (4 cycles):
// and a,(addr) (8 cycles):
// bitwise and 8-bit value into A, check flags:
inline void z80::and_a_u8(const uint8_t u8) {
	f_ = FLAG_H;
	a_ &= u8;
	if (a_ == 0) {
		setZFlag();
	}
}

// or a,r (4 cycles):
// or a,(hl) (8 cycles):
// bitwise or 8-bit value into A, check flags:
inline void z80::or_a_u8(const uint8_t u8) {
	f_ = 0;
	a_ |= u8;
	if (a_ == 0) {
		setZFlag();
	}
}

// xor a,r (4 cycles):
// xor a,(hl) (8 cycles):
// bitwise xor 8-bit value into A, check flags:
inline void z80::xor_a_u8(const uint8_t u8) {
	f_ = 0;
	a_ ^= u8;
	if (a_ == 0) {
		setZFlag();
	}
}

// cp a,r (4 cycles):
// cp a,(addr) (8 cycles):
// Compare (subtract without storing result) 8-bit value to A, check flags:
inline void z80::cp_a_u8(const uint8_t u8) {
	f_ = FLAG_N;
	if (a_ < u8) {
		setCFlag();
	}
	if ((a_ & 0xF) < (u8 & 0xF)) {
		setHFlag();
	}
	if (a_ - u8 == 0) {
		setZFlag();
	}
}

// inc r (4 cycles):
// Increment value of 8-bit register, check flags except CF:
inline void z80::inc_r(uint8_t& r) {
	f_ &= FLAG_C;
	r++;
	if (r == 0) {
		setZFlag();
	}
	if ((r & 0xF) == 0) {
		setHFlag();
	}
}

// dec r (4 cycles):
// Decrement value of 8-bit register, check flags except CF:
inline void z80::dec_r(uint8_t& r) {
	f_ &= FLAG_C;
	r--;
	if (r == 0)
		setZFlag();
	if ((r & 0xF) == 0xF)
		setHFlag();
	setNFlag();
}

// ****************************************************************************
// 16-BIT ARITHMETIC:
// ****************************************************************************

// add hl,rr (8 cycles):
// add 16-bit register to HL, check flags except ZF:
inline void z80::add_hl_rr(const uint8_t rh, const uint8_t rl) {
	const unsigned tmp_hl = hl() + (rh << 8 | rl);
	f_ &= FLAG_Z;
	if (tmp_hl > 0xFFFF) {
		setCFlag();
	}
	if ((hl() & 0xFFF) + ((rh << 8 | rl) & 0xFFF) > 0xFFF) {
		setHFlag();
	}
	h_ = (tmp_hl >> 8) & 0xFF;
	l_ = tmp_hl & 0xFF;
	add_clocks(4);
}

// inc rr (8 cycles):
// Increment 16-bit register:
inline void z80::inc_rr(uint8_t& rh, uint8_t& rl) {
	const unsigned lowinc = rl + 1;
	rl = lowinc & 0xFF;
	rh = (rh + (lowinc >> 8)) & 0xFF;
	add_clocks(4);
}

// dec rr (8 cycles):
// Decrement 16-bit register:
inline void z80::dec_rr(uint8_t& rh, uint8_t& rl) {
	const unsigned lowdec = rl - 1;
	rl = lowdec & 0xFF;
	rh = (rh - (lowdec >> 8 & 1)) & 0xFF;
	add_clocks(4);
}

inline void z80::sp_plus_n(uint16_t& sumout) {
	f_ = 0;
	uint8_t val = PC_READ();
	val = (val ^ 0x80) - 0x80;

	if ((sp & 0xF) + (val & 0xF) > 0xF) {
		setHFlag();
	}
	if (((sp & 0xFF) + val) > 0xFF) {
		setCFlag();
	}
	sumout += ((int8_t)val & 0xFFFF);

	add_clocks(4);
}

// ****************************************************************************
// JUMPS:
// ****************************************************************************

// jp nn (16 cycles):
// Jump to address stored in the next two bytes in memory:
inline void z80::jp_nn() {
	const uint8_t imm0 = PC_READ();
	const uint8_t imm1 = PC_READ();
	PC_MOD(imm1 << 8 | imm0);
}

// jr disp (12 cycles):
// Jump to value of next (signed) byte in memory+current address:
inline void z80::jr_disp() {
	unsigned disp = PC_READ();
	disp = (disp ^ 0x80) - 0x80;
	PC_MOD((pc + disp) & 0xFFFF);
}

// ****************************************************************************
// CALLS, RESTARTS AND RETURNS:
// ****************************************************************************

// call nn (24 cycles):
// Jump to 16-bit immediate operand and push return address onto stack:
inline void z80::call_nn() {
	unsigned const npc = (pc + 2) & 0xFFFF;
	jp_nn();
	PUSH(npc >> 8, npc & 0xFF);
}

// rst n (16 Cycles):
// Push present address onto stack, jump to address n (one of 00h,08h,10h,18h,20h,28h,30h,38h):
inline void z80::rst_n(const uint8_t n) {
	PUSH(pc >> 8, pc & 0xFF);
	PC_MOD(n);
}

// ret (16 cycles):
// Pop two bytes from the stack and jump to that address:
inline void z80::ret() {
	uint8_t high, low;
	pop_rr(high, low);
	PC_MOD(high << 8 | low);
}

void z80::interrupt_raise(Interrupt id) {
	if (id == Interrupt::Vblank) {
		status.interrupt_request_vblank = 1;
		if (status.interrupt_enable_vblank) halt = false;
	}

	if (id == Interrupt::Stat) {
		status.interrupt_request_stat = 1;
		if (status.interrupt_enable_stat) halt = false;
	}

	if (id == Interrupt::Timer) {
		status.interrupt_request_timer = 1;
		if (status.interrupt_enable_timer) halt = false;
	}

	if (id == Interrupt::Serial) {
		status.interrupt_request_serial = 1;
		if (status.interrupt_enable_serial) halt = false;
	}

	if (id == Interrupt::Joypad) {
		status.interrupt_request_joypad = 1;
		if (status.interrupt_enable_joypad) halt = stop = false;
	}
}

void z80::interrupt_test() {
	if (ime) {
		if (status.interrupt_request_vblank && status.interrupt_enable_vblank) {
			status.interrupt_request_vblank = 0;
			return rst_n(0x0040);
		}

		if (status.interrupt_request_stat && status.interrupt_enable_stat) {
			status.interrupt_request_stat = 0;
			return rst_n(0x0048);
		}

		if (status.interrupt_request_timer && status.interrupt_enable_timer) {
			status.interrupt_request_timer = 0;
			return rst_n(0x0050);
		}

		if (status.interrupt_request_serial && status.interrupt_enable_serial) {
			status.interrupt_request_serial = 0;
			return rst_n(0x0058);
		}

		if (status.interrupt_request_joypad && status.interrupt_enable_joypad) {
			status.interrupt_request_joypad = 0;
			return rst_n(0x0060);
		}
	}
}

void z80::Process() {
	uint16_t oldpc = pc;
	uint8_t opcode = PC_READ();
	
	//printf("PC:%04x Op:%02x SP:%04x A:%02x B:%02x C:%02x D:%02x E:%02x H:%02x L:%02x F:%02x %s\n", oldpc, opcode, sp, a_, b_, c_, d_, e_, h_, l_, f_, halt?"skip":"");

	switch (opcode) {
	case 0x00: { // NOP [4]
		break;
	}
	case 0x01: { // LD BC,d16 [12]
		ld_rr_nn(b_, c_);
		break;
	}
	case 0x02: { // LD (BC),A [8]
		WRITE(bc(), a_);
		break;
	}
	case 0x03: { // INC BC [8]
		inc_rr(b_, c_);
		break;
	}
	case 0x04: { // INC B [4]
		inc_r(b_);
		break;
	}
	case 0x05: { // DEC B [4]
		dec_r(b_);
		break;
	}
	case 0x06: { // LD B,d8 [4]
		b_ = PC_READ();
		break;
	}
	case 0x07: { // RLCA [4]
		// Rotate 8-bit register A left, store old bit7 in CF. Reset SF, HCF, ZF:
		f_ = 0;
		const unsigned val = a_;
		a_ <<= 1;
		if (val & 0x80) {
			setCFlag();
			a_ |= 1;
		}
		break;
	}
	case 0x08: { // LD (a16),SP [20]
		const unsigned imml = PC_READ();
		const unsigned immh = PC_READ();
		const unsigned addr = immh << 8 | imml;

		WRITE(addr, sp & 0xFF);
		WRITE((addr + 1) & 0xFFFF, sp >> 8);
		break;
	}
	case 0x09: { // ADD HL,BC [8]
		add_hl_rr(b_, c_);
		break;
	}
	case 0x0A: { // LD A,(BC) [8]
		a_ = READ(bc());
		break;
	}
	case 0x0B: { // DEC BC [8]
		dec_rr(b_, c_);
		break;
	}
	case 0x0C: { // INC C [4]
		inc_r(c_);
		break;
	}
	case 0x0D: { // DEC C [4]
		dec_r(c_);
		break;
	}
	case 0x0E: { // LD C,d8 [8]
		c_ = PC_READ();
		break;
	}
	case 0x0F: { // RRCA [4]
		// Rotate 8-bit register A right, store old bit0 in CF. Reset SF, HCF, ZF:
		const unsigned val = a_;
		f_ = 0x00;
		a_ >>= 1;
		if (val & 1) {
			setCFlag();
			a_ |= 0x80;
		}
		break;
	}
	case 0x10: { // STOP 0 [4]
		// Halt CPU and LCD display until button pressed:
		// TODO - What the fuck is this garbage?
		int i = MMU.rb(pc);
		if (i>127) i = -((~i + 1) & 255);
		pc++;
		b_--;
		if (b_) {
			pc += i;
		}
		break;
	}
	case 0x11: { // LD DE,d16 [12]
		ld_rr_nn(d_, e_);
		break;
	}
	case 0x12: { // LD (DE),A [8]
		WRITE(de(), a_);
		break;
	}
	case 0x13: { // INC DE [8]
		inc_rr(d_, e_);
		break;
	}
	case 0x14: { // INC D [4]
		inc_r(d_);
		break;
	}
	case 0x15: { // DEC D [4]
		dec_r(d_);
		break;
	}
	case 0x16: { // LD D,d8 [8]
		d_ = PC_READ();
		break;
	}
	case 0x17: { // RLA [4]
		// Rotate 8-bit register A left through CF, store old bit7 in CF, old CF value becomes bit0. Reset SF, HCF, ZF:
		const unsigned val = (a_ & 0x80);
		a_ <<= 1;
		a_ |= carryBit();
		f_ = 0;
		if (val) {
			setCFlag();
		}
		break;
	}
	case 0x18: { // JR r8 [12]
		jr_disp();
		break;
	}
	case 0x19: { // ADD HL,DE [8]
		add_hl_rr(d_, e_);
		break;
	}
	case 0x1A: { // LD A,(DE) [8]
		a_ = READ(de());
		break;
	}
	case 0x1B: { // DEC DE [8]
		dec_rr(d_, e_);
		break;
	}
	case 0x1C: { // INC E [4]
		inc_r(e_);
		break;
	}
	case 0x1D: { // DEC E [4]
		dec_r(e_);
		break;
	}
	case 0x1E: { // LD E,d8 [8]
		e_ = PC_READ();
		break;
	}
	case 0x1F: { // RRA [4]
		// Rotate 8-bit register A right through CF, store old bit0 in CF, old CF value becomes bit7. Reset SF, HCF, ZF:
		int val = a_ & 1;
		a_ >>= 1;
		a_ |= (carryBit() << 7);
		f_ = 0;
		if (val) {
			setCFlag();
		}
		break;
	}
	case 0x20: { // JR NZ,r8 [12/8]
		// Jump to value of next (signed) byte in memory+current address if ZF is unset:
		if (f_ & FLAG_Z) {
			jr_disp();
		}
		else {
			PC_MOD((pc + 1) & 0xFFFF);
		}
		break;
	}
	case 0x21: { // LD HL,d16 [12]
		ld_rr_nn(h_, l_);
		break;
	}
	case 0x22: { // LD (HL+),A [8]
		// Put A into memory address in hl. Increment HL:
		unsigned addr = hl();
		WRITE(addr, a_);

		addr = (addr + 1) & 0xFFFF;
		l_ = addr;
		h_ = addr >> 8;
		break;
	}
	case 0x23: { // INC HL [8]
		inc_rr(h_, l_);
		break;
	}
	case 0x24: { // INC H [4]
		inc_r(h_);
		break;
	}
	case 0x25: { // DEC H [4]
		dec_r(h_);
		break;
	}
	case 0x26: { // LD H,d8 [8]
		h_ = PC_READ();
		break;
	}
	case 0x27: { // DAA [4]
		// Adjust register A to correctly represent a BCD. Check ZF, HF and CF:
		int tmpa = a_;

		if (negativeSet()) {
			if (halfSet() || (tmpa & 0xF) > 9) {
				tmpa += 0x06;
			}

			if (carrySet() || tmpa > 0x9F) {
				tmpa += 0x60;
			}
		}
		else {
			if (halfSet()) {
				tmpa = (tmpa - 6) & 0xFF;
			}

			if (carrySet()) {
				tmpa -= 0x60;
			}
		}

		f_ &= ~(FLAG_H | FLAG_Z);

		if ((tmpa & 0x100) == 0x100)
			setCFlag();

		tmpa &= 0xFF;

		if (tmpa == 0) {
			setZFlag();
		}

		a_ = tmpa;
		break;
	}
	case 0x28: { // JR Z,r8 [12/8]
		// Jump to value of next (signed) byte in memory+current address if ZF is set:
		if (!zeroSet()) {
			PC_MOD((pc + 1) & 0xFFFF);
		}
		else {
			jr_disp();
		}
		break;
	}
	case 0x29: { // ADD HL,HL [8]
		add_hl_rr(h_, l_);
		break;
	}
	case 0x2A: { // LD A,(HL+) [8]
		// Put value at address in hl into A. Increment HL:
		unsigned addr = hl();
		a_ = READ(addr);

		addr = (addr + 1) & 0xFFFF;
		l_ = addr;
		h_ = addr >> 8;
		break;
	}
	case 0x2B: { // DEC HL [8]
		dec_rr(h_, l_);
		break;
	}
	case 0x2C: { // INC L [4]
		inc_r(l_);
		break;
	}
	case 0x2D: { // DEC L [4]
		dec_r(l_);
		break;
	}
	case 0x2E: { // LD L,d8 [8]
		l_ = PC_READ();
		break;
	}
	case 0x2F: { // CPL [4]
		// Complement register A. (Flip all bits), set SF and HCF:
		setNFlag();
		setHFlag();
		a_ ^= 0xFF;
		break;
	}
	case 0x30: { // JR NC,r8 [12/8]
		// Jump to value of next (signed) byte in memory+current address if CF is unset:
		if (carrySet()) {
			PC_MOD((pc + 1) & 0xFFFF);
		}
		else {
			jr_disp();
		}
		break;
	}
	case 0x31: { // LD SP,d16 [12]
		// set sp to 16-bit value of next 2 bytes in memory
		const unsigned imml = PC_READ();
		const unsigned immh = PC_READ();
		sp = immh << 8 | imml;
		break;
	}
	case 0x32: { // LD (HL-),A [8]
		// Put A into memory address in hl. Decrement HL:
		unsigned addr = hl();
		WRITE(addr, a_);

		addr = (addr - 1) & 0xFFFF;
		l_ = addr;
		h_ = addr >> 8;

		break;
	}
	case 0x33: { // INC SP [8]
		sp = (sp + 1) & 0xFFFF;
		add_clocks(4);
		break;
	}
	case 0x34: { // INC (HL) [12]
		// Increment value at address in hl, check flags except CF:
		f_ &= FLAG_C;
		const uint8_t val = READ(hl()) + 1;
		WRITE(hl(), val);
		if (val == 0) {
			setZFlag();
		}
		if ((val & 0xF) == 0) {
			setHFlag();
		}
		break;
	}
	case 0x35: { // DEC (HL) [12]
		// Decrement value at address in hl, check flags except CF:
		f_ &= FLAG_C;
		uint8_t val = READ(hl()) - 1;
		WRITE(hl(), val);
		if (val == 0) {
			setZFlag();
		}
		if ((val & 0xF) == 0xF) {
			setHFlag();
		}
		setNFlag();
		break;
	}
	case 0x36: { // LD (HL),d8 [12]
		// set memory at address in hl to value of next byte in memory:
		WRITE(hl(), PC_READ());
		break;
	}
	case 0x37: { // SCF [4]
		// Set CF. Unset SF and HCF:
		setCFlag();
		clearNFlag();
		clearHFlag();
		break;
	}
	case 0x38: { // JR C,r8 [12/8]
		// Jump to value of next (signed) byte in memory+current address if CF is set:
		if (carrySet()) {
			jr_disp();
		}
		else {
			PC_MOD((pc + 1) & 0xFFFF);
		}
		break;
	}
	case 0x39: { // ADD HL,SP [8]
		// add SP to HL, check flags except ZF:
		f_ &= FLAG_Z;
		if (hl() + sp > 0xFFFF) {
			setCFlag();
		}
		if ((hl() & 0xFFF) + (sp & 0xFFF) > 0xFFF) {
			setHFlag();
		}
		uint16_t newhl = hl() + sp;
		l_ = newhl & 0xFF;
		h_ = newhl >> 8;
		add_clocks(4);
		break;
	}
	case 0x3A: { // LD A,(HL-) [8]
		// Put value at address in hl into A. Decrement HL:
		unsigned addr = hl();
		a_ = READ(addr);

		addr = (addr - 1) & 0xFFFF;
		l_ = addr;
		h_ = addr >> 8;
		break;
	}
	case 0x3B: { // DEC SP [8]
		sp = (sp - 1) & 0xFFFF;
		add_clocks(4);
		break;
	}
	case 0x3C: { // INC A [4]
		inc_r(a_);
		break;
	}
	case 0x3D: { // DEC A [4]
		dec_r(a_);
		break;
	}
	case 0x3E: { // LD A,d8 [8]
		a_ = PC_READ();
		break;
	}
	case 0x3F: { // CCF [4]
		// Complement CF (unset if set vv.) Unset SF and HCF.
		if (carrySet()) {
			clearCFlag();
		}
		else {
			setCFlag();
		}
		clearNFlag();
		clearHFlag();
		break;
	}
	case 0x40: { // LD B,B [4]
		b_ = b_;
		break;
	}
	case 0x41: { // LD B,C [4]
		b_ = c_;
		break;
	}
	case 0x42: { // LD B,D [4]
		b_ = d_;
		break;
	}
	case 0x43: { // LD B,E [4]
		b_ = e_;
		break;
	}
	case 0x44: { // LD B,H [4]
		b_ = h_;
		break;
	}
	case 0x45: { // LD B,L [4]
		b_ = l_;
		break;
	}
	case 0x46: { // LD B,(HL) [8]
		b_ = READ(hl());
		break;
	}
	case 0x47: { // LD B,A [4]
		b_ = a_;
		break;
	}
	case 0x48: { // LD C,B [4]
		c_ = b_;
		break;
	}
	case 0x49: { // LD C,C [4]
		c_ = c_;
		break;
	}
	case 0x4A: { // LD C,D [4]
		c_ = d_;
		break;
	}
	case 0x4B: { // LD C,E [4]
		c_ = e_;
		break;
	}
	case 0x4C: { // LD C,H [4]
		c_ = h_;
		break;
	}
	case 0x4D: { // LD C,L [4]
		c_ = l_;
		break;
	}
	case 0x4E: { // LD C,(HL) [8]
		c_ = READ(hl());
		break;
	}
	case 0x4F: { // LD C,A [4]
		c_ = a_;
		break;
	}
	case 0x50: { // LD D,B [4]
		d_ = b_;
		break;
	}
	case 0x51: { // LD D,C [4]
		d_ = c_;
		break;
	}
	case 0x52: { // LD D,D [4]
		d_ = d_;
		break;
	}
	case 0x53: { // LD D,E [4]
		d_ = e_;
		break;
	}
	case 0x54: { // LD D,H [4]
		d_ = h_;
		break;
	}
	case 0x55: { // LD D,L [4]
		d_ = l_;
		break;
	}
	case 0x56: { // LD D,(HL) [8]
		d_ = READ(hl());
		break;
	}
	case 0x57: { // LD D,A [4]
		d_ = a_;
		break;
	}
	case 0x58: { // LD E,B [4]
		e_ = b_;
		break;
	}
	case 0x59: { // LD E,C [4]
		e_ = c_;
		break;
	}
	case 0x5A: { // LD E,D [4]
		e_ = d_;
		break;
	}
	case 0x5B: { // LD E,E [4]
		e_ = e_;
		break;
	}
	case 0x5C: { // LD E,H [4]
		e_ = h_;
		break;
	}
	case 0x5D: { // LD E,L [4]
		e_ = l_;
		break;
	}
	case 0x5E: { // LD E,(HL) [8]
		e_ = READ(hl());
		break;
	}
	case 0x5F: { // LD E,A [4]
		e_ = a_;
		break;
	}
	case 0x60: { // LD H,B [4]
		h_ = b_;
		break;
	}
	case 0x61: { // LD H,C [4]
		h_ = c_;
		break;
	}
	case 0x62: { // LD H,D [4]
		h_ = d_;
		break;
	}
	case 0x63: { // LD H,E [4]
		h_ = e_;
		break;
	}
	case 0x64: { // LD H,H [4]
		h_ = h_;
		break;
	}
	case 0x65: { // LD H,L [4]
		h_ = l_;
		break;
	}
	case 0x66: { // LD H,(HL) [8]
		h_ = READ(hl());
		break;
	}
	case 0x67: { // LD H,A [4]
		h_ = a_;
		break;
	}
	case 0x68: { // LD L,B [4]
		l_ = b_;
		break;
	}
	case 0x69: { // LD L,C [4]
		l_ = c_;
		break;
	}
	case 0x6A: { // LD L,D [4]
		l_ = d_;
		break;
	}
	case 0x6B: { // LD L,E [4]
		l_ = e_;
		break;
	}
	case 0x6C: { // LD L,H [4]
		l_ = h_;
		break;
	}
	case 0x6D: { // LD L,L [4]
		l_ = l_;
		break;
	}
	case 0x6E: { // LD L,(HL) [8]
		l_ = READ(hl());
		break;
	}
	case 0x6F: { // LD L,A [4]
		l_ = a_;
		break;
	}
	case 0x70: { // LD (HL),B [8]
		WRITE(hl(), b_);
		break;
	}
	case 0x71: { // LD (HL),C [8]
		WRITE(hl(), c_);
		break;
	}
	case 0x72: { // LD (HL),D [8]
		WRITE(hl(), d_);
		break;
	}
	case 0x73: { // LD (HL),E [8]
		WRITE(hl(), e_);
		break;
	}
	case 0x74: { // LD (HL),H [8]
		WRITE(hl(), h_);
		break;
	}
	case 0x75: { // LD (HL),L [8]
		WRITE(hl(), l_);
		break;
	}
	case 0x76: { // HALT
		// Halt until interrupt occurs (low power):
		halt = 1;
		break;
	}
	case 0x77: { // LD (HL),A [8]
		WRITE(hl(), a_);
		break;
	}
	case 0x78: { // LD A,B [4]
		a_ = b_;
		break;
	}
	case 0x79: { // LD A,C [4]
		a_ = c_;
		break;
	}
	case 0x7A: { // LD A,D [4]
		a_ = d_;
		break;
	}
	case 0x7B: { // LD A,E [4]
		a_ = e_;
		break;
	}
	case 0x7C: { // LD A,H [4]
		a_ = h_;
		break;
	}
	case 0x7D: { // LD A,L [4]
		a_ = l_;
		break;
	}
	case 0x7E: { // LD A,(HL) [8]
		a_ = READ(hl());
		break;
	}
	case 0x7F: { // LD A,A [4]
		a_ = a_;
		break;
	}
	case 0x80: { // ADD A,B [4]
		add_a_u8(b_);
		break;
	}
	case 0x81: { // ADD A,C [4]
		add_a_u8(c_);
		break;
	}
	case 0x82: { // ADD A,D [4]
		add_a_u8(d_);
		break;
	}
	case 0x83: { // ADD A,E [4]
		add_a_u8(e_);
		break;
	}
	case 0x84: { // ADD A,H [4]
		add_a_u8(h_);
		break;
	}
	case 0x85: { // ADD A,L [4]
		add_a_u8(l_);
		break;
	}
	case 0x86: { // ADD A,(HL) [8]
		add_a_u8(READ(hl()));
		break;
	}
	case 0x87: { // ADD A,A [4]
		add_a_u8(a_);
		break;
	}
	case 0x88: { // ADC A,B [4]
		adc_a_u8(b_);
		break;
	}
	case 0x89: { // ADC A,C [4]
		adc_a_u8(c_);
		break;
	}
	case 0x8A: { // ADC A,D [4]
		adc_a_u8(d_);
		break;
	}
	case 0x8B: { // ADC A,E [4]
		adc_a_u8(e_);
		break;
	}
	case 0x8C: { // ADC A,H [4]
		adc_a_u8(h_);
		break;
	}
	case 0x8D: { // ADC A,L [4]
		adc_a_u8(l_);
		break;
	}
	case 0x8E: { // ADC A,(HL) [8]
		adc_a_u8(READ(hl()));
		break;
	}
	case 0x8F: { // ADC A,A [4]
		adc_a_u8(a_);
		break;
	}
	case 0x90: { // SUB B [4]
		sub_a_u8(b_);
		break;
	}
	case 0x91: { // SUB C [4]
		sub_a_u8(c_);
		break;
	}
	case 0x92: { // SUB D [4]
		sub_a_u8(d_);
		break;
	}
	case 0x93: { // SUB E [4]
		sub_a_u8(e_);
		break;
	}
	case 0x94: { // SUB H [4]
		sub_a_u8(h_);
		break;
	}
	case 0x95: { // SUB L [4]
		sub_a_u8(l_);
		break;
	}
	case 0x96: { // SUB (HL) [8]
		sub_a_u8(READ(hl()));
		break;
	}
	case 0x97: { // SUB A [4]
		// A-A is always 0:
		a_ = 0;
		clearCFlag();
		clearHFlag();
		setZFlag();
		setNFlag();
		break;
	}
	case 0x98: { // SBC A,B [4]
		sbc_a_u8(b_);
		break;
	}
	case 0x99: { // SBC A,C [4]
		sbc_a_u8(c_);
		break;
	}
	case 0x9A: { // SBC A,D [4]
		sbc_a_u8(d_);
		break;
	}
	case 0x9B: { // SBC A,E [4]
		sbc_a_u8(e_);
		break;
	}
	case 0x9C: { // SBC A,H [4]
		sbc_a_u8(h_);
		break;
	}
	case 0x9D: { // SBC A,L [4]
		sbc_a_u8(l_);
		break;
	}
	case 0x9E: { // SBC A,(HL) [8]
		sbc_a_u8(READ(hl()));
		break;
	}
	case 0x9F: { // SBC A,A [4]
		sbc_a_u8(a_);
		break;
	}
	case 0xA0: { // AND B [4]
		and_a_u8(b_);
		break;
	}
	case 0xA1: { // AND C [4]
		and_a_u8(c_);
		break;
	}
	case 0xA2: { // AND D [4]
		and_a_u8(d_);
		break;
	}
	case 0xA3: { // AND E [4]
		and_a_u8(e_);
		break;
	}
	case 0xA4: { // AND H [4]
		and_a_u8(h_);
		break;
	}
	case 0xA5: { // AND L [4]
		and_a_u8(l_);
		break;
	}
	case 0xA6: { // AND (HL) [8]
		and_a_u8(READ(hl()));
		break;
	}
	case 0xA7: { // AND A [4]
		// A&A will always be A:
		f_ = FLAG_H;
		if (a_ == 0) {
			setZFlag();
		}
		break;
	}
	case 0xA8: { // XOR B [4]
		xor_a_u8(b_);
		break;
	}
	case 0xA9: { // XOR C [4]
		xor_a_u8(c_);
		break;
	}
	case 0xAA: { // XOR D [4]
		xor_a_u8(d_);
		break;
	}
	case 0xAB: { // XOR E [4]
		xor_a_u8(e_);
		break;
	}
	case 0xAC: { // XOR H [4]
		xor_a_u8(h_);
		break;
	}
	case 0xAD: { // XOR L [4]
		xor_a_u8(l_);
		break;
	}
	case 0xAE: { // XOR (HL) [8]
		xor_a_u8(READ(hl()));
		break;
	}
	case 0xAF: { // XOR A [4]
		// A^A will always be 0:
		f_ = FLAG_Z;
		a_ = 0;
		break;
	}
	case 0xB0: { // OR B [4]
		or_a_u8(b_);
		break;
	}
	case 0xB1: { // OR C [4]
		or_a_u8(c_);
		break;
	}
	case 0xB2: { // OR D [4]
		or_a_u8(d_);
		break;
	}
	case 0xB3: { // OR E [4]
		or_a_u8(e_);
		break;
	}
	case 0xB4: { // OR H [4]
		or_a_u8(h_);
		break;
	}
	case 0xB5: { // OR L [4]
		or_a_u8(l_);
		break;
	}
	case 0xB6: { // OR (HL) [8]
		or_a_u8(READ(hl()));
		break;
	}
	case 0xB7: { // OR A [4]
		// A|A will always be A:
		f_ = 0;
		if (a_ == 0) {
			setZFlag();
		}
		break;
	}
	case 0xB8: { // CP B [4]
		cp_a_u8(b_);
		break;
	}
	case 0xB9: { // CP C [4]
		cp_a_u8(c_);
		break;
	}
	case 0xBA: { // CP D [4]
		cp_a_u8(d_);
		break;
	}
	case 0xBB: { // CP E [4]
		cp_a_u8(e_);
		break;
	}
	case 0xBC: { // CP H [4]
		cp_a_u8(h_);
		break;
	}
	case 0xBD: { // CP L [4]
		cp_a_u8(l_);
		break;
	}
	case 0xBE: { // CP (HL) [8]
		cp_a_u8(READ(hl()));
		break;
	}
	case 0xBF: { // CP A [4]
		// A always equals A:
		clearCFlag();
		clearHFlag();
		setZFlag();
		setNFlag();
		break;
	}
	case 0xC0: { // RET NZ [20/8]
		// Pop two bytes from the stack and jump to that address, if ZF is unset:
		add_clocks(4);

		if (!zeroSet()) {
			ret();
		}
		break;
	}
	case 0xC1: { // POP BC [12]
		pop_rr(b_, c_);
		break;
	}
	case 0xC2: { // JP NZ,a16 [16/12]
		// Jump to address stored in next two bytes in memory if ZF is unset:
		if (!zeroSet()) {
			jp_nn();
		}
		else {
			PC_MOD((pc + 2) & 0xFFFF);
			add_clocks(4);
		}
		break;
	}
	case 0xC3: { // JP a16 [16]
		jp_nn();
		break;
	}
	case 0xC4: { // CALL NZ,a16 [24/12]
		// Push address of next instruction onto stack and then jump to
		// address stored in next two bytes in memory, if ZF is unset:
		if (!zeroSet()) {
			call_nn();
		}
		else {
			PC_MOD((pc + 2) & 0xFFFF);
			add_clocks(4);
		}
		break;
	}
	case 0xC5: { // PUSH BC [16]
		push_rr(b_, c_);
		break;
	}
	case 0xC6: { // ADD A,d8 [8]
		add_a_u8(PC_READ());
		break;
	}
	case 0xC7: { // RST 00H [16]
		rst_n(0x00);
		break;
	}
	case 0xC8: { // RET Z [20/8]
		// Pop two bytes from the stack and jump to that address, if ZF is set:
		add_clocks(4);

		if (zeroSet()) {
			ret();
		}
		break;
	}
	case 0xC9: { // RET [16]
		ret();
		break;
	}
	case 0xCA: { // JP Z,a16 [16/12]
		// Jump to address stored in next two bytes in memory if ZF is set:
		if (zeroSet()) {
			jp_nn();
		}
		else {
			PC_MOD((pc + 2) & 0xFFFF);
			add_clocks(4);
		}
		break;
	}
	case 0xCB: { // PREFIX CB [4]
		// CB OPCODES (Shifts, rotates and bits):
		ProcessCB();
		break;
	}
	case 0xCC: { // CALL Z,a16 [24/12]
		// Push address of next instruction onto stack and then jump to
		// address stored in next two bytes in memory, if ZF is set:
		if (zeroSet()) {
			call_nn();
		}
		else {
			PC_MOD((pc + 2) & 0xFFFF);
			add_clocks(4);
		}
		break;
	}
	case 0xCD: { // CALL a16 [24]
		call_nn();
		break;
	}
	case 0xCE: { // ADC A,d8 [8]
		adc_a_u8(PC_READ());
		break;
	}
	case 0xCF: { // RST 08H [16]
		rst_n(0x08);
		break;
	}
	case 0xD0: { // RET NC [20/8]
		// Pop two bytes from the stack and jump to that address, if CF is unset:
		add_clocks(4);

		if (!carrySet()) {
			ret();
		}

		break;
	}
	case 0xD1: { // POP DE [12]
		pop_rr(d_, e_);
		break;
	}
	case 0xD2: { // JP NC,a16
		// Jump to address stored in next two bytes in memory if CF is unset:
		if (!carrySet()) {
			jp_nn();
		}
		else {
			PC_MOD((pc + 2) & 0xFFFF);
			add_clocks(4);
		}
		break;
	}
	case 0xD3: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xD4: { // CALL NC,a16 [24/12]
		// Push address of next instruction onto stack and then jump to
		// address stored in next two bytes in memory, if CF is unset:
		if (!carrySet()) {
			call_nn();
		}
		else {
			PC_MOD((pc + 2) & 0xFFFF);
			add_clocks(4);
		}
		break;
	}
	case 0xD5: { // PUSH DE [16]
		push_rr(d_, e_);
		break;
	}
	case 0xD6: { // SUB d8 [8]
		sub_a_u8(PC_READ());
		break;
	}
	case 0xD7: { // RST 10H [16]
		rst_n(0x10);
		break;
	}
	case 0xD8: { // RET C [20/8]
		// Pop two bytes from the stack and jump to that address, if CF is set:
		add_clocks(4);

		if (carrySet()) {
			ret();
		}

		break;
	}
	case 0xD9: { // RETI [16]
		// Pop two bytes from the stack and jump to that address, then enable interrupts:
		uint8_t sl, sh;
		pop_rr(sh, sl);
		PC_MOD(sh << 8 | sl);
		ime = 1;
		break;
	}
	case 0xDA: { // JP C,a16 [16/12]
		// Jump to address stored in next two bytes in memory if CF is set:
		if (carrySet()) {
			jp_nn();
		}
		else {
			PC_MOD((pc + 2) & 0xFFFF);
			add_clocks(4);
		}
		break;
	}
	case 0xDB: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xDC: { // CALL C,a16 [24/12]
		// Push address of next instruction onto stack and then jump to
		// address stored in next two bytes in memory, if CF is set:
		if (carrySet()) {
			call_nn();
		}
		else {
			PC_MOD((pc + 2) & 0xFFFF);
			add_clocks(4);
		}
		break;
	}
	case 0xDD: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xDE: { // SBC A,d8 [8]
		sbc_a_u8(PC_READ());
		break;
	}
	case 0xDF: { // RST 18H [16]
		rst_n(0x18);
		break;
	}
	case 0xE0: { // LDH (a8),A [12]
		// Put value in A into address (0xFF00 + next byte in memory):
		FF_WRITE(PC_READ(), a_);
		break;
	}
	case 0xE1: { // POP HL [12]
		pop_rr(h_, l_);
		break;
	}
	case 0xE2: { // LD (C),A [8]
		// Put A into address (0xFF00 + register C):
		FF_WRITE(c_, a_);
		break;
	}
	case 0xE3: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xE4: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xE5: { // PUSH HL [16]
		push_rr(h_, l_);
		break;
	}
	case 0xE6: { // AND d8 [8]
		and_a_u8(PC_READ());
		break;
	}
	case 0xE7: { // RST 20H [16]
		rst_n(0x20);
		break;
	}
	case 0xE8: { // ADD SP,r8 [16]
		// Add next (signed) byte in memory to SP, reset ZF and SF, check HCF and CF:
		sp_plus_n(sp);
		break;
	}
	case 0xE9: { // JP (HL) [4]
		// Jump to address in hl:
		pc = hl();
		break;
	}
	case 0xEA: { // LD (a16),A [16]
		// set memory at address given by the next 2 bytes to value in A:
		// Incrementing PC before call, because of possible interrupt.
		const uint8_t imml = PC_READ();
		const uint8_t immh = PC_READ();
		WRITE(immh << 8 | imml, a_);
		break;
	}
	case 0xEB: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xEC: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xED: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xEE: { // XOR d8 [8]
		xor_a_u8(PC_READ());
		break;
	}
	case 0xEF: { // RST 28H [16]
		rst_n(0x28);
		break;
	}
	case 0xF0: { // LDH A,(a8) [12]
		// Put value at address (0xFF00 + next byte in memory) into A:
		a_ = FF_READ(PC_READ());
		break;
	}
	case 0xF1: { // POP AF [12]
		pop_rr(a_, f_);
		break;
	}
	case 0xF2: { // LD A,(C) [8]
		// Put value at address (0xFF00 + register C) into A:
		a_ = FF_READ(c_);
		break;
	}
	case 0xF3: { // DI [4]
		ime = 0;
		break;
	}
	case 0xF4: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xF5: { // PUSH AF [16]
		push_rr(a_, f_);
		break;
	}
	case 0xF6: { // OR d8 [8]
		or_a_u8(PC_READ());
		break;
	}
	case 0xF7: { // RST 30H [16]
		rst_n(0x30);
		break;
	}
	case 0xF8: { // LD HL,SP+r8 [12]
		// Put (sp+next (signed) byte in memory) into hl (unsets ZF and SF, may enable HF and CF):
		uint16_t sum;
		sp_plus_n(sum);
		l_ = sum & 0xFF;
		h_ = sum >> 8;
		break;
	}
	case 0xF9: { // LD SP,HL [8]
		// Put value in HL into SP
		sp = hl();
		add_clocks(4);
		break;
	}
	case 0xFA: { // LD A,(a16) [16]
		// set A to value in memory at address given by the 2 next bytes.
		const uint8_t imml = PC_READ();
		const uint8_t immh = PC_READ();
		a_ = READ(immh << 8 | imml);
		break;
	}
	case 0xFB: { // EI [4]
		ime = 1;
		break;
	}
	case 0xFC: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xFD: { // XX
		InvalidOpCode(opcode);
		break;
	}
	case 0xFE: { // CP d8 [8]
		cp_a_u8(PC_READ());
		break;
	}
	case 0xFF: { // RST 38H [16]
		rst_n(0x38);
		break;
	}
	}
}

// ****************************************************************************
// CB OPCODES (Shifts, rotates and bits):
// ****************************************************************************
// swap r (8 cycles):
// Swap upper and lower nibbles of 8-bit register, reset flags, check zero flag:
inline void z80::swap_r(uint8_t& r) {
	f_ = 0;
	uint8_t r2 = r;
	uint8_t val = r2 >> 4;
	r2 <<= 4;
	r2 |= val;
	if (r2 == 0) {
		setZFlag();
	}
	r = r2;
}


// rlc r (8 cycles):
// Rotate 8-bit register left, store old bit7 in CF. Reset SF and HCF, Check ZF:
inline void z80::rlc_r(uint8_t& r) {
	f_ = 0;
	uint8_t r2 = r;
	r2 <<= 1;
	if ((r & 0x80) != 0) {
		setCFlag();
		r2 |= 1;
	}
	if (r2 == 0) {
		setZFlag();
	}
	r = r2;
}


// rl r (8 cycles):
// Rotate 8-bit register left through CF, store old bit7 in CF, old CF value becomes bit0. Reset SF and HCF, Check ZF:
inline void z80::rl_r(uint8_t& r) {
	uint8_t r2 = r;
	const uint8_t val = (r2 & 0x80);
	r2 <<= 1;
	r2 |= carryBit();
	f_ = 0;
	if (val) {
		setCFlag();
	}
	if (r2 == 0) {
		setZFlag();
	}
	r = r2;
}


// rrc r (8 cycles):
// Rotate 8-bit register right, store old bit0 in CF. Reset SF and HCF, Check ZF:
inline void z80::rrc_r(uint8_t& r) {
	f_ = 0;
	uint8_t r2 = r;
	const uint8_t val = r2;
	r2 >>= 1;
	if (val & 1) {
		setCFlag();
		r2 |= 0x80;
	}
	if (r2 == 0) {
		setZFlag();
	}
	r = r2;
}

// rr r (8 cycles):
// Rotate 8-bit register right through CF, store old bit0 in CF, old CF value becomes bit7. Reset SF and HCF, Check ZF:
inline void z80::rr_r(uint8_t& r) {
	uint8_t r2 = r;
	int val = r2 & 1;
	r2 >>= 1;
	r2 |= carryBit() << 7;
	f_ = 0;
	if (val) {
		setCFlag();
	}
	if (r2 == 0) {
		setZFlag();
	}
	r = r2;
}

// sla r (8 cycles):
// Shift 8-bit register left, store old bit7 in CF. Reset SF and HCF, Check ZF:
inline void z80::sla_r(uint8_t& r) {
	f_ = 0;
	uint8_t r2 = r;
	int val = (r2 & 0x80);
	r2 <<= 1;
	if (val) {
		setCFlag();
	}
	if (r2 == 0) {
		setZFlag();
	}
	r = r2;
}

// sra r (8 cycles):
// Shift 8-bit register right, store old bit0 in CF. bit7=old bit7. Reset SF and HCF, Check ZF:
inline void z80::sra_r(uint8_t& r) {
	f_ = 0;
	uint8_t r2 = r;
	if (r2 & 1) {
		setCFlag();
	}
	r2 >>= 1;
	if (r2 & 0x40) {
		r2 |= 0x80;
	}
	if (r2 == 0) {
		setZFlag();
	}
	r = r2;
}


// srl r (8 cycles):
// Shift 8-bit register right, store old bit0 in CF. Reset SF and HCF, Check ZF:
inline void z80::srl_r(uint8_t& r) {
	f_ = 0;
	uint8_t r2 = r;
	if (r2 & 1) {
		setCFlag();
	}
	r2 >>= 1;
	if (r2 == 0) {
		setZFlag();
	}
	r = r2;
}


// bit n,r (8 cycles):
// bit n,(hl) (12 cycles):
// Test bitn in 8-bit value, check ZF, unset SF, set HCF:
inline void z80::bitn_u8(const uint8_t bitmask, const uint8_t u8) {
	if ((u8 & bitmask) == 0) {
		setZFlag();
	}
	else {
		clearZFlag();
	}
	clearNFlag();
	setHFlag();
}

#define bit0_u8(u8) bitn_u8(0x01, (u8))
#define bit1_u8(u8) bitn_u8(0x02, (u8))	
#define bit2_u8(u8) bitn_u8(0x04, (u8))
#define bit3_u8(u8) bitn_u8(0x08, (u8))
#define bit4_u8(u8) bitn_u8(0x10, (u8))
#define bit5_u8(u8) bitn_u8(0x20, (u8))
#define bit6_u8(u8) bitn_u8(0x40, (u8))
#define bit7_u8(u8) bitn_u8(0x80, (u8))

// set n,r (8 cycles):
// Set bitn of 8-bit register:
#define set0_r(r) ( (r) |= 0x01 )
#define set1_r(r) ( (r) |= 0x02 )
#define set2_r(r) ( (r) |= 0x04 )
#define set3_r(r) ( (r) |= 0x08 )
#define set4_r(r) ( (r) |= 0x10 )
#define set5_r(r) ( (r) |= 0x20 )
#define set6_r(r) ( (r) |= 0x40 )
#define set7_r(r) ( (r) |= 0x80 )

// set n,(hl) (16 cycles):
// Set bitn of value at address stored in HL:
inline void z80::setn_mem_hl(const uint8_t n) {
	uint8_t val = READ(hl());
	val |= 1 << (n);
	WRITE(hl(), val);
}

// res n,r (8 cycles):
// Unset bitn of 8-bit register:
#define res0_r(r) ( (r) &= 0xFE )
#define res1_r(r) ( (r) &= 0xFD )
#define res2_r(r) ( (r) &= 0xFB )
#define res3_r(r) ( (r) &= 0xF7 )
#define res4_r(r) ( (r) &= 0xEF )
#define res5_r(r) ( (r) &= 0xDF )
#define res6_r(r) ( (r) &= 0xBF )
#define res7_r(r) ( (r) &= 0x7F )

// res n,(hl) (16 cycles):
// Unset bitn of value at address stored in HL:
inline void z80::resn_mem_hl(const uint8_t n) {
	uint8_t val = READ(hl());
	val &= ~(1 << (n));
	WRITE(hl(), val);
}

inline void z80::ProcessCB() {
	uint8_t opcode = PC_READ();

	switch (opcode) {
	case 0x00: rlc_r(b_); break; // CB RLC B 
	case 0x01: rlc_r(c_); break; // CB RLC C 
	case 0x02: rlc_r(d_); break; // CB RLC D 
	case 0x03: rlc_r(e_); break; // CB RLC E 
	case 0x04: rlc_r(h_); break; // CB RLC H 
	case 0x05: rlc_r(l_); break; // CB RLC L 
	case 0x06: { // CB RLC (HL)
		// Rotate 8-bit value stored at address in HL left, store old bit7 in CF.
		// Reset SF and HCF. Check ZF:
		f_ = 0;
		const uint8_t val = READ(hl());
		uint8_t val2 = val;
		val2 <<= 1;
		if ((val & 0x80) != 0) {
			setCFlag();
			val2 |= 1;
		}
		if (val2 == 0) {
			setZFlag();
		}
		WRITE(hl(), val2);
		break;
	}
	case 0x07: rlc_r(a_); break; // CB RLC A 

	case 0x08: rrc_r(b_); break; // CB RRC B
	case 0x09: rrc_r(c_); break; // CB RRC C 
	case 0x0A: rrc_r(d_); break; // CB RRC D 
	case 0x0B: rrc_r(e_); break; // CB RRC E 
	case 0x0C: rrc_r(h_); break; // CB RRC H 
	case 0x0D: rrc_r(l_); break; // CB RRC L 
	case 0x0E: { // CB RRC (HL) 
		// Rotate 8-bit value stored at address in HL right, store old bit0 in CF.
		// Reset SF and HCF. Check ZF:
		f_ = 0;
		const uint8_t val = READ(hl());
		uint8_t val2 = val;
		val2 >>= 1;
		if ((val & 1) != 0) {
			setCFlag();
			val2 |= 0x80;
		}
		if (val2 == 0) {
			setZFlag();
		}
		WRITE(hl(), val2);
		break;
	}
	case 0x0F: rrc_r(a_); break; // CB RRC A 

	case 0x10: rl_r(b_); break; // CB RL B 
	case 0x11: rl_r(c_); break; // CB RL C 
	case 0x12: rl_r(d_); break; // CB RL D 
	case 0x13: rl_r(e_); break; // CB RL E 
	case 0x14: rl_r(h_); break; // CB RL H 
	case 0x15: rl_r(l_); break; // CB RL L 
	case 0x16: { // CB RL (HL) 
		// Rotate 8-bit value stored at address in HL left thorugh CF,
		// store old bit7 in CF, old CF value becoms bit0. Reset SF and HCF. Check ZF:
		uint8_t val2 = READ(hl());
		const uint8_t val = (val2 & 0x80);
		val2 <<= 1;
		val2 |= carryBit();
		f_ = 0;
		if (val) {
			setCFlag();
		}
		if (val2 == 0) {
			setZFlag();
		}
		WRITE(hl(), val2);
		break;
	}
	case 0x17: rl_r(a_); break; // CB RL A 

	case 0x18: rr_r(b_); break; // CB RR B 
	case 0x19: rr_r(c_); break; // CB RR C 
	case 0x1A: rr_r(d_); break; // CB RR D 
	case 0x1B: rr_r(e_); break; // CB RR E 
	case 0x1C: rr_r(h_); break; // CB RR H 
	case 0x1D: rr_r(l_); break; // CB RR L 
	case 0x1E: { // CB RR (HL) 
		// Rotate 8-bit value stored at address in HL right thorugh CF,
		// store old bit0 in CF, old CF value becoms bit7. Reset SF and HCF. Check ZF:
		uint8_t val2 = READ(hl());
		const uint8_t val = val2 & 1;
		val2 >>= 1;
		val2 |= carryBit() << 7;
		f_ = 0;
		if (val) {
			setCFlag();
		}
		if (val2 == 0) {
			setZFlag();
		}
		WRITE(hl(), val2);
		break;
	}
	case 0x1F: rr_r(a_); break; // CB RR A 

	case 0x20: sla_r(b_); break; // CB SLA B 
	case 0x21: sla_r(c_); break; // CB SLA C 
	case 0x22: sla_r(d_); break; // CB SLA D 
	case 0x23: sla_r(e_); break; // CB SLA E 
	case 0x24: sla_r(h_); break; // CB SLA H 
	case 0x25: sla_r(l_); break; // CB SLA L 
	case 0x26: { // CB SLA (HL) 
		// Shift 8-bit value stored at address in HL left, store old bit7 in CF.
		// Reset SF and HCF. Check ZF:
		f_ = 0;
		uint8_t val2 = READ(hl());
		const uint8_t val = (val2 & 0x80);
		val2 <<= 1;
		if (val) {
			setCFlag();
		}
		if (val2 == 0) {
			setZFlag();
		}
		WRITE(hl(), val2);
		break;
	}
	case 0x27: sla_r(a_); break; // CB SLA A 

	case 0x28: sra_r(b_); break; // CB SRA B 
	case 0x29: sra_r(c_); break; // CB SRA C 
	case 0x2A: sra_r(d_); break; // CB SRA D 
	case 0x2B: sra_r(e_); break; // CB SRA E 
	case 0x2C: sra_r(h_); break; // CB SRA H 
	case 0x2D: sra_r(l_); break; // CB SRA L 
	case 0x2E: { // CB SRA (HL) 
		f_ = 0;
		int val = READ(hl());
		if (val & 1) {
			setCFlag();
		}
		val >>= 1;
		if (val & 0x40) {
			val |= 0x80;
		}
		if (val == 0) {
			setZFlag();
		}
		WRITE(hl(), val);
		break;
	}
	case 0x2F: sra_r(a_); break; // CB SRA A 

	case 0x30: swap_r(b_); // CB SWAP B 
	case 0x31: swap_r(c_); // CB SWAP C 
	case 0x32: swap_r(d_); // CB SWAP D 
	case 0x33: swap_r(e_); // CB SWAP E 
	case 0x34: swap_r(h_); // CB SWAP H 
	case 0x35: swap_r(l_); // CB SWAP L 
	case 0x36: { // CB SWAP (HL) 
		f_ = 0;
		uint8_t val = READ(hl());
		const uint8_t val2 = val >> 4;
		val <<= 4;
		val |= val2;
		WRITE(hl(), val);
		if (val == 0) {
			setZFlag();
		}
		break;
	}
	case 0x37: swap_r(a_); break; // CB SWAP A 

	case 0x38: srl_r(b_); break; // CB SRL B 
	case 0x39: srl_r(c_); break; // CB SRL C 
	case 0x3A: srl_r(d_); break; // CB SRL D 
	case 0x3B: srl_r(e_); break; // CB SRL E 
	case 0x3C: srl_r(h_); break; // CB SRL H 
	case 0x3D: srl_r(l_); break; // CB SRL L 
	case 0x3E: { // CB SRL (HL) 
		f_ = 0;
		uint8_t val = READ(hl());
		if (val & 1) {
			setCFlag();
		}
		val >>= 1;
		if (val == 0) {
			setZFlag();
		}
		WRITE(hl(), val);
		break;
	}
	case 0x3F: srl_r(a_); break; // CB SRL A 

	case 0x40: bit0_u8(b_); break; // CB BIT 0,B 
	case 0x41: bit0_u8(c_); break; // CB BIT 0,C 
	case 0x42: bit0_u8(d_); break; // CB BIT 0,D 
	case 0x43: bit0_u8(e_); break; // CB BIT 0,E 
	case 0x44: bit0_u8(h_); break; // CB BIT 0,H 
	case 0x45: bit0_u8(l_); break; // CB BIT 0,L 
	case 0x46: bit0_u8(READ(hl())); break; // CB BIT 0,(HL) 
	case 0x47: bit0_u8(a_); break; // CB BIT 0,A 

	case 0x48: bit1_u8(b_); break; // CB BIT 1,B 
	case 0x49: bit1_u8(c_); break; // CB BIT 1,C 
	case 0x4A: bit1_u8(d_); break; // CB BIT 1,D 
	case 0x4B: bit1_u8(e_); break; // CB BIT 1,E 
	case 0x4C: bit1_u8(h_); break; // CB BIT 1,H 
	case 0x4D: bit1_u8(l_); break; // CB BIT 1,L 
	case 0x4E: bit1_u8(READ(hl())); break; // CB BIT 1,(HL) 
	case 0x4F: bit1_u8(a_); break; // CB BIT 1,A 

	case 0x50: bit2_u8(b_); break; // CB BIT 2,B 
	case 0x51: bit2_u8(c_); break; // CB BIT 2,C 
	case 0x52: bit2_u8(d_); break; // CB BIT 2,D 
	case 0x53: bit2_u8(e_); break; // CB BIT 2,E 
	case 0x54: bit2_u8(h_); break; // CB BIT 2,H 
	case 0x55: bit2_u8(l_); break; // CB BIT 2,L 
	case 0x56: bit2_u8(READ(hl())); break; // CB BIT 2,(HL) 
	case 0x57: bit2_u8(a_); break; // CB BIT 2,A 

	case 0x58: bit3_u8(b_); break; // CB BIT 3,B 
	case 0x59: bit3_u8(c_); break; // CB BIT 3,C 
	case 0x5A: bit3_u8(d_); break; // CB BIT 3,D 
	case 0x5B: bit3_u8(e_); break; // CB BIT 3,E 
	case 0x5C: bit3_u8(h_); break; // CB BIT 3,H 
	case 0x5D: bit3_u8(l_); break; // CB BIT 3,L 
	case 0x5E: bit3_u8(READ(hl())); break; // CB BIT 3,(HL) 
	case 0x5F: bit3_u8(a_); break; // CB BIT 3,A 

	case 0x60: bit4_u8(b_); break; // CB BIT 4,B 
	case 0x61: bit4_u8(c_); break; // CB BIT 4,C 
	case 0x62: bit4_u8(d_); break; // CB BIT 4,D 
	case 0x63: bit4_u8(e_); break; // CB BIT 4,E 
	case 0x64: bit4_u8(h_); break; // CB BIT 4,H 
	case 0x65: bit4_u8(l_); break; // CB BIT 4,L 
	case 0x66: bit4_u8(READ(hl())); break; // CB BIT 4,(HL) 
	case 0x67: bit4_u8(a_); break; // CB BIT 4,A 

	case 0x68: bit5_u8(b_); break; // CB BIT 5,B 
	case 0x69: bit5_u8(c_); break; // CB BIT 5,C 
	case 0x6A: bit5_u8(d_); break; // CB BIT 5,D 
	case 0x6B: bit5_u8(e_); break; // CB BIT 5,E 
	case 0x6C: bit5_u8(h_); break; // CB BIT 5,H 
	case 0x6D: bit5_u8(l_); break; // CB BIT 5,L 
	case 0x6E: bit5_u8(READ(hl())); break; // CB BIT 5,(HL) 
	case 0x6F: bit5_u8(a_); break; // CB BIT 5,A 

	case 0x70: bit6_u8(b_); break; // CB BIT 6,B 
	case 0x71: bit6_u8(c_); break; // CB BIT 6,C 
	case 0x72: bit6_u8(d_); break; // CB BIT 6,D 
	case 0x73: bit6_u8(e_); break; // CB BIT 6,E 
	case 0x74: bit6_u8(h_); break; // CB BIT 6,H 
	case 0x75: bit6_u8(l_); break; // CB BIT 6,L 
	case 0x76: bit6_u8(READ(hl())); break; // CB BIT 6,(HL) 
	case 0x77: bit6_u8(a_); break; // CB BIT 6,A 

	case 0x78: bit7_u8(b_); break; // CB BIT 7,B 
	case 0x79: bit7_u8(c_); break; // CB BIT 7,C 
	case 0x7A: bit7_u8(d_); break; // CB BIT 7,D 
	case 0x7B: bit7_u8(e_); break; // CB BIT 7,E 
	case 0x7C: bit7_u8(h_); break; // CB BIT 7,H 
	case 0x7D: bit7_u8(l_); break; // CB BIT 7,L 
	case 0x7E: bit7_u8(READ(hl())); break; // CB BIT 7,(HL) 
	case 0x7F: bit7_u8(a_); break; // CB BIT 7,A 

	case 0x80: res0_r(b_); break; // CB RES 0,B 
	case 0x81: res0_r(c_); break; // CB RES 0,C 
	case 0x82: res0_r(d_); break; // CB RES 0,D 
	case 0x83: res0_r(e_); break; // CB RES 0,E 
	case 0x84: res0_r(h_); break; // CB RES 0,H 
	case 0x85: res0_r(l_); break; // CB RES 0,L 
	case 0x86: resn_mem_hl(0); // CB RES 0,(HL) 
	case 0x87: res0_r(a_); break; // CB RES 0,A 

	case 0x88: res1_r(b_); break; // CB RES 1,B 
	case 0x89: res1_r(c_); break; // CB RES 1,C 
	case 0x8A: res1_r(d_); break; // CB RES 1,D 
	case 0x8B: res1_r(e_); break; // CB RES 1,E 
	case 0x8C: res1_r(h_); break; // CB RES 1,H 
	case 0x8D: res1_r(l_); break; // CB RES 1,L 
	case 0x8E: resn_mem_hl(1); break; // CB RES 1,(HL) 
	case 0x8F: res1_r(a_); break; // CB RES 1,A 

	case 0x90: res2_r(b_); break; // CB RES 2,B 
	case 0x91: res2_r(c_); break; // CB RES 2,C 
	case 0x92: res2_r(d_); break; // CB RES 2,D 
	case 0x93: res2_r(e_); break; // CB RES 2,E 
	case 0x94: res2_r(h_); break; // CB RES 2,H 
	case 0x95: res2_r(l_); break; // CB RES 2,L 
	case 0x96: resn_mem_hl(2); break; // CB RES 2,(HL) 
	case 0x97: res2_r(a_); break; // CB RES 2,A 

	case 0x98: res3_r(b_); break; // CB RES 3,B 
	case 0x99: res3_r(c_); break; // CB RES 3,C 
	case 0x9A: res3_r(d_); break; // CB RES 3,D 
	case 0x9B: res3_r(e_); break; // CB RES 3,E 
	case 0x9C: res3_r(h_); break; // CB RES 3,H 
	case 0x9D: res3_r(l_); break; // CB RES 3,L 
	case 0x9E: resn_mem_hl(3); break; // CB RES 3,(HL) 
	case 0x9F: res3_r(a_); break; // CB RES 3,A 

	case 0xA0: res4_r(b_); break; // CB RES 4,B 
	case 0xA1: res4_r(c_); break; // CB RES 4,C 
	case 0xA2: res4_r(d_); break; // CB RES 4,D 
	case 0xA3: res4_r(e_); break; // CB RES 4,E 
	case 0xA4: res4_r(h_); break; // CB RES 4,H 
	case 0xA5: res4_r(l_); break; // CB RES 4,L 
	case 0xA6: resn_mem_hl(4); break; // CB RES 4,(HL) 
	case 0xA7: res4_r(a_); break; // CB RES 4,A 

	case 0xA8: res5_r(b_); break; // CB RES 5,B 
	case 0xA9: res5_r(c_); break; // CB RES 5,C 
	case 0xAA: res5_r(d_); break; // CB RES 5,D 
	case 0xAB: res5_r(e_); break; // CB RES 5,E 
	case 0xAC: res5_r(h_); break; // CB RES 5,H 
	case 0xAD: res5_r(l_); break; // CB RES 5,L 
	case 0xAE: resn_mem_hl(5); break; // CB RES 5,(HL) 
	case 0xAF: res5_r(a_); break; // CB RES 5,A 

	case 0xB0: res6_r(b_); break; // CB RES 6,B 
	case 0xB1: res6_r(c_); break; // CB RES 6,C 
	case 0xB2: res6_r(d_); break; // CB RES 6,D 
	case 0xB3: res6_r(e_); break; // CB RES 6,E 
	case 0xB4: res6_r(h_); break; // CB RES 6,H 
	case 0xB5: res6_r(l_); break; // CB RES 6,L 
	case 0xB6: resn_mem_hl(6); break; // CB RES 6,(HL) 
	case 0xB7: res6_r(a_); break; // CB RES 6,A 

	case 0xB8: res7_r(b_); break; // CB RES 7,B 
	case 0xB9: res7_r(c_); break; // CB RES 7,C 
	case 0xBA: res7_r(d_); break; // CB RES 7,D 
	case 0xBB: res7_r(e_); break; // CB RES 7,E 
	case 0xBC: res7_r(h_); break; // CB RES 7,H 
	case 0xBD: res7_r(l_); break; // CB RES 7,L 
	case 0xBE: resn_mem_hl(7); break; // CB RES 7,(HL) 
	case 0xBF: res7_r(a_); break; // CB RES 7,A 

	case 0xC0: set0_r(b_); break; // CB SET 0,B 
	case 0xC1: set0_r(c_); break; // CB SET 0,C 
	case 0xC2: set0_r(d_); break; // CB SET 0,D 
	case 0xC3: set0_r(e_); break; // CB SET 0,E 
	case 0xC4: set0_r(h_); break; // CB SET 0,H 
	case 0xC5: set0_r(l_); break; // CB SET 0,L 
	case 0xC6: setn_mem_hl(0); break; // CB SET 0,(HL) 
	case 0xC7: set0_r(a_); break; // CB SET 0,A 

	case 0xC8: set1_r(b_); break; // CB SET 1,B 
	case 0xC9: set1_r(c_); break; // CB SET 1,C 
	case 0xCA: set1_r(d_); break; // CB SET 1,D 
	case 0xCB: set1_r(e_); break; // CB SET 1,E 
	case 0xCC: set1_r(h_); break; // CB SET 1,H 
	case 0xCD: set1_r(l_); break; // CB SET 1,L 
	case 0xCE: setn_mem_hl(1); break; // CB SET 1,(HL) 
	case 0xCF: set1_r(a_); break; // CB SET 1,A 

	case 0xD0: set2_r(b_); break; // CB SET 2,B 
	case 0xD1: set2_r(c_); break; // CB SET 2,C 
	case 0xD2: set2_r(d_); break; // CB SET 2,D 
	case 0xD3: set2_r(e_); break; // CB SET 2,E 
	case 0xD4: set2_r(h_); break; // CB SET 2,H 
	case 0xD5: set2_r(l_); break; // CB SET 2,L 
	case 0xD6: setn_mem_hl(2); break; // CB SET 2,(HL) 
	case 0xD7: set2_r(a_); break; // CB SET 2,A 

	case 0xD8: set3_r(b_); break; // CB SET 3,B 
	case 0xD9: set3_r(c_); break; // CB SET 3,C 
	case 0xDA: set3_r(d_); break; // CB SET 3,D 
	case 0xDB: set3_r(e_); break; // CB SET 3,E 
	case 0xDC: set3_r(h_); break; // CB SET 3,H 
	case 0xDD: set3_r(l_); break; // CB SET 3,L 
	case 0xDE: setn_mem_hl(3); break; // CB SET 3,(HL) 
	case 0xDF: set3_r(a_); break; // CB SET 3,A 

	case 0xE0: set4_r(b_); break; // CB SET 4,B 
	case 0xE1: set4_r(c_); break; // CB SET 4,C 
	case 0xE2: set4_r(d_); break; // CB SET 4,D 
	case 0xE3: set4_r(e_); break; // CB SET 4,E 
	case 0xE4: set4_r(h_); break; // CB SET 4,H 
	case 0xE5: set4_r(l_); break; // CB SET 4,L 
	case 0xE6: setn_mem_hl(4); break; // CB SET 4,(HL) 
	case 0xE7: set4_r(a_); break; // CB SET 4,A 

	case 0xE8: set5_r(b_); break; // CB SET 5,B 
	case 0xE9: set5_r(c_); break; // CB SET 5,C 
	case 0xEA: set5_r(d_); break; // CB SET 5,D 
	case 0xEB: set5_r(e_); break; // CB SET 5,E 
	case 0xEC: set5_r(h_); break; // CB SET 5,H 
	case 0xED: set5_r(l_); break; // CB SET 5,L 
	case 0xEE: setn_mem_hl(5); break; // CB SET 5,(HL) 
	case 0xEF: set5_r(a_); break; // CB SET 5,A 

	case 0xF0: set6_r(b_); break; // CB SET 6,B 
	case 0xF1: set6_r(c_); break; // CB SET 6,C 
	case 0xF2: set6_r(d_); break; // CB SET 6,D 
	case 0xF3: set6_r(e_); break; // CB SET 6,E 
	case 0xF4: set6_r(h_); break; // CB SET 6,H 
	case 0xF5: set6_r(l_); break; // CB SET 6,L 
	case 0xF6: setn_mem_hl(6); break; // CB SET 6,(HL) 
	case 0xF7: set6_r(a_); break; // CB SET 6,A 

	case 0xF8: set7_r(b_); break; // CB SET 7,B 
	case 0xF9: set7_r(c_); break; // CB SET 7,C 
	case 0xFA: set7_r(d_); break; // CB SET 7,D 
	case 0xFB: set7_r(e_); break; // CB SET 7,E 
	case 0xFC: set7_r(h_); break; // CB SET 7,H 
	case 0xFD: set7_r(l_); break; // CB SET 7,L 
	case 0xFE: setn_mem_hl(7); break; // CB SET 7,(HL) 
	case 0xFF: set7_r(a_); break; // CB SET 7,A 
	}
}