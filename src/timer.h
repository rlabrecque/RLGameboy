#pragma once

#include<stdint.h>

class timer {
public:
	struct clock {
		uint64_t _main;
		uint64_t _sub;
		uint64_t _div;
	};

	uint8_t _div;
	uint8_t _tma;
	uint8_t _tima;
	uint8_t _tac;
	clock _clock;

	void reset();

	void inc();

	void step();

	uint8_t rb(uint16_t addr);

	void wb(uint16_t addr, uint8_t val);
};

extern timer TIMER;