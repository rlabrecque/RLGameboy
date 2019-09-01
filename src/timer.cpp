#include "cpu.h"
#include "mmu.h"
#include "timer.h"
/*
timer TIMER;

void timer::reset() {
	_div = 0;
	_tma = 0;
	_tima = 0;
	_tac = 0;
	_clock._main = 0;
	_clock._sub = 0;
	_clock._div = 0;
}

void timer::inc() {
	uint64_t oldclk = _clock._main;

	if (_clock._sub > 3) {
		_clock._main++;
		_clock._sub -= 4;

		_clock._div++;
		if (_clock._div == 16) {
			_clock._div = 0;
			_div++;
			_div &= 255;
		}
	}

	if (_tac & 4) {
		switch (_tac & 3) {
		case 0:
			if (_clock._main >= 64) {
				step();
			}
			break;
		case 1:
			if (_clock._main >= 1) {
				step();
			}
			break;
		case 2:
			if (_clock._main >= 4) {
				step();
			}
			break;
		case 3:
			if (_clock._main >= 16) {
				step();
			}
			break;
		}
	}
}

void timer::step() {
	_tima++;
	_clock._main = 0;
	if (_tima > 255) {
		_tima = _tma;
		//MMU._if |= 4;
	}
}

uint8_t timer::rb(uint16_t addr) {
	switch (addr)
	{
	case 0xFF04:
		return _div;
	case 0xFF05:
		return _tima;
	case 0xFF06:
		return _tma;
	case 0xFF07:
		return _tac;
	}

	__debugbreak();
	return 0;
}

void timer::wb(uint16_t addr, uint8_t val) {
	switch (addr) {
	case 0xFF04:
		_div = 0;
		break;
	case 0xFF05:
		_tima = val;
		break;
	case 0xFF06:
		_tma = val;
		break;
	case 0xFF07:
		_tac = val & 7;
		break;
	}
}*/