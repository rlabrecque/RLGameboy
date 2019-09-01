#pragma once

#include "timer.h"

class key {
public:
	uint8_t _keys[2];
	uint8_t _colidx;

	void reset();

	uint8_t rb();

	void wb(uint8_t v);
};

extern key KEY;