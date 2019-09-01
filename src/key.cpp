#include "key.h"

#include <stdio.h>

key KEY;

void key::reset() {
	_keys[0] = 0xF;
	_keys[1] = 0xF;

	_colidx = 0;
}

uint8_t key::rb() {
	switch (_colidx) {
	case 0x00:
		return 0x00;
		break;
	case 0x10:
		return _keys[0];
		break;
	case 0x20:
		return _keys[1];
		break;
	}

	printf("[KEY] rb unhandled case should never happen: %d\n", _colidx);
	return 0x00;
}

void key::wb(uint8_t v) {
	_colidx = v & 0x30;
}