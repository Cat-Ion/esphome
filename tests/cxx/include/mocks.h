#pragma once

#include <stdint.h>

struct MockValues {
	uint32_t millis{0};
} mock_values;

void mocks_reset() {
	mock_values = MockValues();
}

namespace esphome {
uint32_t millis() {
	return mock_values.millis;
}
}
