#pragma once
#include <stdint.h>

uint16_t crc16_iso11784_reflected(const uint8_t rawDataBits[], size_t nBits);