// Version 1.0.0
// Date: 2026-08-17
// Owner: Ken Caird

// FDX-B / ISO 11784/11785 CRC-16 implementation
// Polynomial: 0x1021
// Initial value: 0xFFFF
// Process bits LSB-first (per transmitted bit). Final XOR: 0xFFFF.
//
// rawDataBits[64] should contain the 64 bits in transmission order:
//   rawDataBits[0] == first bit transmitted (LSB of first byte if tag sends LSB-first per byte).
#include <Arduino.h>
#include "crc16.h"

// Compute CRC-16 (reflected) for ISO 11784/11785 FDX-B frame.
// rawDataBits: array of bits (0/1), index 0 is first transmitted bit (LSB-first).
// nBits: number of bits to process (use 64).
// Returns 16-bit CRC (no final XOR).
uint16_t crc16_iso11784_reflected(const uint8_t rawDataBits[], size_t nBits) {
  const uint16_t POLY_REF = 0x8408; // reflected 0x1021
  uint16_t crc = 0x0000;            // initial value confirmed by your test

  for (size_t i = 0; i < nBits; ++i) {
    uint8_t bit = rawDataBits[i] & 1u;
    uint8_t mix = (uint8_t)((crc & 0x0001u) ^ bit);
    crc >>= 1;
    if (mix) crc ^= POLY_REF;
  }
  return crc & 0xFFFFu;
}