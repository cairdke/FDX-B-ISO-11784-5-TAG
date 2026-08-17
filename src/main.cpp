// Version 3.0.0
// Date: 2026-08-17
// Owner: Ken Caird

// ===============================================================
// ISO 11784/11785 FDX-B ASK Modulator – Arduino UNO
// Clean, lab-ready implementation for real reader testing
// ===============================================================

#include <Arduino.h>
#include <crc16.h>


const uint8_t OUT_PIN = 2;

// --- Your tag parameters (edit these for your own IDs) ---
const uint64_t NATIONAL_ID   = 98100661108;  // 38-bit, 12 Character National ID (LSB-first)
const uint16_t COUNTRY_CODE  = 578;             // 10-bit 3-digit ISO 3166-1 numeric country code (LSB-first)
const bool     DATA_BLOCK    = false;
const bool     ANIMAL_FLAG   = true;
const uint32_t EXTENDED_DATA = 0;
const uint16_t INTER_FRAME_BITS = 200;

// --- Frame + DBP buffers ---
uint8_t frameBits[128];
uint8_t dbp[256];
volatile uint16_t dbp_len = 0;
volatile uint16_t dbp_pos = 0;
volatile bool     sending_frame = true;
volatile uint32_t delay_halfbits = 0;

// ---------------------------------------------------------------
// Helper: push bits LSB-first into frameBits[]
// ---------------------------------------------------------------
void pushBit(uint8_t bit, uint16_t &idx) {
    if (idx < 128) frameBits[idx++] = bit & 1;
}

void pushLSB(uint32_t value, uint8_t count, uint16_t &idx) {
    for (uint8_t i = 0; i < count; i++)
        pushBit((value >> i) & 1, idx);
}

void reverseBits(uint8_t *bits, uint16_t count) {
    for (uint16_t i = 0; i < count / 2; i++) {
        uint8_t tmp = bits[i];
        bits[i] = bits[count - 1 - i];
        bits[count - 1 - i] = tmp;
    }
}

// ---------------------------------------------------------------
// Build FDX-B frame – ISO 11785 compliant
// ---------------------------------------------------------------
void buildFDXFrame() {
    uint16_t idx = 0;

    // 1. Header: 10000000000 (LSB-first)
    pushLSB(0b10000000000, 11, idx);

    // 2. Build 64-bit ID block
    uint64_t idBlock = 0;

    // National ID (38 bits, LSB-first)
    idBlock |= (NATIONAL_ID & ((1ULL << 38) - 1));
            
    // Country Code (10 bits)
    idBlock |= (uint64_t)(COUNTRY_CODE & 0x3FF) << 38;

    // Data Block flag (bit 48)
    idBlock |= (uint64_t)(DATA_BLOCK ? 1ULL : 0ULL) << 48;

    // Animal flag (bit 63)
    idBlock |= (uint64_t)(ANIMAL_FLAG ? 1ULL : 0ULL) << 63;

    // Reserved bits 50–63 = 0

    // 3. Extract raw 64 bits for CRC (LSB-first)
    uint8_t rawDataBits[64];
    for (uint8_t i = 0; i < 64; i++)
        rawDataBits[i] = (idBlock >> i) & 1;

       

    // 4. Compute CRC
    //uint16_t crc = computeFDXBCRC(rawDataBits, 64);
    uint16_t crc = crc16_iso11784_reflected(rawDataBits, 64);
    Serial.print("CRC: 0x");
    Serial.println(crc, HEX);
    
    // 5. Push 64 data bits with control bits (1 per 8 bits)
    for (uint8_t grp = 0; grp < 8; grp++) {
        for (uint8_t i = 0; i < 8; i++)
            pushBit(rawDataBits[grp * 8 + i], idx);
        pushBit(1, idx);   // control bit
    }
    
      
    // 6. Push CRC (LSB-first), with control bits after each byte
    for (uint8_t i = 0; i < 8; i++)
        pushBit((crc >> i) & 1, idx);   // low byte
    pushBit(1, idx);                    // control

    for (uint8_t i = 8; i < 16; i++)
        pushBit((crc >> i) & 1, idx);   // high byte
    pushBit(1, idx);                    // control

    // 7. Extended data (only if DATA_BLOCK = true)
    if (DATA_BLOCK) {
        for (uint8_t grp = 0; grp < 3; grp++) {
            uint8_t b = (EXTENDED_DATA >> (grp * 8)) & 0xFF;
            for (uint8_t i = 0; i < 8; i++)
                pushBit((b >> i) & 1, idx);
            pushBit(1, idx);   // control
        }
    }

    // 8. Pad to 128 bits
    while (idx < 128)
        pushBit(0, idx);
    
       
    
}




// ---------------------------------------------------------------
// DBP encoding – ISO 11785 rules
// ---------------------------------------------------------------
void encodeDBP() {
    dbp_len = 0;
    uint8_t last_level = 0;

    for (uint16_t i = 0; i < 128; i++) {
        uint8_t bit = frameBits[i];

        // First half-bit (boundary): toggle ONLY for 0-bit
        if (bit == 0) last_level ^= 1;
        dbp[dbp_len++] = last_level;

        // Second half-bit (mid): ALWAYS toggle
        last_level ^= 1;
        dbp[dbp_len++] = last_level;
    }
}

// ---------------------------------------------------------------
// Timer1 ISR – outputs DBP envelope
// ---------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
    if (sending_frame) {
        digitalWrite(OUT_PIN, dbp[dbp_pos]);
        dbp_pos++;
        if (dbp_pos >= dbp_len) {
            sending_frame  = false;
            delay_halfbits = (uint32_t)INTER_FRAME_BITS * 2;
            dbp_pos = 0;
            digitalWrite(OUT_PIN, LOW);
        }
    } else {
        digitalWrite(OUT_PIN, LOW);
        if (delay_halfbits > 0) delay_halfbits--;
        else sending_frame = true;
    }
}

// ---------------------------------------------------------------
// Setup Timer1 for ~119.25 µs half-bit
// ---------------------------------------------------------------
void setupTimer() {
    noInterrupts();
    TCCR1A = 0; TCCR1B = 0;
    TCCR1B |= (1 << WGM12);
    OCR1A   = 1908;          // 16 MHz, prescaler 1 → ~119.25 µs
    TIMSK1 |= (1 << OCIE1A);
    TCCR1B |= (1 << CS10);
    interrupts();
}

void setupCarrier() {
  // Set OC2A (Pin 11) as output
  pinMode(11, OUTPUT);

  // Timer2: CTC mode, toggle OC2A on compare match
  TCCR2A = 0;
  TCCR2B = 0;

  // CTC mode (WGM21 = 1)
  TCCR2A |= (1 << WGM21);

  // Toggle OC2A on compare match (COM2A0 = 1)
  TCCR2A |= (1 << COM2A0);

  // No prescaler (CS20 = 1)
  TCCR2B |= (1 << CS20);

  // Set compare value for ~134.2 kHz
  OCR2A = 59;  // tweak if you want slightly closer
}


void setup() {
    Serial.begin(115200);
    pinMode(OUT_PIN, OUTPUT);
    digitalWrite(OUT_PIN, LOW);

    buildFDXFrame();
    encodeDBP();
    setupTimer();
    setupCarrier();
}

void loop() {}
