#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool    valid;      // 'A'면 true, 'V'면 false
    double  latitude;   // degrees, +북/-남
    double  longitude;  // degrees, +동/-서
    uint8_t hh, mm, ss; // UTC (옵션)
} GPSFix;

void GPS_Init(void);
void GPS_OnByte(uint8_t b);        // ISR에서 1바이트씩 투입
bool GPS_GetLatestFix(GPSFix *out); // 최근 유효 fix 복사
