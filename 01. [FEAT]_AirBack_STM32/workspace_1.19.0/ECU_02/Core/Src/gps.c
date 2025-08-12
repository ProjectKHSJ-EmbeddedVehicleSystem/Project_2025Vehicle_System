#include "gps.h"
#include <string.h>
#include <stdlib.h>

#define LINE_MAX 128

static char line_buf[LINE_MAX];
static volatile uint16_t line_len;
static GPSFix latest;
static volatile bool has_new_valid;

static int nmea_checksum_ok(const char *s) {
    // s는 '$' 다음부터 '*' 전까지
    uint8_t csum = 0;
    while (*s && *s != '*') { csum ^= (uint8_t)(*s++); }
    if (*s != '*') return 0;
    char h1 = *(s+1), h2 = *(s+2);
    uint8_t val = (uint8_t)strtoul((char[]){h1,h2,0}, NULL, 16);
    return csum == val;
}

// ddmm.mmmm -> degrees(double)
static double dm_to_deg(const char *dm) {
    if (!dm || !*dm) return 0.0;
    double v = atof(dm);
    int deg = (int)(v / 100.0);
    double min = v - (deg * 100.0);
    return (double)deg + min/60.0;
}

static void parse_gprmc(char *s) {
    // s: "GPRMC,hhmmss.sss,A,llll.ll,a,yyyyy.yy,a,..."
    // 토큰 분해
    // 인덱스: 0=GPRMC 1=UTC 2=valid 3=lat 4=N/S 5=lon 6=E/W
    char *field[16] = {0};
    int idx = 0;
    for (char *p = s; *p && idx < 16; ++idx) {
        field[idx] = p;
        while (*p && *p != ',' && *p != '*') p++;
        if (*p == ',' ) { *p++ = '\0'; }
        else if (*p == '*') { *p = '\0'; break; }
    }
    if (idx < 7) return;

    GPSFix fix = {0};
    // 유효성
    fix.valid = (field[2] && field[2][0] == 'A');

    // 시간 hhmmss
    if (field[1] && strlen(field[1]) >= 6) {
        fix.hh = (field[1][0]-'0')*10 + (field[1][1]-'0');
        fix.mm = (field[1][2]-'0')*10 + (field[1][3]-'0');
        fix.ss = (field[1][4]-'0')*10 + (field[1][5]-'0');
    }

    // 위도/경도
    double lat = dm_to_deg(field[3]);
    double lon = dm_to_deg(field[5]);
    if (field[4] && field[4][0] == 'S') lat = -lat;
    if (field[6] && field[6][0] == 'W') lon = -lon;

    fix.latitude  = lat;
    fix.longitude = lon;

    if (fix.valid) {
        latest = fix;
        has_new_valid = true;
    }
}

void GPS_Init(void) {
    line_len = 0;
    latest.valid = false;
    has_new_valid = false;
}

void GPS_OnByte(uint8_t b) {
    if (b == '\r') return; // 무시
    if (b == '\n') {
        // 한 줄 완성
        if (line_len >= 6 && line_buf[0] == '$') {
            // 체크섬 확인
            char *star = strchr(line_buf, '*');
            if (star && nmea_checksum_ok(line_buf + 1)) {
                // 문장 타입 확인
                if (strstr(line_buf, "$GPRMC") == line_buf || strstr(line_buf, "$GNRMC") == line_buf) {
                    // '$' 제거하고 타입 이후부터 파싱
                    parse_gprmc(line_buf + 1); // 'GPRMC,...'
                }
            }
        }
        line_len = 0;
        return;
    }
    if (line_len < LINE_MAX-1) {
        line_buf[line_len++] = (char)b;
        line_buf[line_len] = '\0';
    } else {
        line_len = 0; // overflow 시 라인 리셋
    }
}

bool GPS_GetLatestFix(GPSFix *out) {
    if (!out) return false;
    if (!latest.valid) return false;
    *out = latest;
    return has_new_valid; // true면 "새로 업데이트됨"
}
