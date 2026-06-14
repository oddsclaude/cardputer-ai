#include "tts.h"
#include <M5Unified.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static constexpr uint32_t SR    = 8000;  // sample rate (Hz)
static constexpr int      PITCH = 130;   // voiced fundamental (Hz)

// Second-order parallel formant resonator.
// H(z) ≈ bandpass at `f` Hz with bandwidth `bw` Hz.
struct Res {
    float a = 0, b = 0, a2 = 0, y1 = 0, y2 = 0;
    void reset(float f, float bw) {
        a  = expf(-(float)M_PI * bw / SR);
        b  = 2.f * a * cosf(2.f * (float)M_PI * f / SR);
        a2 = a * a;
        y1 = y2 = 0;
    }
    float tick(float x) {
        float y = b*y1 - a2*y2 + (1.f - a2)*x;
        y2 = y1; y1 = y;
        return y;
    }
};

// Phoneme parameters.
struct Ph {
    uint16_t f1, f2, f3;  // formant frequencies (Hz)
    bool voiced;           // glottal pulse excitation
    bool noise;            // noise component (fricatives/stops)
    uint8_t dur10;         // duration in units of 10 ms
};

// Index matches the letter mapping in char_to_ph().
static const Ph PH[] = {
 // f1    f2    f3    vcd   nse   dur
 {  0,    0,    0,   false,false, 8},  // 0  silence
 {270, 2290, 3010,   true,false,  9},  // 1  IY  (beat)
 {390, 1990, 2550,   true,false,  8},  // 2  IH  (bit)
 {530, 1840, 2480,   true,false,  9},  // 3  EH  (bet)
 {660, 1720, 2410,   true,false,  9},  // 4  AE  (bat)
 {520, 1190, 2390,   true,false,  9},  // 5  AH  (but)
 {730, 1090, 2440,   true,false, 10},  // 6  AA  (father)
 {570,  840, 2410,   true,false,  9},  // 7  AO  (bought)
 {440, 1020, 2240,   true,false,  7},  // 8  UH  (book)
 {300,  870, 2240,   true,false,  9},  // 9  UW  (boot)
 {490, 1350, 1690,   true,false, 10},  // 10 ER  (bird)
 {280,  900, 2200,   true,false,  8},  // 11 M
 {280, 1700, 2600,   true,false,  7},  // 12 N
 {360, 1090, 2670,   true,false,  9},  // 13 L
 {490, 1350, 1690,   true,false,  8},  // 14 R
 {295,  610, 2200,   true,false,  8},  // 15 W
 {240, 2100, 3000,   true,false,  7},  // 16 Y
 {300,  900, 2400,   true,true,   5},  // 17 B  (voiced stop)
 {300, 1700, 2600,   true,true,   5},  // 18 D
 {300, 1980, 2500,   true,true,   5},  // 19 G
 {600,  900, 2400,  false,true,   5},  // 20 P  (unvoiced stop)
 {600, 1700, 2600,  false,true,   5},  // 21 T
 {600, 1980, 2500,  false,true,   5},  // 22 K
 {175,  900, 2400,  false,true,   9},  // 23 F
 {200, 1500, 2500,  false,true,   9},  // 24 TH
 {200, 1700, 2600,  false,true,   8},  // 25 S
 {200, 1800, 2700,  false,true,   9},  // 26 SH
 {175,  900, 2400,   true,true,   9},  // 27 V
 {200, 1700, 2600,   true,true,   8},  // 28 Z
 {800, 1200, 2400,  false,true,   5},  // 29 HH (aspiration)
};

// Map one ASCII character to 1–2 phoneme indices (into PH[]).
static void char_to_ph(char c, uint8_t* out, int* n) {
    *n = 1;
    switch (tolower((unsigned char)c)) {
        case 'a': out[0] = 4;  break;  // AE
        case 'b': out[0] = 17; break;
        case 'c': out[0] = 22; break;  // K
        case 'd': out[0] = 18; break;
        case 'e': out[0] = 3;  break;  // EH
        case 'f': out[0] = 23; break;
        case 'g': out[0] = 19; break;
        case 'h': out[0] = 29; break;  // HH
        case 'i': out[0] = 2;  break;  // IH
        case 'j': out[0] = 18; break;  // D (approximate)
        case 'k': out[0] = 22; break;
        case 'l': out[0] = 13; break;
        case 'm': out[0] = 11; break;
        case 'n': out[0] = 12; break;
        case 'o': out[0] = 7;  break;  // AO
        case 'p': out[0] = 20; break;
        case 'q': out[0] = 22; break;
        case 'r': out[0] = 14; break;
        case 's': out[0] = 25; break;
        case 't': out[0] = 21; break;
        case 'u': out[0] = 8;  break;  // UH
        case 'v': out[0] = 27; break;
        case 'w': out[0] = 15; break;
        case 'x': *n=2; out[0]=22; out[1]=25; break; // K+S
        case 'y': out[0] = 1;  break;  // IY
        case 'z': out[0] = 28; break;
        case ' ':  case '\t': case '\n': out[0] = 0; break;  // short pause
        case ',':  case ';':             out[0] = 0; break;
        case '.':  case '!': case '?':   *n=2; out[0]=0; out[1]=0; break; // longer pause
        default: *n = 0; break;
    }
}

static Res   s_r1, s_r2, s_r3;
static float s_phase = 0.f;

// Buffer sized for the longest phoneme (120 ms at 8 kHz = 960 samples).
static int16_t s_buf[960];

static void synth_one(uint8_t idx) {
    const Ph& p = PH[idx];
    int len = (int)p.dur10 * (int)SR / 100;
    if (len > (int)(sizeof(s_buf) / sizeof(s_buf[0])))
        len = sizeof(s_buf) / sizeof(s_buf[0]);

    s_r1.reset(p.f1 ? p.f1 : 100.f, 90.f);
    s_r2.reset(p.f2 ? p.f2 : 100.f, 110.f);
    s_r3.reset(p.f3 ? p.f3 : 100.f, 130.f);

    const float pp = (float)SR / PITCH;  // samples per pitch period

    for (int i = 0; i < len; i++) {
        float exc = 0.f;

        if (p.voiced) {
            // Approximate glottal pulse: sharp positive peak, slight negative plateau.
            s_phase += 1.f / pp;
            if (s_phase >= 1.f) s_phase -= 1.f;
            exc = (s_phase < 0.06f) ? 1.0f : -0.04f;
        }

        if (p.noise) {
            float ns = ((float)(rand() & 0xFFFF) / 32767.f - 1.f);
            exc = p.voiced ? exc * 0.35f + ns * 0.65f : ns;
        }

        if (idx == 0) { s_buf[i] = 0; continue; }

        float out = s_r1.tick(exc) * 0.50f
                  + s_r2.tick(exc) * 0.35f
                  + s_r3.tick(exc) * 0.15f;
        out *= 5500.f;
        if (out >  32000.f) out =  32000.f;
        if (out < -32000.f) out = -32000.f;
        s_buf[i] = (int16_t)out;
    }

    if (len > 0) {
        M5.Speaker.playRaw(s_buf, (size_t)len, SR, false, 1, 0, true);
        while (M5.Speaker.isPlaying(0)) vTaskDelay(1);
    }
}

void tts_speak(const char* text) {
    if (!text || !*text) return;

    M5.Speaker.begin();
    M5.Speaker.setVolume(200);

    s_phase = 0.f;
    s_r1 = {}; s_r2 = {}; s_r3 = {};

    for (const char* p = text; *p; p++) {
        uint8_t phs[2]; int n = 0;
        char_to_ph(*p, phs, &n);
        for (int i = 0; i < n; i++) synth_one(phs[i]);
    }
}
