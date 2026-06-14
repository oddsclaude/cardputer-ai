#pragma once

// Speak text through the M5 speaker using formant synthesis.
// Blocks until playback is complete. Call from the main task only.
void tts_speak(const char* text);
