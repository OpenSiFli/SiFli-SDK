#ifndef _AUDIO_TEST_API_H_
#define _AUDIO_TEST_API_H_
#include <stdint.h>
void audio_test_play_1khz(uint32_t samplerate,  uint8_t eq_enble);
void audio_test_play_zero(uint32_t samplerate,  uint8_t eq_enble);
void audio_test_play_file(uint32_t samplerate,  uint8_t eq_enble);
void audio_test_record_and_play_file(uint8_t eq_enble);
void audio_test_record_16k_start(uint8_t playing, uint8_t dump);
void audio_test_record_16k_end(void);
void audio_test_3a_enable(uint8_t is_enable, uint8_t eq_enable);

#endif
