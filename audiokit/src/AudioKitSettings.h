/**
 * @file AudioKitSettings.h
 * @author Phil Schatzmann
 * @brief AUDIOKIT_BOARD selects a specic board:
 *  1) lyrat_v4_3
 *  2) lyrat_v4_2
 *  3) lyrat_mini_v1_1
 *  4) esp32_s2_kaluga_1_v1_2
 *  5) ai_thinker (ES8388) 2957 3478 A149
 *  6) ai_thinker (AC101) 2473 2762 2957
 *  7) ai_thinker (ES8388) 2957
 *  10) generci_es8388
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 */

#pragma once

// Select your specific board
#ifndef AUDIOKIT_BOARD
#  define AUDIOKIT_BOARD 11
#endif

// Default settings
#define AUDIOKIT_DEFAULT_INPUT          AUDIO_HAL_ADC_INPUT_LINE1
#define AUDIOKIT_DEFAULT_OUTPUT         AUDIO_HAL_DAC_OUTPUT_ALL
#define AUDIOKIT_DEFAULT_MASTER_SLAVE   AUDIO_HAL_MODE_SLAVE
#define AUDIOKIT_DEFAULT_RATE           AUDIO_HAL_16K_SAMPLES//AUDIO_HAL_44K_SAMPLES
#define AUDIOKIT_DEFAULT_BITSIZE        AUDIO_HAL_BIT_LENGTH_16BITS
#define AUDIOKIT_DEFAULT_I2S_FMT        AUDIO_HAL_I2S_NORMAL

// Define the default gain for the microphone amp 0xbb=9DB; 0x00=0DB
// Alternativly you can call es8388_set_mic_gain(es_mic_gain_t gain) if you prefer to use
// value from an comprehensive enum
#define ES8388_DEFAULT_MIC_GAIN         0xbb

// 1 = Using Arduino Wire Library; 0 = Use ESP32 I2C API
#ifndef AUDIOKIT_USE_WIRE
#  define AUDIOKIT_USE_WIRE 0
// for AUDIOKIT_WIRE use Wire or Wire1
#  define AUDIOKIT_WIRE Wire1
#endif

// Set to 0 if you want to deactivate the ESP32 I2S functionality
#ifndef SETUP_ESP32_I2S
#  define SETUP_ESP32_I2S 1
#endif

// USE SD ? Set to 0 to deactivate the SD
#ifndef AUDIOKIT_SETUP_SD
#  define AUDIOKIT_SETUP_SD 0
// for AUDIOKIT_SD_SPI you can select SPI or SPI_VSPI
#  define AUDIOKIT_SD_SPI SPI
#endif

// To increase the max volume e.g. for ai_thinker (ES8388) 2957 or A202 -> set to 1 or 2
// 0 AUX volume is LINE level
// 1 you can control the AUX volume with setVolume()
#ifndef AI_THINKER_ES8388_VOLUME_HACK
#  define AI_THINKER_ES8388_VOLUME_HACK 1
#endif

// ESP32 Specific Settings
#ifdef ESP32
#  ifndef AUDIOKIT_FREE_RTOS
#    define AUDIOKIT_FREE_RTOS 1
#  endif

#  ifndef AUDIOKIT_MUTEX_SUPPORT
#    define AUDIOKIT_MUTEX_SUPPORT 1
#  endif
#endif


