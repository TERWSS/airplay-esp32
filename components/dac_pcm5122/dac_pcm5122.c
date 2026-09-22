/**
 * @file dac_pcm5122.c
 * @brief TI PCM5122 stereo audio DAC driver
 *
 * Implements the dac_ops_t interface for the PCM5122 via I2C control.
 * Basic functions only — power mode, digital volume and analog gain.
 * No DSP/miniDSP features exercised here (the biquad/DRC coefficient RAM
 * is left at its power-on-reset identity state), unlike the TAS57xx/TAS58xx
 * drivers.
 *
 * Register map and init sequence adapted from a community ESPHome pcm5122
 * component (esp32-audio-dock/esphome-pcm5122/components/pcm5122), which in
 * turn cites TI's PCM5122 datasheet SLASE55.
 *
 * Power mode maps DAC_POWER_ON to PLAY and both DAC_POWER_STANDBY and
 * DAC_POWER_OFF to PWRDOWN — the part has no intermediate state worth
 * distinguishing here, so anything short of actively playing powers it
 * all the way down.
 */

#include "dac_pcm5122.h"
#include "board_utils.h"

#include <math.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define I2C_LINE_SPEED 400000

// Page 0 register addresses (TI PCM5122 datasheet SLASE55, §8.6)
#define PCM5122_REG_PAGE_SELECT 0x00
#define PCM5122_REG_RESET       0x01
#define PCM5122_REG_STATE       0x02 // "Power Control" in the datasheet
#define PCM5122_REG_MUTE        0x03
#define PCM5122_REG_ERROR_DET   0x25
#define PCM5122_REG_AUDIO_FMT   0x28
#define PCM5122_REG_MIXER       0x2A // DAC data path / mixer
#define PCM5122_REG_DVOL_LEFT   0x3D
#define PCM5122_REG_DVOL_RIGHT  0x3E
#define PCM5122_REG_PLL_REF     0x0D

// Page 1 register addresses
#define PCM5122_REG_ANALOG_GAIN 0x02

// Full reset (RSTM|RSTR): resets the whole digital block, not just the
// audio modules. The part needs ~100ms on each side to settle.
#define PCM5122_RESET_FULL 0x11

#define PCM5122_AUDIO_FMT_I2S_16BIT 0x00 // AFMT=I2S (bits 5:4), ALEN=16-bit

// Ignore missing-MCLK errors: no MCLK pin is wired on this DAC's supported
// boards, so the PLL runs from BCK and would otherwise trip the detector.
#define PCM5122_ERROR_DET_IGNORE_MCLK 0x08

#define PCM5122_PLL_REF_BCK 0x10

// Register 0x02 (STATE/Power Control): 0x00 = play, 0x10 = standby,
// 0x01 = powerdown.
#define PCM5122_STATE_PLAY    0x00
#define PCM5122_STATE_STANDBY 0x10
#define PCM5122_STATE_PWRDOWN 0x01

#define PCM5122_MUTE_BOTH 0x11
#define PCM5122_MUTE_NONE 0x00

// Analog Gain Control (P1-R2): LAGN/RAGN select 0 dB or -6 dB analog gain
#define PCM5122_ANALOG_GAIN_LAGN (1 << 4)
#define PCM5122_ANALOG_GAIN_RAGN (1 << 0)
#if CONFIG_PCM5122_ANALOG_GAIN_MINUS_6DB
#define PCM5122_ANALOG_GAIN_VAL \
  (PCM5122_ANALOG_GAIN_LAGN | PCM5122_ANALOG_GAIN_RAGN)
#define PCM5122_ANALOG_GAIN_IS_MINUS_6DB 1
#else
#define PCM5122_ANALOG_GAIN_VAL 0x00
#define PCM5122_ANALOG_GAIN_IS_MINUS_6DB 0
#endif

// DAC Data Path / Mixer (P0-R42): stereo, left -> left, right -> right
#define PCM5122_MIXER_STEREO 0x11

// DVOL register: 0x00 = +24 dB, 0x30 = 0 dB, 0xFE = -103 dB (0.5 dB/step).
// 0xFF is a dedicated mute code, not a volume level, so writes stay off it.
#define PCM5122_DVOL_0DB     0x30
#define PCM5122_DVOL_MIN_REG 0xFE
#define PCM5122_DVOL_MAX_REG 0x00

static const char TAG[] = "PCM5122 DAC";

static i2c_master_dev_handle_t s_dac_device = NULL;
static SemaphoreHandle_t s_dac_mutex = NULL;
static int s_current_page = -1; // -1 = unknown, forces a page-select write

// A soft MCU reset (RTS toggle, OTA reboot, panic recovery) leaves the
// PCM5122 continuously powered — there is no reset pin wired to it on this
// board — so a transaction the MCU abandoned mid-byte can leave the part's
// I2C state machine wedged. Give each raw write a couple of short retries
// before giving up, the same defensive pattern dac_es8311 uses.
static esp_err_t pcm5122_i2c_write_retry(uint8_t reg, uint8_t val) {
  esp_err_t err = ESP_OK;
  for (int attempt = 0; attempt < 5; attempt++) {
    err = board_i2c_write(s_dac_device, reg, &val, 1);
    if (err == ESP_OK) {
      return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return err;
}

static esp_err_t pcm5122_select_page(uint8_t page) {
  if (s_current_page == page) {
    return ESP_OK;
  }
  esp_err_t err = pcm5122_i2c_write_retry(PCM5122_REG_PAGE_SELECT, page);
  if (err != ESP_OK) {
    s_current_page = -1;
    return err;
  }
  s_current_page = page;
  return ESP_OK;
}

static esp_err_t pcm5122_write(uint8_t page, uint8_t reg, uint8_t val) {
  esp_err_t err = pcm5122_select_page(page);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Page select %u failed: %s", page, esp_err_to_name(err));
    return err;
  }
  err = pcm5122_i2c_write_retry(reg, val);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Write p%u/reg 0x%02X=0x%02X failed: %s", page, reg, val,
             esp_err_to_name(err));
  }
  return err;
}

static esp_err_t pcm5122_init(void *i2c_bus) {
  i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)i2c_bus;
  const uint8_t addr = CONFIG_PCM5122_I2C_ADDR;
  esp_err_t err;

  if (s_dac_mutex == NULL) {
    s_dac_mutex = xSemaphoreCreateMutex();
    if (s_dac_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create DAC mutex");
      return ESP_ERR_NO_MEM;
    }
  }

  // Confirm the part answers before touching any registers.
  err = i2c_master_probe(bus, addr, 100);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "No PCM5122 detected at 0x%02X: %s", addr,
             esp_err_to_name(err));
    return err;
  }

  err = board_i2c_add_device(bus, addr, I2C_LINE_SPEED, &s_dac_device);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add PCM5122 to I2C bus: %s",
             esp_err_to_name(err));
    return err;
  }
  s_current_page = -1;

  // Full reset. RSTM|RSTR (both bits) resets the whole digital block, not
  // just the audio modules — the part needs ~100ms to settle on each side.
  err = pcm5122_write(0, PCM5122_REG_RESET, PCM5122_RESET_FULL);
  if (err != ESP_OK) {
    goto fail;
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  err = pcm5122_write(0, PCM5122_REG_RESET, 0x00);
  if (err != ESP_OK) {
    goto fail;
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  err = pcm5122_write(0, PCM5122_REG_MUTE, PCM5122_MUTE_NONE);
  if (err != ESP_OK) {
    goto fail;
  }
  err = pcm5122_write(0, PCM5122_REG_MIXER, PCM5122_MIXER_STEREO);
  if (err != ESP_OK) {
    goto fail;
  }
  // I2S, 16-bit — the only format this firmware's I2S output ever runs.
  err = pcm5122_write(0, PCM5122_REG_AUDIO_FMT, PCM5122_AUDIO_FMT_I2S_16BIT);
  if (err != ESP_OK) {
    goto fail;
  }
  err = pcm5122_write(0, PCM5122_REG_PLL_REF, PCM5122_PLL_REF_BCK);
  if (err != ESP_OK) {
    goto fail;
  }
  err = pcm5122_write(0, PCM5122_REG_ERROR_DET, PCM5122_ERROR_DET_IGNORE_MCLK);
  if (err != ESP_OK) {
    goto fail;
  }

  // Analog gain: 0 dB (2V RMS) or -6 dB (1V RMS), per Kconfig.
  err = pcm5122_write(1, PCM5122_REG_ANALOG_GAIN, PCM5122_ANALOG_GAIN_VAL);
  if (err != ESP_OK) {
    goto fail;
  }

  err = pcm5122_write(0, PCM5122_REG_DVOL_LEFT, PCM5122_DVOL_0DB);
  if (err != ESP_OK) {
    goto fail;
  }
  err = pcm5122_write(0, PCM5122_REG_DVOL_RIGHT, PCM5122_DVOL_0DB);
  if (err != ESP_OK) {
    goto fail;
  }

  // Leave the actual play/standby/powerdown state to pcm5122_set_power_mode()
  // — the board puts the part into DAC_POWER_OFF right after dac_init().
  ESP_LOGI(TAG, "PCM5122 initialized at 0x%02X (analog gain %s)", addr,
           PCM5122_ANALOG_GAIN_IS_MINUS_6DB ? "-6 dB" : "0 dB");
  return ESP_OK;

fail:
  // Any failure past this point leaves the part in an unknown state — drop
  // the device so later set_volume()/set_power_mode()/enable_speaker() calls
  // don't keep hitting I2C against a half-configured DAC.
  board_i2c_remove_device(s_dac_device);
  s_dac_device = NULL;
  return err;
}

static esp_err_t pcm5122_deinit(void) {
  if (s_dac_device) {
    esp_err_t err = board_i2c_remove_device(s_dac_device);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to remove from I2C bus: %s", esp_err_to_name(err));
    }
    s_dac_device = NULL;
  }
  if (s_dac_mutex != NULL) {
    vSemaphoreDelete(s_dac_mutex);
    s_dac_mutex = NULL;
  }
  return ESP_OK;
}

static void pcm5122_set_power_mode(dac_power_mode_t mode) {
  xSemaphoreTake(s_dac_mutex, portMAX_DELAY);
  switch (mode) {
  case DAC_POWER_ON:
    // Exit powerdown before unmuting, so nothing pops on the way up.
    pcm5122_write(0, PCM5122_REG_STATE, PCM5122_STATE_PLAY);
    pcm5122_write(0, PCM5122_REG_MUTE, PCM5122_MUTE_NONE);
    ESP_LOGD(TAG, "Power mode: PLAY");
    break;
  case DAC_POWER_STANDBY:
  case DAC_POWER_OFF:
    // Whenever the part isn't actively playing, fully power it down rather
    // than just standby — mute first so powerdown itself is silent.
    pcm5122_write(0, PCM5122_REG_MUTE, PCM5122_MUTE_BOTH);
    pcm5122_write(0, PCM5122_REG_STATE, PCM5122_STATE_PWRDOWN);
    ESP_LOGD(TAG, "Power mode: PWRDOWN (requested %s)",
             mode == DAC_POWER_STANDBY ? "STANDBY" : "OFF");
    break;
  default:
    ESP_LOGW(TAG, "Unhandled power mode: %d", mode);
    break;
  }
  xSemaphoreGive(s_dac_mutex);
}

static void pcm5122_set_volume(float volume_airplay_db) {
  xSemaphoreTake(s_dac_mutex, portMAX_DELAY);

  // Clamp AirPlay input to -30..0 dB, as the other DAC drivers do.
  if (volume_airplay_db > 0.0f) {
    volume_airplay_db = 0.0f;
  } else if (volume_airplay_db < -30.0f) {
    volume_airplay_db = -30.0f;
  }

  // Map AirPlay -30..0 dB below the configured ceiling, then into the
  // PCM5122's 0.5 dB/step register (0x30 = 0 dB).
  float target_db = (float)CONFIG_PCM5122_MAX_VOLUME + volume_airplay_db;
  if (target_db < -103.0f) {
    target_db = -103.0f;
  } else if (target_db > 24.0f) {
    target_db = 24.0f;
  }

  int reg = PCM5122_DVOL_0DB - (int)lroundf(target_db * 2.0f);
  if (reg < PCM5122_DVOL_MAX_REG) {
    reg = PCM5122_DVOL_MAX_REG;
  } else if (reg > PCM5122_DVOL_MIN_REG) {
    reg = PCM5122_DVOL_MIN_REG;
  }

  ESP_LOGD(TAG, "Volume: %.1f dB AirPlay -> %.1f dB DAC -> reg 0x%02X",
           volume_airplay_db, target_db, (uint8_t)reg);

  pcm5122_write(0, PCM5122_REG_DVOL_LEFT, (uint8_t)reg);
  pcm5122_write(0, PCM5122_REG_DVOL_RIGHT, (uint8_t)reg);
  xSemaphoreGive(s_dac_mutex);
}

static void pcm5122_enable_speaker(bool enable) {
  xSemaphoreTake(s_dac_mutex, portMAX_DELAY);
  pcm5122_write(0, PCM5122_REG_MUTE,
               enable ? PCM5122_MUTE_NONE : PCM5122_MUTE_BOTH);
  xSemaphoreGive(s_dac_mutex);
}

static void pcm5122_enable_line_out(bool enable) {
  (void)enable;
  ESP_LOGW(TAG, "Line out not supported");
}

const dac_ops_t dac_pcm5122_ops = {
    .init = pcm5122_init,
    .deinit = pcm5122_deinit,
    .set_volume = pcm5122_set_volume,
    .set_power_mode = pcm5122_set_power_mode,
    .on_i2s_started = NULL,
    .enable_speaker = pcm5122_enable_speaker,
    .enable_line_out = pcm5122_enable_line_out,
};
