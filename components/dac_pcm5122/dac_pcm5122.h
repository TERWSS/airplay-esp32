#pragma once

#include "dac.h"

/**
 * PCM5122 DAC driver ops — register with dac_register() before calling
 * dac_init().
 */
extern const dac_ops_t dac_pcm5122_ops;
