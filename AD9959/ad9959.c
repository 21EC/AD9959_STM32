/*
 * ad9959.c
 *
 *  Created on: Jun 1, 2021
 *      Author: wiagn
 */

// CubeMX/IDE SPI settings : nSS disable; MSB first; CPOL Low; CPHA 1 Edge
// SPI baudrate can reach max 200 Mbps (SCLK 200 Mhz)
// For higher baudrate, Use QSPI (800 Msps max)
// Some codes are based on https://github.com/cjheath/AD9959

// Beware that editing specific channel's parameter would enable that channel

#include "ad9959.h"
#include <limits.h>

#ifndef DDS_MAX_PRECISION
static uint32_t reciprocal;             // 2^(64-shift)/core_clock
static uint8_t shift;                  // (2<<shift) < core_clock, but just (28 or less)
#endif

static void AD9959_Write(AD9959_Handler *device, AD9959_Register reg, uint32_t value);
//static uint32_t AD9959_Read(AD9959_Handler *device, AD9959_Register reg);
static void AD9959_Update(AD9959_Handler *device);

void AD9959_Init(AD9959_Handler *device, SPI_HandleTypeDef *hspi, GPIO_TypeDef *device_cs_port, uint16_t device_cs_pin, GPIO_TypeDef *device_mrst_port,
    uint16_t device_mrst_pin, GPIO_TypeDef *device_update_port, uint16_t device_update_pin, uint32_t device_crystal_clock, uint32_t device_calibration)
{
  // Setup
  device->SPI_handler = hspi;
  device->cs_port = device_cs_port;
  device->cs_pin = device_cs_pin;
  device->mrst_port = device_mrst_port;
  device->mrst_pin = device_mrst_pin;
  device->update_port = device_update_port;
  device->update_pin = device_update_pin;
  device->crystal_clock = device_crystal_clock;
  device->calibration_factor = device_calibration;
  switch (device->SPI_handler->Init.Direction)
  {
  case SPI_DIRECTION_2LINES:
    device->IO_mode = IO3Wire;
    break;
  case SPI_DIRECTION_1LINE:
    device->IO_mode = IO2Wire;
    break;
  }

  // Reset device
  AD9959_Reset(device, 0);

  return;
}

void AD9959_Reset(AD9959_Handler *device, AD9959_CFR_Bits device_cfr)
{
  // Set device_cfr to 0 to use default settings
  if (device_cfr == 0)
  {
    device_cfr = DACFullScale | MatchPipeDelay | OutputSineWave | AutoclearPhase | AutoclearSweep;
  }

  // HW reset
  HAL_GPIO_WritePin(device->mrst_port, device->mrst_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(device->mrst_port, device->mrst_pin, GPIO_PIN_RESET);

  // Register reset
  AD9959_SetChannels(device, Channel_All);
  AD9959_Write(device, CFR, device_cfr);
  AD9959_SetChannels(device, Channel_None);
  AD9959_SetClock(device, 0, device->calibration_factor);
  AD9959_Update(device);
  return;
}

// The calibration parameter supports frequency calibration.
// Set it first to the default 0 (no adjustment), and set the chip some known frequency and measure the actual frequency.
// Convert the error to parts-per-billion (positive if your frequency is high, negative if it's low).
// Provide this value as your new calibration value. For example if you program 10MHz, but measure 10000043.2Hz,
// your calibration factor should be 4320.
void AD9959_SetClock(AD9959_Handler *device, uint8_t mult, int32_t calibration) // Mult must be 0 or in range 4..20
{
  uint32_t transfer_data = 0;
  if (mult < 4 || mult > 20)
    mult = 1;                         // Multiplier is disabled.

  device->multiplier = mult;
  device->calibration_factor = calibration;
//  device->core_clock = device->crystal_clock * (1000000000 + calibration) / 1000000000ULL * mult; // 没啥用，VCO打不开
//  device->core_clock = device->crystal_clock * (1000000000 + calibration) * mult / 1000000000ULL; // 直接爆炸，频率跑飞
  device->core_clock = device->crystal_clock * ((uint64_t)1000000000 + calibration) * mult / (uint64_t)1000000000; // 正常

#ifndef DDS_MAX_PRECISION
  uint64_t scaled = device->core_clock;
  for (shift = 32; shift > 0 && (scaled & 0x100000000) == 0; shift--)
    scaled <<= 1;                   // Ensure that reciprocal fits in 32 bits
  reciprocal = (0x1ULL << (32 + shift)) / device->core_clock;
#endif
  // High VCO Gain is needed for a 255-500MHz master clock, and not up to 160Mhz
  // In-between is unspecified.
  transfer_data |= ((device->core_clock > 200000000 ? VCOGain : 0) | (mult * PllDivider) | ChargePump3) << 16; // Lock fast
  // Profile0 means each channel is modulated by a different profile pin:
  transfer_data |= (ModLevels2 | RampUpDownOff | Profile0) << 8;
  transfer_data |= SyncClkDisable; // Don't output SYNC_CLK
  AD9959_Write(device, FR1, transfer_data);
  return;
}

void AD9959_SetChannels(AD9959_Handler *device, AD9959_Channel chan)
{
  if (device->last_channels != chan)
    AD9959_Write(device, CSR, chan | MSB_First | device->IO_mode);
  device->last_channels = chan;
  return;
}

// Calculating deltas is expensive. You might use this infrequently and then use setDelta
uint32_t AD9959_GenerateFrequencyDelta(AD9959_Handler *device, uint32_t freq)
{
#ifdef DDS_MAX_PRECISION
//  double ret;
//  ret = ((double)(ULONG_MAX + 1)) / (double)(device->core_clock) * (double)freq;
//  return (uint32_t)ret;
  uint64_t ret;
  ret = ((uint64_t)ULONG_MAX + 1) * (uint64_t)freq / (device->core_clock);
  return (uint32_t)(ret);

#else
  // The reciprocal/16 is a rounding factor determined experimentally
  return ((uint64_t) freq * reciprocal + reciprocal / 16) >> shift;
#endif
}

// Only works when channel frequency is over 100KHz, or output wave would be wrong(Device limit)
void AD9959_SetChannelFrequency(AD9959_Handler *device, AD9959_Channel chan, uint32_t freq)
{
  AD9959_SetChannelDelta(device, chan, AD9959_GenerateFrequencyDelta(device, freq));
}

void AD9959_SetChannelDelta(AD9959_Handler *device, AD9959_Channel chan, uint32_t delta)
{
  AD9959_SetChannels(device, chan);
  AD9959_Write(device, CFTW, delta);
  AD9959_Update(device);
}

void AD9959_SetPhase(AD9959_Handler *device, AD9959_Channel chan, uint16_t phase)
{
  // Maximum phase value is 16383 (14-bit)
  AD9959_SetChannels(device, chan);
  AD9959_Write(device, CPOW, phase & 0x3FFF);        // Phase wraps around anyway
  AD9959_Update(device);
  return;
}

void AD9959_SetAmplitude(AD9959_Handler *device, AD9959_Channel chan, uint16_t amplitude)
{
  uint32_t transfer_acr = 0;

  // Maximum amplitude value is 1024 (10-bit)
  if (amplitude > 1024)
    amplitude = 1024;                 // Clamp to the maximum

  if (amplitude < 1024)               // Enable amplitude control with no ramping
    transfer_acr |= MultiplierEnable;

  transfer_acr |= amplitude & 0x3FF;  // 10 bits

  AD9959_SetChannels(device, chan);
  AD9959_Write(device, ACR, transfer_acr);
  AD9959_Update(device);
  return;
}

void AD9959_SweepFrequency(AD9959_Handler *device, AD9959_Channel chan, uint32_t freq, uint8_t follow)       // Target frequency
{
    AD9959_SweepDelta(device, chan, AD9959_GenerateFrequencyDelta(device, freq), follow);
}

void AD9959_SweepDelta(AD9959_Handler *device, AD9959_Channel chan, uint32_t delta, uint8_t follow)
{
  AD9959_SetChannels(device, chan);
  // Set up for frequency sweep
  AD9959_Write(device, CFR, (FrequencyModulation | SweepEnable | DACFullScale | MatchPipeDelay | ((follow == 1) ? 0 : SweepNoDwell)));
  // Write the frequency delta into the sweep destination register
  AD9959_Write(device, CW1, delta);
  AD9959_Update(device);
}

void AD9959_SweepRates(AD9959_Handler *device, AD9959_Channel chan, uint32_t increment, uint8_t up_rate, uint32_t decrement, uint8_t down_rate)
{
  AD9959_SetChannels(device, chan);
  AD9959_Write(device, RDW, increment);                      // Rising Sweep Delta Word
  AD9959_Write(device, FDW, increment);                      // Falling Sweep Delta Word
  AD9959_Write(device, LSRR, (down_rate<<8) | up_rate);      // Linear Sweep Ramp Rate
  AD9959_Update(device);
}

void AD9959_SweepAmplitude(AD9959_Handler *device, AD9959_Channel chan, uint16_t amplitude, uint8_t follow)  // Target amplitude (half)
{
  AD9959_SetChannels(device, chan);
  // Set up for amplitude sweep
  AD9959_Write(device, CFR, (AmplitudeModulation | SweepEnable |DACFullScale | MatchPipeDelay | ((follow == 1) ? 0 : SweepNoDwell)));
  // Write the amplitude into the sweep destination register, MSB aligned
  AD9959_Write(device, CW1, ((uint32_t)amplitude) * (0x1<<(32-10)));
  AD9959_Update(device);
}

void AD9959_SweepPhase(AD9959_Handler *device, AD9959_Channel chan, uint16_t phase, uint8_t follow)          // Target phase (180 degrees)
{
  AD9959_SetChannels(device, chan);
  // Set up for phase sweep
  AD9959_Write(device, CFR, (PhaseModulation | SweepEnable | DACFullScale | MatchPipeDelay | ((follow == 1) ? 0 : SweepNoDwell)));
  // Write the phase into the sweep destination register, MSB aligned
  AD9959_Write(device, CW1, ((uint32_t)phase) * (0x1<<(32-14)));
  AD9959_Update(device);
}

// Send I/O Update signal to apply changes, however setting enabled channels doesn't need an I/O update
static void AD9959_Update(AD9959_Handler *device)
{
  HAL_GPIO_WritePin(device->update_port, device->update_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(device->update_port, device->update_pin, GPIO_PIN_RESET);
  return;
}

// Write data "value" to "reg" register
static void AD9959_Write(AD9959_Handler *device, AD9959_Register reg, uint32_t value)
{
  // The indices of this array match the values of the Register enum:
  const uint8_t register_length[8] =
  { 1, 3, 2, 3, 4, 2, 3, 2 };  // And 4 beyond that

  uint8_t i, transfer_data[4];

  // Get length of target register as byte
  uint8_t len = (reg & 0x7F) < sizeof(register_length) / sizeof(uint8_t) ? register_length[reg & 0x07] : 4;

  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(device->SPI_handler, &reg, sizeof(reg), HAL_MAX_DELAY);

  // For MSB first mode
  i = 0;
  while (len-- > 0)
  {
    transfer_data[i] = (value >> len * 8) & 0xFF;
    i++;
  }

  HAL_SPI_Transmit(device->SPI_handler, transfer_data, i, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
  return;
}

// unfinished
/*
static uint32_t AD9959_Read(AD9959_Handler *device, AD9959_Register reg)
{
  uint32_t transdata, recivdata;
  transdata = (reg | 0x80);
  HAL_SPI_TransmitReceive(device->SPI_handler, &transdata, &recivdata, Size, Timeout)
  return AD9959_Write(device, 0x80 | reg, 0);  // The zero data is discarded, just the return value is used
}
*/
