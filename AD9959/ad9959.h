/*
 * ad9959.h
 *
 *  Created on: Jun 1, 2021
 *      Author: wiagn
 */

#ifndef __AD9959_H
#define __AD9959_H

#include "main.h"

#define DDS_MAX_PRECISION

typedef enum
{
  SINGLE_TONE,
  MODULATION,
  LINEAR_SWEEP,
  OUTPUT_AMPLITUDE_CONTROL
} AD9959_Mode;

typedef enum
{
  Channel_None = 0x00,
  Channel_0 = 0x10,
  Channel_1 = 0x20,
  Channel_2 = 0x40,
  Channel_3 = 0x80,
  Channel_All = 0xF0,
} AD9959_Channel;

typedef enum
{                // There are 334 bytes in all the registers.
  CSR = 0x00,    // 1 byte, Channel Select Register
  FR1 = 0x01,    // 3 bytes, Function Register 1
  FR2 = 0x02,    // 2 bytes, Function Register 2
                 // The following registers are duplicated for each channel.
                 // A write goes to any and all registers enabled in channel select (CSR)
                 // To read successfully you must first select one channel
  CFR = 0x03,    // 3 bytes, Channel Function Register (one for each channel!)
  CFTW = 0x04,   // 4 bytes, Channel Frequency Tuning Word
  CPOW = 0x05,   // 2 bytes, Channel Phase Offset Word (aligned to LSB, top 2 bits unused)
  ACR = 0x06,    // 3 bytes, Amplitude Control Register (rate byte, control byte, scale byte)
  LSRR = 0x07,   // 2 bytes, Linear Sweep Rate Register (falling, rising)
  RDW = 0x08,    // 4 bytes, Rising Delta Word
  FDW = 0x09,    // 4 bytes, Falling Delta Word
                 // The following registers (per channel) are used to provide 16 modulation values
                 // This library doesn't provide modulation. Only CW1 is used, for sweep destination.
  CW1 = 0x0A,    // 4 bytes, Channel Word 1-15 (phase & amplitude MSB aligned)
  CW2 = 0x0B,
  CW3 = 0x0C,
  CW4 = 0x0D,
  CW5 = 0x0E,
  CW6 = 0x0F,
  CW7 = 0x10,
  CW8 = 0x11,
  CW9 = 0x12,
  CW10 = 0x13,
  CW11 = 0x14,
  CW12 = 0x15,
  CW13 = 0x16,
  CW14 = 0x17,
  CW15 = 0x18
} AD9959_Register;

typedef enum
{
  // Bit order selection (default MSB):
  MSB_First = 0x00,
  LSB_First = 0x01,
  // Serial I/O Modes (default IO2Wire):
  IO2Wire = 0x00,
  IO3Wire = 0x02,
  IO2Bit = 0x04,
  IO4Bit = 0x06,
} AD9959_CSR_Bits;

typedef enum
{
  // Function Register 1 is 3 bytes wide.
  // Most significant byte:
  // Higher charge pump values decrease lock time and increase phase noise
  ChargePump0 = 0x00,
  ChargePump1 = 0x01,
  ChargePump2 = 0x02,
  ChargePump3 = 0x03,

  PllDivider = 0x04,    // multiply 4..20 by this (or shift 19)
  VCOGain = 0x80,    // Set low for VCO<160MHz, high for >255MHz

  // Middle byte:
  ModLevels2 = 0x00,    // How many levels of modulation?
  ModLevels4 = 0x01,
  ModLevels8 = 0x02,
  ModLevels16 = 0x03,

  RampUpDownOff = 0x00,    // Which pins contol amplitude ramping?
  RampUpDownP2P3 = 0x04,    // Profile=0 means ramp-up, 1 means ramp-down
  RampUpDownP3 = 0x08,    // Profile=0 means ramp-up, 1 means ramp-down
  RampUpDownSDIO123 = 0x0C,    // Only in 1-bit I/O mode

  Profile0 = 0x00,
  Profile7 = 0x07,

  // Least significant byte:
  SyncAuto = 0x00,    // Master SYNC_OUT->Slave SYNC_IN, with FR2
  SyncSoft = 0x01,    // Each time this is set, system clock slips one cycle
  SyncHard = 0x02,    // Synchronise devices by slipping on SYNC_IN signal

  // Software can power-down individual channels (using CFR[7:6])
  DACRefPwrDown = 0x10,    // Power-down DAC reference
  SyncClkDisable = 0x20,    // Don't output SYNC_CLK
  ExtFullPwrDown = 0x40,    // External power-down means full power-down (DAC&PLL)
  RefClkInPwrDown = 0x80,    // Disable reference clock input
} AD9959_FR1_Bits;

typedef enum
{
  AllChanAutoClearSweep = 0x8000,  // Clear sweep accumulator(s) on I/O_UPDATE
  AllChanClearSweep = 0x4000,  // Clear sweep accumulator(s) immediately
  AllChanAutoClearPhase = 0x2000,  // Clear phase accumulator(s) on I/O_UPDATE
  AllChanClearPhase = 0x2000,  // Clear phase accumulator(s) immediately
  AutoSyncEnable = 0x0080,
  MasterSyncEnable = 0x0040,
  MasterSyncStatus = 0x0020,
  MasterSyncMask = 0x0010,
  SystemClockOffset = 0x0003,         // Mask for 2-bit clock offset controls
} AD9959_FR2_Bits;

// Channel Function Register
typedef enum
{
  ModulationMode = 0xC00000,        // Mask for modulation mode
  AmplitudeModulation = 0x400000,     // Mask for modulation mode
  FrequencyModulation = 0x800000,     // Mask for modulation mode
  PhaseModulation = 0xC00000,        // Mask for modulation mode
  SweepNoDwell = 0x008000,        // No dwell mode
  SweepEnable = 0x004000,        // Enable the sweep
  SweepStepTimerExt = 0x002000,       // Reset the sweep step timer on I/O_UPDATE
  DACFullScale = 0x000300,        // 1/8, 1/4, 1/2 or full DAC current
  DigitalPowerDown = 0x000080,        // Power down the DDS core
  DACPowerDown = 0x000040,        // Power down the DAC
  MatchPipeDelay = 0x000020,        // Compensate for pipeline delays
  AutoclearSweep = 0x000010,        // Clear the sweep accumulator on I/O_UPDATE
  ClearSweep = 0x000008,        // Clear the sweep accumulator immediately
  AutoclearPhase = 0x000004,        // Clear the phase accumulator on I/O_UPDATE
  ClearPhase = 0x000002,        // Clear the phase accumulator immediately
  OutputSineWave = 0x000001,        // default is cosine
} AD9959_CFR_Bits;

// Amplitude Control Register
typedef enum
{
  RampRate = 0xFF0000,     // Time between ramp steps
  StepSize = 0x00C000,     // Amplitude step size (00=1,01=2,10=4,11=8)
  MultiplierEnable = 0x001000,     // 0 means bypass the amplitude multiplier
  RampEnable = 0x000800,     // 0 means aplitude control is manual
  LoadARRAtIOUpdate = 0x000400,     // Reload Amplitude Rate Register at I/O Update
  ScaleFactor = 0x0003FF,     // 10 bits for the amplitude target
} AD9959_ACR_Bits;

typedef struct
{

} AD9959_Channel_Handler;

typedef struct
{
  SPI_HandleTypeDef *SPI_handler;

  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;

  GPIO_TypeDef *mrst_port;
  uint16_t mrst_pin;

  GPIO_TypeDef *update_port;
  uint16_t update_pin;

  uint32_t crystal_clock;
  uint32_t calibration_factor;
  uint8_t multiplier;
  uint64_t core_clock;

  AD9959_CSR_Bits IO_mode;
  AD9959_Channel last_channels;
} AD9959_Handler;

void AD9959_Init(AD9959_Handler *device, SPI_HandleTypeDef *hspi, GPIO_TypeDef *device_cs_port, uint16_t device_cs_pin, GPIO_TypeDef *device_mrst_port, uint16_t device_mrst_pin, GPIO_TypeDef *device_update_port,
    uint16_t device_update_pin, uint32_t device_crystal_clock, uint32_t device_calibration);
void AD9959_Reset(AD9959_Handler *device, AD9959_CFR_Bits device_cfr);
void AD9959_SetClock(AD9959_Handler *device, uint8_t mult, int32_t calibration);
void AD9959_SetChannels(AD9959_Handler *device, AD9959_Channel chan);
uint32_t AD9959_GenerateFrequencyDelta(AD9959_Handler *device, uint32_t freq);
void AD9959_SetChannelFrequency(AD9959_Handler *device, AD9959_Channel chan, uint32_t freq);
void AD9959_SetChannelDelta(AD9959_Handler *device, AD9959_Channel chan, uint32_t delta);
void AD9959_SetPhase(AD9959_Handler *device, AD9959_Channel chan, uint16_t phase);
void AD9959_SetAmplitude(AD9959_Handler *device, AD9959_Channel chan, uint16_t amplitude);

void AD9959_SweepFrequency(AD9959_Handler *device, AD9959_Channel chan, uint32_t freq, uint8_t follow) ;
void AD9959_SweepDelta(AD9959_Handler *device, AD9959_Channel chan, uint32_t delta, uint8_t follow);
void AD9959_SweepRates(AD9959_Handler *device, AD9959_Channel chan, uint32_t increment, uint8_t up_rate, uint32_t decrement, uint8_t down_rate);
void AD9959_SweepAmplitude(AD9959_Handler *device, AD9959_Channel chan, uint16_t amplitude, uint8_t follow);
void AD9959_SweepPhase(AD9959_Handler *device, AD9959_Channel chan, uint16_t phase, uint8_t follow);

#endif /* __AD9959_H */
