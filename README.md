# AD9959_STM32

## How to use

Connection:

AD9959|Connection
---|---
SCK|STM32-SCK
CS|GPIO-CS
UPDT|GPIO-UPDT
MRST|GPIO-MRST
PDC|Must be GND
SD0|STM32-MOSI
SD1|Float
SD2|Float
SD3|Must be GND
P0|GPIO-P0 when used
P1|GPI1-P0 when used
P2|GPI2-P0 when used
P3|GPI3-P0 when used

Implementation:

```cpp
AD9959_Handler u_ad9959;
AD9959_Init(&u_ad9959, &hspi1, ADI_CS_GPIO_Port, ADI_CS_Pin, ADI_RST_GPIO_Port, ADI_RST_Pin, ADI_UPDT_GPIO_Port, ADI_UPDT_Pin, 25000000, 0);
AD9959_SetClock(&u_ad9959, 0, 0);
AD9959_SetChannelFrequency(&u_ad9959, Channel_All, 10000);

// Sweep, use tim1 pwm out to trig Profile Pin
// AD9959_SweepFrequency(&u_ad9959, Channel_All, 50000000, 0) ;
// AD9959_SweepRates(&u_ad9959, Channel_All, 5000000, 125, 5000000, 125);

```
