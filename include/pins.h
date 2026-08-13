// ============================================================
// RF.Guru LoRa 433 APRS Tracker - Hardware Pin Definitions
// RP2040 GPIO Mapping
// ============================================================

#pragma once

// LEDs
#define PIN_LED_PWR     9
#define PIN_LED_GPS     10
#define PIN_LED_LORA    11

// Power amplifier enable.
// V1: the external 13.8 V amplifier. V2: no external amplifier exists -
// the MiniF27's own PA feeds a low-pass filter and the SMA directly -
// but the line still has to be asserted around a transmission, so the
// radio layer treats "a PA is present" as reason enough to drive it.
#define PIN_PA          2

// I2C Sensor Power
#define PIN_I2C_PWR     3

// GPS UART
#define PIN_GPS_TX      4
#define PIN_GPS_RX      5
#define PIN_GPS_RST     12

// I2C Bus
#define PIN_I2C_SDA     6
#define PIN_I2C_SCL     7

// SPI (LoRa)
#define PIN_SPI_MISO    16
#define PIN_SPI_CLK     18
#define PIN_SPI_MOSI    19
#define PIN_LORA_CS     21
#define PIN_LORA_RST    20

// SX126x BUSY handshake line.
// V1 boards (RFM95/SX1276) do not use it; V2 boards (SX1262) wire it to
// GP22. Verified on hardware with tools/chipprobe.
#define PIN_LORA_BUSY   22

// Radio interrupt line: DIO0 on SX127x, DIO1 on SX126x.
// Neither board revision routes it to the RP2040, so the driver polls the
// chip's IRQ status register instead. Set to a GPIO if a future revision
// wires it up.
#define PIN_LORA_DIO    -1

// ADC Voltage
#define PIN_ADC_PA      27   // with PA
#define PIN_ADC_NOPA    26   // without PA

// GP15 is held low on the board. It was believed to be a V2 revision
// strap, but it measures low on V1 hardware too, so it identifies
// nothing and the firmware does not read it - the board revision comes
// from the radio, over SPI. Left documented so nobody wires it into a
// decision again, and so nobody drives it as an output without first
// finding out what it is connected to.
// #define PIN_BOARD_ID    15
