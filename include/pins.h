// ============================================================
// RF.Guru LoRa 433 APRS Tracker - Hardware Pin Definitions
// RP2040 GPIO Mapping
// ============================================================

#pragma once

// LEDs
#define PIN_LED_PWR     9
#define PIN_LED_GPS     10
#define PIN_LED_LORA    11

// Power Amplifier
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

// ADC Voltage
#define PIN_ADC_PA      27   // with PA
#define PIN_ADC_NOPA    26   // without PA
