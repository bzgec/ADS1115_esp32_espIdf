/*
 * Based on Texas Instruments ADS1113/4/5 datasheet, May 2009 (SBAS444B, revised October 2009)
 * Copied/modified from:
 *   - https://github.com/addicore/ADS1115
 *   - https://github.com/Molorius/esp32-ads1115
 *   - https://github.com/jrowberg/i2cdevlib
 */

#ifndef __ADS1115_h
#define __ADS1115_h

#include "Types.h"
#include "esp_err.h"

// -----------------------------------------------------------------------------
// Arduino-style "printf" debug constant (uncomment to enable)
// -----------------------------------------------------------------------------
//#define ADS1115_SERIAL_DEBUG

#define ADS1115_RA_CONVERSION       0x00
#define ADS1115_RA_CONFIG           0x01
#define ADS1115_RA_LO_THRESH        0x02
#define ADS1115_RA_HI_THRESH        0x03
#define ADS1115_COMMAND_RESET       0x06

//  I2C Address
#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND) 
#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin
#define ADS1115_DEFAULT_ADDRESS     ADS1115_ADDRESS_ADDR_GND

#define ADS1115_CFG_OS_BIT          15
#define ADS1115_CFG_MUX_BIT         14
#define ADS1115_CFG_MUX_LENGTH      3
#define ADS1115_CFG_PGA_BIT         11
#define ADS1115_CFG_PGA_LENGTH      3
#define ADS1115_CFG_MODE_BIT        8
#define ADS1115_CFG_DR_BIT          7
#define ADS1115_CFG_DR_LENGTH       3
#define ADS1115_CFG_COMP_MODE_BIT   4
#define ADS1115_CFG_COMP_POL_BIT    3
#define ADS1115_CFG_COMP_LAT_BIT    2
#define ADS1115_CFG_COMP_QUE_BIT    1
#define ADS1115_CFG_COMP_QUE_LENGTH 2

// Input multiplexer configuration
#define ADS1115_MUX_P0_N1           0x00  // 000 : AINP = AIN0 and AINN = AIN1 (default) 
#define ADS1115_MUX_P0_N3           0x01  // 001 : AINP = AIN0 and AINN = AIN3
#define ADS1115_MUX_P1_N3           0x02  // 010 : AINP = AIN1 and AINN = AIN3
#define ADS1115_MUX_P2_N3           0x03  // 011 : AINP = AIN2 and AINN = AIN3
#define ADS1115_MUX_P0_NG           0x04  // 100 : AINP = AIN0 and AINN = GND
#define ADS1115_MUX_P1_NG           0x05  // 101 : AINP = AIN1 and AINN = GND
#define ADS1115_MUX_P2_NG           0x06  // 110 : AINP = AIN2 and AINN = GND
#define ADS1115_MUX_P3_NG           0x07  // 111 : AINP = AIN3 and AINN = GND

// Programmable gain amplifier configuration
#define ADS1115_PGA_6P144           0x00
#define ADS1115_PGA_4P096           0x01
#define ADS1115_PGA_2P048           0x02 // default
#define ADS1115_PGA_1P024           0x03
#define ADS1115_PGA_0P512           0x04
#define ADS1115_PGA_0P256           0x05
#define ADS1115_PGA_0P256B          0x06
#define ADS1115_PGA_0P256C          0x07

// voltage range in mV
#define ADS1115_PGA_VOLTAGE_RANGE_6P144  6144
#define ADS1115_PGA_VOLTAGE_RANGE_4P096  4096
#define ADS1115_PGA_VOLTAGE_RANGE_2P048  2048
#define ADS1115_PGA_VOLTAGE_RANGE_1P024  1024
#define ADS1115_PGA_VOLTAGE_RANGE_0P512  0512
#define ADS1115_PGA_VOLTAGE_RANGE_0P256  0256


/*
 * LSB = FSR / 2^16
 *
 * Table 3. Full-Scale Range and Corresponding LSB Size
 * ----------------------------------------
 * |    FSR           |     LSB SIZE      |
 * ----------------------------------------
 * |    ±6.144 V (1)  |     187.5 μV      |
 * |    ±4.096 V(1)   |     125 μV        |
 * |    ±2.048 V      |     62.5 μV       |
 * |    ±1.024 V      |     31.25 μV      |
 * |    ±0.512 V      |     15.625 μV     |
 * |    ±0.256 V      |     7.8125 μV     |
 * ----------------------------------------
 * (1) This parameter expresses the full-scale range of the ADC scaling.
 *     Do not apply more than VDD + 0.3 V to the analog inputs of the
 *     device.
 */
#define ADS1115_MV_6P144            0.187500
#define ADS1115_MV_4P096            0.125000
#define ADS1115_MV_2P048            0.062500 // default
#define ADS1115_MV_1P024            0.031250
#define ADS1115_MV_0P512            0.015625
#define ADS1115_MV_0P256            0.007813
#define ADS1115_MV_0P256B           0.007813 
#define ADS1115_MV_0P256C           0.007813

// Device operating mode
#define ADS1115_MODE_CONTINUOUS     0x00
#define ADS1115_MODE_SINGLESHOT     0x01 // default

// Data rate -> SPS - Samples per second
// Conversions in the ADS111x settle within a single cycle; thus, the conversion time is equal to 1 / DR.
//            SPS                                 conversion time
#define ADS1115_DATA_RATE_8              0x00  // 125.00 ms
#define ADS1115_DATA_RATE_16             0x01  // 62.500 ms
#define ADS1115_DATA_RATE_32             0x02  // 31.250 ms
#define ADS1115_DATA_RATE_64             0x03  // 15.625 ms
#define ADS1115_DATA_RATE_128            0x04  //  7.813 ms, default
#define ADS1115_DATA_RATE_250            0x05  //  4.000 ms
#define ADS1115_DATA_RATE_475            0x06  //  2.105 ms
#define ADS1115_DATA_RATE_860            0x07  //  1.163 ms

// Comparator mode
#define ADS1115_COMP_MODE_HYSTERESIS    0x00 // default
#define ADS1115_COMP_MODE_WINDOW        0x01

// Comparator polarity
#define ADS1115_COMP_POL_ACTIVE_LOW     0x00 // default
#define ADS1115_COMP_POL_ACTIVE_HIGH    0x01

/* Latching comparator
 *  This bit controls whether the ALERT/RDY pin latches after being asserted or
 *  clears after conversions are within the margin of the upper and lower threshold
 *  values. This bit serves no function on the ADS1113.
 */
#define ADS1115_COMP_LAT_NON_LATCHING   0x00 // default, the ALERT/RDY pin does not latch when asserted
#define ADS1115_COMP_LAT_LATCHING       0x01 // The asserted ALERT/RDY pin remains latched until 
                                             // conversion data are read by the master or an 
                                             // appropriate SMBus alert response is sent by the master.
                                             // The device responds with its address, and it is the lowest
                                             // address currently asserting the ALERT/RDY bus line.

/* Comparator queue and disable
 *  These bits perform two functions. When set to 11, the comparator is disabled and
 *  the ALERT/RDY pin is set to a high-impedance state. When set to any other
 *  value, the ALERT/RDY pin and the comparator function are enabled, and the set
 *  value determines the number of successive conversions exceeding the upper or
 *  lower threshold required before asserting the ALERT/RDY pin. These bits serve
 *  no function on the ADS1113.
 */
#define ADS1115_COMP_QUE_ASSERT1    0x00  // Assert after one conversion
#define ADS1115_COMP_QUE_ASSERT2    0x01  // Assert after two conversions
#define ADS1115_COMP_QUE_ASSERT4    0x02  // Assert after four conversions
#define ADS1115_COMP_QUE_DISABLE    0x03  // default, Disable comparator and set ALERT/RDY pin to high-impedance

#define GET_CONVERSION_READ_ONLY        0x00
#define GET_CONVERSION_TRIGGER_AND_POOL 0x01

#define I2CDEV_DEFAULT_READ_TIMEOUT     1000


#define ADS1115_SERIAL_DEBUG

#ifdef __cplusplus
extern "C" {
#endif

void ads1115_pollAlertReadyPin();
void ads1115_i2c_master_init(BYTE bySDA_pin, BYTE bySCL_pin);
esp_err_t ads1115_readWord(BYTE byRegAddr, WORD* pwData);
esp_err_t ads1115_readBitInWord(BYTE byRegAddr, BYTE byBitNum, WORD* pwData);
esp_err_t ads1115_readBitsInWord(BYTE byRegAddr, BYTE byBitStart, BYTE byLength, WORD* pwData);
esp_err_t ads1115_writeWord(BYTE byRegAddr, WORD* pwData);
esp_err_t ads1115_writeBitInWord(BYTE byRegAddr, BYTE byBitNum, BOOL bData);
esp_err_t ads1115_writeBitsInWord(BYTE byRegAddr, BYTE byBitStart, BYTE byLength, WORD wData);
esp_err_t ads1115_reset();
void ads1115_set_address_default();
void ads1115_set_address(BYTE byAddress);
void ads1115_set_alert_pin(BYTE byPinAlrt);
void ads1115_setToDefault();
esp_err_t ads1115_testConnection();
BOOL ads1115_pollConversion(WORD wMax_retries);
esp_err_t ads1115_getConversion(BOOL bTriggerAndPoll, SHORT *psData);
esp_err_t ads1115_getConversionP0N1(SHORT *psData);
esp_err_t ads1115_getConversionP0N3(SHORT *psData);
esp_err_t ads1115_getConversionP1N3(SHORT *psData);
esp_err_t ads1115_getConversionP2N3(SHORT *psData);
esp_err_t ads1115_getConversionP0GND(SHORT *psData);
esp_err_t ads1115_getConversionP1GND(SHORT *psData);
esp_err_t ads1115_getConversionP2GND(SHORT *psData);
esp_err_t ads1115_getConversionP3GND(SHORT *psData);
esp_err_t ads1115_getMilliVolts(BOOL bTriggerAndPoll, float *pfData);
float ads1115_getMvPerCount();
BOOL ads1115_isConversionReady();
esp_err_t ads1115_triggerConversion();
BYTE ads1115_getMultiplexer();
esp_err_t ads1115_setMultiplexer(BYTE byMux);
BYTE ads1115_getGain();
esp_err_t ads1115_setGain(BYTE byGain);
BOOL ads1115_getMode();
esp_err_t ads1115_setMode(BOOL bMode);
BYTE ads1115_getRate();
esp_err_t ads1115_setRate(BYTE byRate);
BOOL ads1115_getComparatorMode();
esp_err_t ads1115_setComparatorMode(BOOL bMode);
BOOL ads1115_getComparatorPolarity();
esp_err_t ads1115_setComparatorPolarity(BOOL bPolarity);
BOOL ads1115_getComparatorLatchEnabled();
esp_err_t ads1115_setComparatorLatchEnabled(BOOL bEnabled);
BYTE ads1115_getComparatorQueueMode();
esp_err_t ads1115_setComparatorQueueMode(BYTE byMode);
SHORT ads1115_getLowThreshold();
void ads1115_setLowThreshold(SHORT sThreshold);
SHORT ads1115_getHighThreshold();
void ads1115_setHighThreshold(SHORT sThreshold);
void ads1115_setConversionReadyPinMode();
WORD ads1115_createMask(WORD a, WORD b);
WORD ads1115_shiftDown(WORD wExtractFrom, LONG lPlaces);
WORD ads1115_getValueFromBits(WORD wExtractFrom, LONG lHigh, LONG lLength);
void ads1115_showConfigRegister();

#ifdef __cplusplus
}
#endif


// For polling the ALERT pin
#ifndef GPIO_REG_ADDR
#define GPIO_REG_ADDR 0x3FF44000
#endif  // GPIO_REG_ADDR
#ifndef GPIO_REG_IN_ADDR_OFFSET
#define GPIO_REG_IN_ADDR_OFFSET 0x003c
#endif  // GPIO_REG_IN_ADDR_OFFSET
#ifndef GPIO_REG_IN_ADDR
#define GPIO_REG_IN_ADDR (GPIO_REG_ADDR + GPIO_REG_IN_ADDR_OFFSET)
#endif  // GPIO_REG_IN_ADDR
#ifndef GPIO_REG_IN
#define GPIO_REG_IN (*((DWORD*)GPIO_REG_IN_ADDR))
#endif  // GPIO_REG_IN

#ifndef READ_BUTTON
#define READ_BUTTON(gpio_numb)  ((GPIO_REG_IN & (1 << (gpio_numb))) != 0)
#endif  // READ_BUTTON


#endif  // __ADS1115_h