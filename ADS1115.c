/*
 * Based on Texas Instruments ADS1113/4/5 datasheet, May 2009 (SBAS444B, revised October 2009)
 * Copied/modified from:
 *   - https://github.com/addicore/ADS1115
 *   - https://github.com/Molorius/esp32-ads1115
 *   - https://github.com/jrowberg/i2cdevlib
 */

#include "ADS1115.h"
#include "esp_log.h"
#include "esp_task_wdt.h"  // esp_task_wdt_reset()
#include "driver/i2c.h"

#define TAG_EXTERNAL_ADC  "ADS1115"

static BYTE s_byDevAddr = ADS1115_DEFAULT_ADDRESS; //  I2C Address
static WORD s_awBuffer[2];
static BOOL s_bDevMode;
static BYTE s_byMuxMode;
static BYTE s_byPGAMode;
static BYTE s_byPinAlrt;
static BYTE s_byLastReg;

/** Poll the assigned pin for conversion status 
 */
void ads1115_pollAlertReadyPin() 
{
  DWORD i;
  for (i = 0; i<100000*10; i++)
  {
    esp_task_wdt_reset();
    if (!READ_BUTTON(s_byPinAlrt)) 
    {
      //printf("got conformation, i=%d\n", i);
      return;
    }
  }
  ESP_LOGI(TAG_EXTERNAL_ADC, "Failed to wait for AlertReadyPin, it's stuck high!");
}

void ads1115_i2c_master_init(BYTE bySDA_pin, BYTE bySCL_pin)
{
	i2c_config_t i2c_config = 
	{
		.mode = I2C_MODE_MASTER,
		.sda_io_num = bySDA_pin,
		.scl_io_num = bySCL_pin,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

/** Read single word from a 16-bit device register.
 * @param byRegAddr Register byRegAddr to read from
 * @param pwData Container for word value read from device
 * @return esp_err_t of read operation
 */
esp_err_t ads1115_readWord(BYTE byRegAddr, WORD* pwData) 
{
  BYTE byOut[2];
  esp_err_t espRc = 0;
  i2c_cmd_handle_t cmd;

  // check if last register that was written to or read from is the same as this one
  if (s_byLastReg != byRegAddr)
  {
    cmd = i2c_cmd_link_create();
	  i2c_master_start(cmd);
    // first write to Address Pointer register to set the register we want to read from
	  i2c_master_write_byte(cmd, (s_byDevAddr << 1) | I2C_MASTER_WRITE, true);
	  i2c_master_write_byte(cmd, byRegAddr, TRUE);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    s_byLastReg = byRegAddr;
    if(espRc != ESP_OK)
    {
      return espRc;
    }
  }
  
  // Start to read from register (register to be read from is written Address Pointer Register (in ADS1115))
  cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (s_byDevAddr << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, byOut, 2, I2C_MASTER_ACK);
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

  // first byte transmitted from ADS1115 is most significant byte from the register
  *pwData = byOut[0] << 8 | byOut[1];  // switch bytes

  return espRc;
}

/** Read a single bit from a 16-bit device register.
 * @param byRegAddr Register byRegAddr to read from
 * @param byBitNum Bit position to read (0-15)
 * @param pwData Container for single bit value
 * @return esp_err_t Status of read operation
 */
esp_err_t ads1115_readBitInWord(BYTE byRegAddr, BYTE byBitNum, WORD* pwData) 
{
  WORD wReadWord;
  esp_err_t espRc = ads1115_readWord(byRegAddr, &wReadWord);
  *pwData = wReadWord & (1 << byBitNum);
  return espRc;
}

/** Read multiple bits from a 16-bit device register.
 * @param byRegAddr Register byRegAddr to read from
 * @param byBitStart First bit position to read (0-15)
 * @param byLength Number of bits to read (not more than 16)
 * @param pwData Container for right-aligned value (i.e. '101' read from any byBitStart position will equal 0x05)
 * @return esp_err_t Status of read operation
 */
esp_err_t ads1115_readBitsInWord(BYTE byRegAddr, BYTE byBitStart, BYTE byLength, WORD* pwData)
{
  // 1101011001101001 read byte
  // fedcba9876543210 bit numbers
  //    xxx           args: byBitStart=12, byLength=3
  //    010           masked
  //           -> 010 shifted
  esp_err_t espRc;
  WORD wReadWord;
  WORD wMask;

  espRc = ads1115_readWord(byRegAddr, &wReadWord);
  if (espRc == ESP_OK)
  {
    wMask = ((1 << byLength) - 1) << (byBitStart - byLength + 1);
    wReadWord &= wMask;
    wReadWord >>= (byBitStart - byLength + 1);
    *pwData = wReadWord;
  }
  return espRc;
}

/** Write single word to a 16-bit device register.
 * @param byRegAddr Register address to write to
 * @param pwData New word value to write
 * @return esp_err_t Status of operation (true = success)
 */
esp_err_t ads1115_writeWord(BYTE byRegAddr, WORD* pwData) 
{
  BYTE byOut[2];
  esp_err_t espRc;
  i2c_cmd_handle_t cmd;

  // first byte to be send goes in the most significant byte of the register
  byOut[0] = *pwData >> 8;    // get 8 greater bits
  byOut[1] = *pwData & 0xFF;  // get 8 lower bits

  cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (s_byDevAddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, byRegAddr, TRUE);
  i2c_master_write(cmd, byOut, 2, TRUE);
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

  s_byLastReg = byRegAddr;

  return espRc;
}

/** write a single bit in a 16-bit device register.
 * @param byRegAddr Register byRegAddr to write to
 * @param byBitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return esp_err_t Status of operation
 */
esp_err_t ads1115_writeBitInWord(BYTE byRegAddr, BYTE byBitNum, BOOL bData) 
{
  WORD wReadWord;
  esp_err_t espErr;
  
  espErr = ads1115_readWord(byRegAddr, &wReadWord);
  if(espErr == ESP_OK)
  {
    //                             set bit                          clear bit
    wReadWord = (bData != 0) ? (wReadWord | (1 << byBitNum)) : (wReadWord & ~(1 << byBitNum));
    return ads1115_writeWord(byRegAddr, &wReadWord);
  }
  else
  {
    return espErr;
  }
  
}

/** Write multiple bits in a 16-bit device register.
 * @param byRegAddr Register byRegAddr to write to
 * @param byBitStart First bit position to write (0-15)
 * @param byLength Number of bits to write (not more than 16)
 * @param wData Right-aligned value to write
 * @return esp_err_t Status of operation
 */                                
esp_err_t ads1115_writeBitsInWord(BYTE byRegAddr, BYTE byBitStart, BYTE byLength, WORD wData)
{
  //              010 value to write
  // fedcba9876543210 bit numbers
  //    xxx           args: byBitStart=12, byLength=3
  // 0001110000000000 wMask word
  // 1010111110010110 original value (sample)
  // 1010001110010110 original & ~wMask
  // 1010101110010110 masked | value
  esp_err_t espRc;
  WORD wReadWord;
  WORD wMask;

  espRc = ads1115_readWord(byRegAddr, &wReadWord);
  if (espRc == ESP_OK) 
  {
    wMask = ((1 << byLength) - 1) << (byBitStart - byLength + 1);
    wData <<= (byBitStart - byLength + 1); // shift data into correct position
    wData &= wMask; // zero all non-important bits in data
    wReadWord &= ~(wMask); // zero all important bits in existing word
    wReadWord |= wData; // combine data with existing word
    return ads1115_writeWord(byRegAddr, &wReadWord);
  }
  else
  {
    return espRc;
  }
}

/* I2C General Call
 *  The ADS111x respond to the I2C general call address (0000000) if the eighth bit is 0. The devices acknowledge
 *  the general call address and respond to commands in the second byte. If the second byte is 00000110 (06h), the
 *  ADS111x reset the internal registers and enter a power-down state.
*/
esp_err_t ads1115_reset() 
{
  esp_err_t espRc;
  i2c_cmd_handle_t cmd;

  cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (0x00 << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, ADS1115_COMMAND_RESET, TRUE);
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

  s_byLastReg = ADS1115_COMMAND_RESET;

  return espRc;
}

/** Default constructor, uses default I2C address.
 * @see ADS1115_DEFAULT_ADDRESS
 */
void ads1115_set_address_default() 
{
  s_byDevAddr = ADS1115_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see ADS1115_DEFAULT_ADDRESS
 * @see ADS1115_ADDRESS_ADDR_GND
 * @see ADS1115_ADDRESS_ADDR_VDD
 * @see ADS1115_ADDRESS_ADDR_SDA
 * @see ADS1115_ADDRESS_ADDR_SDL
 */
void ads1115_set_address(BYTE byAddress) 
{
  s_byDevAddr = byAddress;
}

void ads1115_set_alert_pin(BYTE byPinAlrt)
{
  s_byPinAlrt = byPinAlrt;
}

/** Power on and prepare for general usage.
 * This device is ready to use automatically upon power-up. It defaults to
 * single-shot read mode, P0/N1 mux, 2.048v gain, 128 samples/sec, default
 * comparator with hysterysis, active-low polarity, non-latching comparator,
 * and comparater-disabled operation. 
 */
void ads1115_setToDefault() 
{
  esp_err_t espErr;

  espErr = ads1115_setMultiplexer(ADS1115_MUX_P0_N1);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setToDefault: Error setting multiplexer: 0x%.2X", espErr);
  espErr = ads1115_setGain(ADS1115_PGA_2P048);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setToDefault: Error setting gain: 0x%.2X", espErr);
  espErr = ads1115_setMode(ADS1115_MODE_SINGLESHOT);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setToDefault: Error setting ADS mode: 0x%.2X", espErr);
  espErr = ads1115_setRate(ADS1115_DATA_RATE_128);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setToDefault: Error setting data rate: 0x%.2X", espErr);
  espErr = ads1115_setComparatorMode(ADS1115_COMP_MODE_HYSTERESIS);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setToDefault: Error setting comparator mode: 0x%.2X", espErr);
  espErr = ads1115_setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_LOW);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setToDefault: Error setting comparator polarity: 0x%.2X", espErr);
  espErr = ads1115_setComparatorLatchEnabled(ADS1115_COMP_LAT_NON_LATCHING);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setToDefault: Error setting comparator latch: 0x%.2X", espErr);
  espErr = ads1115_setComparatorQueueMode(ADS1115_COMP_QUE_DISABLE);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setToDefault: Error setting comparator queue mode: 0x%.2X", espErr);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return esp_err_t
 */
esp_err_t ads1115_testConnection() 
{
  return ads1115_readWord(ADS1115_RA_CONVERSION, s_awBuffer);
}

/** Poll the operational status bit until the conversion is finished
 * Retry at most 'max_retries' times
 * conversion is finished, then return true;
 * @see ADS1115_CFG_OS_BIT
 * @return True if data is available, false otherwise
 */
BOOL ads1115_pollConversion(WORD wMax_retries) 
{  
  for(WORD i = 0; i < wMax_retries; i++) 
  {
    if (ads1115_isConversionReady()) 
    {
      return TRUE;
    }
  }
  return FALSE;
}

/** Read differential value based on current MUX configuration.
 * The default MUX setting sets the device to get the differential between the
 * AIN0 and AIN1 pins. There are 8 possible MUX settings, but if you are using
 * all four input pins as single-end voltage sensors, then the default option is
 * not what you want; instead you will need to set the MUX to compare the
 * desired AIN* pin with GND. There are shortcut methods (getConversion*) to do
 * this conveniently, but you can also do it manually with setMultiplexer()
 * followed by this method.
 *
 * In single-shot mode, this register may not have fresh data. You need to write
 * a 1 bit to the MSB of the CONFIG register to trigger a single read/conversion
 * before this will be populated with fresh data. This technique is not as
 * effortless, but it has enormous potential to save power by only running the
 * comparison circuitry when needed.
 *
 * @param triggerAndPoll If true (and only in singleshot mode) the conversion trigger 
 *        will be executed and the conversion results will be polled.
 * @return 16-bit signed differential value
 * @see getConversionP0N1();
 * @see getConversionPON3();
 * @see getConversionP1N3();
 * @see getConversionP2N3();
 * @see getConversionP0GND();
 * @see getConversionP1GND();
 * @see getConversionP2GND();
 * @see getConversionP3GND);
 * @see setMultiplexer();
 * @see ADS1115_RA_CONVERSION
 * @see ADS1115_MUX_P0_N1
 * @see ADS1115_MUX_P0_N3
 * @see ADS1115_MUX_P1_N3
 * @see ADS1115_MUX_P2_N3
 * @see ADS1115_MUX_P0_NG
 * @see ADS1115_MUX_P1_NG
 * @see ADS1115_MUX_P2_NG
 * @see ADS1115_MUX_P3_NG
 */
esp_err_t ads1115_getConversion(BOOL bTriggerAndPoll, SHORT *pshData) 
{
	esp_err_t espErr;

  if (bTriggerAndPoll && s_bDevMode == ADS1115_MODE_SINGLESHOT) 
  {
    ads1115_triggerConversion();
    ads1115_pollConversion(I2CDEV_DEFAULT_READ_TIMEOUT);
  }

  espErr = ads1115_readWord(ADS1115_RA_CONVERSION, s_awBuffer);
  *pshData = s_awBuffer[0];

  return espErr;
}
/** Get AIN0/N1 differential.
 * This changes the MUX setting to AIN0/N1 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
esp_err_t ads1115_getConversionP0N1(SHORT *pshData)
{
  if (s_byMuxMode != ADS1115_MUX_P0_N1) 
  {
    ads1115_setMultiplexer(ADS1115_MUX_P0_N1);
  }
  return ads1115_getConversion(GET_CONVERSION_READ_ONLY, pshData);
}

/** Get AIN0/N3 differential.
 * This changes the MUX setting to AIN0/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
esp_err_t ads1115_getConversionP0N3(SHORT *pshData) 
{
    if (s_byMuxMode != ADS1115_MUX_P0_N3)
    {
      ads1115_setMultiplexer(ADS1115_MUX_P0_N3);
    } 
    return ads1115_getConversion(GET_CONVERSION_READ_ONLY, pshData);
}

/** Get AIN1/N3 differential.
 * This changes the MUX setting to AIN1/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
esp_err_t ads1115_getConversionP1N3(SHORT *pshData) 
{
    if (s_byMuxMode != ADS1115_MUX_P1_N3)
    {
      ads1115_setMultiplexer(ADS1115_MUX_P1_N3);
    } 
    return ads1115_getConversion(GET_CONVERSION_READ_ONLY, pshData);
}

/** Get AIN2/N3 differential.
 * This changes the MUX setting to AIN2/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
esp_err_t ads1115_getConversionP2N3(SHORT *pshData) 
{
    if (s_byMuxMode != ADS1115_MUX_P2_N3)
    {
      ads1115_setMultiplexer(ADS1115_MUX_P2_N3);
    }
    return ads1115_getConversion(GET_CONVERSION_READ_ONLY, pshData);
}

/** Get AIN0/GND differential.
 * This changes the MUX setting to AIN0/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
esp_err_t ads1115_getConversionP0GND(SHORT *pshData) 
{
    if (s_byMuxMode != ADS1115_MUX_P0_NG) 
    {
      ads1115_setMultiplexer(ADS1115_MUX_P0_NG);
    }
    return ads1115_getConversion(GET_CONVERSION_READ_ONLY, pshData);
}
/** Get AIN1/GND differential.
 * This changes the MUX setting to AIN1/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
esp_err_t ads1115_getConversionP1GND(SHORT *pshData) 
{
    if (s_byMuxMode != ADS1115_MUX_P1_NG)
    {
      ads1115_setMultiplexer(ADS1115_MUX_P1_NG);
    }
    return ads1115_getConversion(GET_CONVERSION_READ_ONLY, pshData);
}
/** Get AIN2/GND differential.
 * This changes the MUX setting to AIN2/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
esp_err_t ads1115_getConversionP2GND(SHORT *pshData) 
{
    if (s_byMuxMode != ADS1115_MUX_P2_NG) 
    {
      ads1115_setMultiplexer(ADS1115_MUX_P2_NG);
    }
    return ads1115_getConversion(GET_CONVERSION_READ_ONLY, pshData);
}
/** Get AIN3/GND differential.
 * This changes the MUX setting to AIN3/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
esp_err_t ads1115_getConversionP3GND(SHORT *pshData) 
{
    if (s_byMuxMode != ADS1115_MUX_P3_NG) 
    {
      ads1115_setMultiplexer(ADS1115_MUX_P3_NG);
    }
    return ads1115_getConversion(GET_CONVERSION_READ_ONLY, pshData);
}

/** Get the current voltage reading
 * Read the current differential and return it multiplied
 * by the constant for the current gain.  mV is returned to
 * increase the precision of the voltage
 * @param bTriggerAndPoll If true (and only in singleshot mode) the conversion trigger 
 *        will be executed and the conversion results will be polled.
 */
esp_err_t ads1115_getMilliVolts(BOOL bTriggerAndPoll, float *pfData)
{
  SHORT shData = 0;
  esp_err_t espErr = ESP_OK;

  switch (s_byPGAMode) 
  { 
    case ADS1115_PGA_6P144:
      espErr = ads1115_getConversion(bTriggerAndPoll, &shData);
      *pfData = shData * ADS1115_MV_6P144;
      break;    
    case ADS1115_PGA_4P096:
      espErr = ads1115_getConversion(bTriggerAndPoll, &shData);
      *pfData = shData * ADS1115_MV_4P096;
      break;             
    case ADS1115_PGA_2P048:    
      espErr = ads1115_getConversion(bTriggerAndPoll, &shData);
      *pfData = shData * ADS1115_MV_2P048;
      break;       
    case ADS1115_PGA_1P024:     
      espErr = ads1115_getConversion(bTriggerAndPoll, &shData);
      *pfData = shData * ADS1115_MV_1P024;
      break;       
    case ADS1115_PGA_0P512:      
      espErr = ads1115_getConversion(bTriggerAndPoll, &shData);
      *pfData = shData * ADS1115_MV_0P512;
      break;       
    case ADS1115_PGA_0P256:           
    case ADS1115_PGA_0P256B:          
    case ADS1115_PGA_0P256C:      
      espErr = ads1115_getConversion(bTriggerAndPoll, &shData);
      *pfData = shData * ADS1115_MV_0P256;
      break;       
  }
  return espErr;
}

/**
 * Return the current multiplier for the PGA setting.
 * 
 * This may be directly retreived by using getMilliVolts(),
 * but this causes an independent read.  This function could
 * be used to average a number of reads from the getConversion()
 * getConversionx() functions and cut downon the number of 
 * floating-point calculations needed.
 *
 */
 
float ads1115_getMvPerCount() 
{
  float fReturn = FALSE;
  switch (s_byPGAMode) 
  {
    case ADS1115_PGA_6P144:
      fReturn = ADS1115_MV_6P144;
      break;    
    case ADS1115_PGA_4P096:
      fReturn =  ADS1115_MV_4P096;
      break;             
    case ADS1115_PGA_2P048:    
      fReturn = ADS1115_MV_2P048;
      break;       
    case ADS1115_PGA_1P024:     
      fReturn = ADS1115_MV_1P024;
      break;       
    case ADS1115_PGA_0P512:      
      fReturn = ADS1115_MV_0P512;
      break;       
    case ADS1115_PGA_0P256:           
    case ADS1115_PGA_0P256B:          
    case ADS1115_PGA_0P256C:      
      fReturn = ADS1115_MV_0P256;
      break;       
  }
  return fReturn;
}

// CONFIG register

/** Get operational status.
 * @return Current operational status (false for active conversion, true for inactive)
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_OS_BIT
 */
BOOL ads1115_isConversionReady() 
{
  ads1115_readBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_OS_BIT, s_awBuffer);
  return s_awBuffer[0];
}

/** Trigger a new conversion.
 * Writing to this bit will only have effect while in power-down mode (no conversions active).
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_OS_BIT
 */
esp_err_t ads1115_triggerConversion() 
{
  return ads1115_writeBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_OS_BIT, 1);
}

/** Get multiplexer connection.
 * @return Current multiplexer connection setting
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MUX_BIT
 * @see ADS1115_CFG_MUX_LENGTH
 */
BYTE ads1115_getMultiplexer() 
{
  ads1115_readBitsInWord(ADS1115_RA_CONFIG, ADS1115_CFG_MUX_BIT, ADS1115_CFG_MUX_LENGTH, s_awBuffer);
  s_byMuxMode = (BYTE)s_awBuffer[0];
  return s_byMuxMode;
}

/** Set multiplexer connection.  Continous mode may fill the conversion register
 * with data before the MUX setting has taken effect.  A stop/start of the conversion
 * is done to reset the values.
 * @param mux New multiplexer connection setting
 * @see ADS1115_MUX_P0_N1
 * @see ADS1115_MUX_P0_N3
 * @see ADS1115_MUX_P1_N3
 * @see ADS1115_MUX_P2_N3
 * @see ADS1115_MUX_P0_NG
 * @see ADS1115_MUX_P1_NG
 * @see ADS1115_MUX_P2_NG
 * @see ADS1115_MUX_P3_NG
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MUX_BIT
 * @see ADS1115_CFG_MUX_LENGTH
 */
esp_err_t ads1115_setMultiplexer(BYTE byMux) 
{
  SHORT shStartStopDummy;
  esp_err_t espErr; 

  espErr = ads1115_writeBitsInWord(ADS1115_RA_CONFIG, ADS1115_CFG_MUX_BIT, ADS1115_CFG_MUX_LENGTH, byMux);
  if (espErr == ESP_OK) 
  {
    s_byMuxMode = byMux;
    if (s_bDevMode == ADS1115_MODE_CONTINUOUS) 
    {
      // Force a stop/start, it must happen if in continuous mode
      espErr = ads1115_setMode(ADS1115_MODE_SINGLESHOT);
      if(espErr != ESP_OK)
      {
        return espErr;
      }
      ads1115_getConversion(GET_CONVERSION_READ_ONLY, &shStartStopDummy);
      espErr = ads1115_setMode(ADS1115_MODE_CONTINUOUS);
      if(espErr != ESP_OK)
      {
        return espErr;
      }
    }
  }  
    
  return espErr;  
}

/** Get programmable gain amplifier level.
 * @return Current programmable gain amplifier level
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_PGA_BIT
 * @see ADS1115_CFG_PGA_LENGTH
 */
BYTE ads1115_getGain() 
{
  ads1115_readBitsInWord(ADS1115_RA_CONFIG, ADS1115_CFG_PGA_BIT, ADS1115_CFG_PGA_LENGTH, s_awBuffer);
  s_byPGAMode = (BYTE)s_awBuffer[0];
  return s_byPGAMode;
}

/** Set programmable gain amplifier level.  
 * Continous mode may fill the conversion register
 * with data before the gain setting has taken effect.  A stop/start of the conversion
 * is done to reset the values.
 * @param gain New programmable gain amplifier level
 * @see ADS1115_PGA_6P144
 * @see ADS1115_PGA_4P096
 * @see ADS1115_PGA_2P048
 * @see ADS1115_PGA_1P024
 * @see ADS1115_PGA_0P512
 * @see ADS1115_PGA_0P256
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_PGA_BIT
 * @see ADS1115_CFG_PGA_LENGTH
 */
esp_err_t ads1115_setGain(BYTE byGain) 
{
  SHORT shStartStopDummy;
  esp_err_t espErr;

  espErr = ads1115_writeBitsInWord(ADS1115_RA_CONFIG, ADS1115_CFG_PGA_BIT, ADS1115_CFG_PGA_LENGTH, byGain);
  if (espErr == ESP_OK) 
  {
    s_byPGAMode = byGain;
    if (s_bDevMode == ADS1115_MODE_CONTINUOUS) 
    {
     // Force a stop/start
     espErr = ads1115_setMode(ADS1115_MODE_SINGLESHOT);
     if(espErr != ESP_OK) return espErr;
     ads1115_getConversion(GET_CONVERSION_READ_ONLY, &shStartStopDummy);
     espErr = ads1115_setMode(ADS1115_MODE_CONTINUOUS);
    }
  }

  return espErr;
}

/** Get device mode.
 * @return Current device mode
 * @see ADS1115_MODE_CONTINUOUS
 * @see ADS1115_MODE_SINGLESHOT
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MODE_BIT
 */
BOOL ads1115_getMode() 
{
  ads1115_readBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_MODE_BIT, s_awBuffer);
  s_bDevMode = s_awBuffer[0];
  return s_bDevMode;
}

/** Set device mode.
 * @param mode New device mode
 * @see ADS1115_MODE_CONTINUOUS
 * @see ADS1115_MODE_SINGLESHOT
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MODE_BIT
 */
esp_err_t ads1115_setMode(BOOL bMode)
{
  esp_err_t espErr = ads1115_writeBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_MODE_BIT, bMode);
  if (espErr == ESP_OK) 
  {
    s_bDevMode = bMode;
  }
  return espErr;
}

/** Get data rate.
 * @return Current data rate
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_DR_BIT
 * @see ADS1115_CFG_DR_LENGTH
 */
BYTE ads1115_getRate() 
{
  ads1115_readBitsInWord(ADS1115_RA_CONFIG, ADS1115_CFG_DR_BIT, ADS1115_CFG_DR_LENGTH, s_awBuffer);
  return (BYTE)s_awBuffer[0];
}

/** Set data rate.
 * @param rate New data rate
 * @see ADS1115_DATA_RATE_8
 * @see ADS1115_DATA_RATE_16
 * @see ADS1115_DATA_RATE_32
 * @see ADS1115_DATA_RATE_64
 * @see ADS1115_DATA_RATE_128
 * @see ADS1115_DATA_RATE_250
 * @see ADS1115_DATA_RATE_475
 * @see ADS1115_DATA_RATE_860
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_DR_BIT
 * @see ADS1115_CFG_DR_LENGTH
 */
esp_err_t ads1115_setRate(BYTE byRate) 
{
  return ads1115_writeBitsInWord(ADS1115_RA_CONFIG, ADS1115_CFG_DR_BIT, ADS1115_CFG_DR_LENGTH, byRate);
}

/** Get comparator mode.
 * @return Current comparator mode
 * @see ADS1115_COMP_MODE_HYSTERESIS
 * @see ADS1115_COMP_MODE_WINDOW
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_MODE_BIT
 */
BOOL ads1115_getComparatorMode()
{
  ads1115_readBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_MODE_BIT, s_awBuffer);
  return s_awBuffer[0];
}

/** Set comparator mode.
 * @param mode New comparator mode
 * @see ADS1115_COMP_MODE_HYSTERESIS
 * @see ADS1115_COMP_MODE_WINDOW
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_MODE_BIT
 */
esp_err_t ads1115_setComparatorMode(BOOL bMode)
{
  return ads1115_writeBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_MODE_BIT, bMode);
}

/** Get comparator polarity setting.
 * @return Current comparator polarity setting
 * @see ADS1115_COMP_POL_ACTIVE_LOW
 * @see ADS1115_COMP_POL_ACTIVE_HIGH
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_POL_BIT
 */
BOOL ads1115_getComparatorPolarity()
{
  ads1115_readBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_POL_BIT, s_awBuffer);
  return s_awBuffer[0];
}

/** Set comparator polarity setting.
 * @param polarity New comparator polarity setting
 * @see ADS1115_COMP_POL_ACTIVE_LOW
 * @see ADS1115_COMP_POL_ACTIVE_HIGH
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_POL_BIT
 */
esp_err_t ads1115_setComparatorPolarity(BOOL bPolarity) 
{
  return ads1115_writeBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_POL_BIT, bPolarity);
}

/** Get comparator latch enabled value.
 * @return Current comparator latch enabled value
 * @see ADS1115_COMP_LAT_NON_LATCHING
 * @see ADS1115_COMP_LAT_LATCHING
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_LAT_BIT
 */
BOOL ads1115_getComparatorLatchEnabled()
{
  ads1115_readBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_LAT_BIT, s_awBuffer);
  return s_awBuffer[0];
}

/** Set comparator latch enabled value.
 * @param enabled New comparator latch enabled value
 * @see ADS1115_COMP_LAT_NON_LATCHING
 * @see ADS1115_COMP_LAT_LATCHING
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_LAT_BIT
 */
esp_err_t ads1115_setComparatorLatchEnabled(BOOL bEnabled)
{
  return ads1115_writeBitInWord(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_LAT_BIT, bEnabled);
}

/** Get comparator queue mode.
 * @return Current comparator queue mode
 * @see ADS1115_COMP_QUE_ASSERT1
 * @see ADS1115_COMP_QUE_ASSERT2
 * @see ADS1115_COMP_QUE_ASSERT4
 * @see ADS1115_COMP_QUE_DISABLE
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_QUE_BIT
 * @see ADS1115_CFG_COMP_QUE_LENGTH
 */
BYTE ads1115_getComparatorQueueMode()
{
  ads1115_readBitsInWord(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_QUE_BIT, ADS1115_CFG_COMP_QUE_LENGTH, s_awBuffer);
  return (BYTE)s_awBuffer[0];
}

/** Set comparator queue mode.
 * @param mode New comparator queue mode
 * @see ADS1115_COMP_QUE_ASSERT1
 * @see ADS1115_COMP_QUE_ASSERT2
 * @see ADS1115_COMP_QUE_ASSERT4
 * @see ADS1115_COMP_QUE_DISABLE
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_QUE_BIT
 * @see ADS1115_CFG_COMP_QUE_LENGTH
 */
esp_err_t ads1115_setComparatorQueueMode(BYTE byMode)
{
  return ads1115_writeBitsInWord(ADS1115_RA_CONFIG, ADS1115_CFG_COMP_QUE_BIT, ADS1115_CFG_COMP_QUE_LENGTH, byMode);
}

// *_THRESH registers

/** Get low threshold value.
 * @return Current low threshold value
 * @see ADS1115_RA_LO_THRESH
 */
SHORT ads1115_getLowThreshold()
{
    ads1115_readWord(ADS1115_RA_LO_THRESH, s_awBuffer);
    return s_awBuffer[0];
}

/** Set low threshold value.
 * @param threshold New low threshold value
 * @see ADS1115_RA_LO_THRESH
 */
void ads1115_setLowThreshold(SHORT shThreshold) 
{
  WORD wHelper = (WORD)shThreshold;
  ads1115_writeWord(ADS1115_RA_LO_THRESH, &wHelper);
}

/** Get high threshold value.
 * @return Current high threshold value
 * @see ADS1115_RA_HI_THRESH
 */
SHORT ads1115_getHighThreshold()
{
  ads1115_readWord(ADS1115_RA_HI_THRESH, s_awBuffer);
  return s_awBuffer[0];
}

/** Set high threshold value.
 * @param threshold New high threshold value
 * @see ADS1115_RA_HI_THRESH
 */
void ads1115_setHighThreshold(SHORT shThreshold)
{
  WORD wHelper = (WORD)shThreshold;
  ads1115_writeWord(ADS1115_RA_HI_THRESH, &wHelper);
}

/** Configures ALERT/RDY pin as a conversion ready pin.
 *  It does this by setting the MSB of the high threshold register to '1' and the MSB 
 *  of the low threshold register to '0'. COMP_POL and COMP_QUE bits will be set to '0'.
 *  Note: ALERT/RDY pin requires a pull up resistor.
 */
void ads1115_setConversionReadyPinMode()
{
  esp_err_t espErr;
  espErr = ads1115_writeBitInWord(ADS1115_RA_HI_THRESH, 15, 1);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setConversionReadyPinMode: writeBitInWord (ADS1115_RA_HI_THRESH) Error: 0x%.2X", espErr);
  espErr = ads1115_writeBitInWord(ADS1115_RA_LO_THRESH, 15, 0);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setConversionReadyPinMode: writeBitInWord (ADS1115_RA_LO_THRESH) Error: 0x%.2X", espErr);
  espErr = ads1115_setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_LOW);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setConversionReadyPinMode: setComparatorPolarity Error: 0x%.2X", espErr);
  espErr = ads1115_setComparatorQueueMode(ADS1115_COMP_QUE_ASSERT1);
  if(espErr != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setConversionReadyPinMode: setComparatorQueueMode Error: 0x%.2X", espErr);
}

// Create a wMask between two bits
WORD ads1115_createMask(WORD a, WORD b) 
{
  WORD wMask = 0;
  for (WORD i=a; i<=b; i++)
  {
    wMask |= 1 << i;
  }
  return wMask;
}

WORD ads1115_shiftDown(WORD wExtractFrom, LONG lPlaces) 
{
  return (wExtractFrom >> lPlaces);
}


WORD ads1115_getValueFromBits(WORD wExtractFrom, LONG lHigh, LONG lLength)
{
  LONG lLow= lHigh-lLength +1;
  WORD wMask = ads1115_createMask(lLow ,lHigh);
  return ads1115_shiftDown(wExtractFrom & wMask, lLow); 
}

/*
 * Show all the config register settings
 */
void ads1115_showConfigRegister()
{
  ads1115_readWord(ADS1115_RA_CONFIG, s_awBuffer);
  WORD wConfigRegister = s_awBuffer[0];    
    
#ifdef ADS1115_SERIAL_DEBUG
  printf("Register is: 0x%X\n", wConfigRegister);
  printf("OS: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_OS_BIT,1));
  printf("MUX: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_MUX_BIT, ADS1115_CFG_MUX_LENGTH));    
  printf("PGA: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_PGA_BIT, ADS1115_CFG_PGA_LENGTH));  
  printf("MODE: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_MODE_BIT, 1));
  printf("DR: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_DR_BIT, ADS1115_CFG_DR_LENGTH));
  printf("CMP_MODE: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_COMP_MODE_BIT, 1));
  printf("CMP_POL: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_COMP_POL_BIT, 1));
  printf("CMP_LAT: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_COMP_LAT_BIT, 1));
  printf("CMP_QUE: 0x%X\n", ads1115_getValueFromBits(wConfigRegister, ADS1115_CFG_COMP_QUE_BIT, \
                                                     ADS1115_CFG_COMP_QUE_LENGTH));
#endif
};