//******************************************************************************

//    _____ _               _____                        _     _
//   |_   _| |             |  __ \                      | |   (_)
//     | | | |_   _  __ _  | |  | | ___ _ __ _   _  __ _| |__  _ _ __
//     | | | | | | |/ _` | | |  | |/ _ \ '__| | | |/ _` | '_ \| | '_ \
//    _| |_| | |_| | (_| | | |__| |  __/ |  | |_| | (_| | |_) | | | | |
//   |_____|_|\__, |\__,_| |_____/ \___|_|   \__, |\__,_|_.__/|_|_| |_|
//             __/ |                          __/ |
//            |___/                          |___/

//******************************************************************************



#include "stm32l0xx_hal.h"
#include "apds9960.h"



/* Members */
gesture_data_type gesture_data_;
int gesture_ud_delta_;
int gesture_lr_delta_;
int gesture_ud_count_;
int gesture_lr_count_;
int gesture_near_count_;
int gesture_far_count_;
int gesture_state_;
int gesture_motion_;


/*******************************************************************************
 * Public methods for controlling the APDS-9960
 ******************************************************************************/


uint8_t APDS9960_getMode(I2C_HandleTypeDef *hi2c, uint8_t * mode)
{
    /* Read current ENABLE register */
    return APDS9960_wireReadDataByte(hi2c, APDS9960_ENABLE, mode);
}


uint8_t APDS9960_setMode(I2C_HandleTypeDef *hi2c, uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;
    uint8_t result;

    /* Read current ENABLE register */
    result = APDS9960_getMode(hi2c, &reg_val);
    if( result == HAL_ERROR ) {
        return HAL_ERROR;
    }

    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if(mode <= 6 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }

    /* Write value back to ENABLE register */
    result =  APDS9960_wireWriteDataByte(hi2c, APDS9960_ENABLE, &reg_val);
    if( result == HAL_ERROR ) {
        return HAL_ERROR;
    }
    return HAL_OK;
}


/**
 * Turn the APDS-9960 on
 *
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_enablePower(I2C_HandleTypeDef *hi2c)
{

    return APDS9960_setMode(hi2c,POWER, 1);
}

/**
 * Turn the APDS-9960 off
 *
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_disablePower(I2C_HandleTypeDef *hi2c)
{
    return APDS9960_setMode(hi2c, POWER, 0);
}


/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 */
void APDS9960_resetGestureParameters()
{
    gesture_data_.index = 0;
    gesture_data_.total_gestures = 0;

    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;

    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;

    gesture_near_count_ = 0;
    gesture_far_count_ = 0;

    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}

/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/

/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t APDS9960_getProxIntLowThresh(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from PILT register */
    if( !APDS9960_wireReadDataByte(hi2c, APDS9960_PILT, &val) ) {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setProxIntLowThresh(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_PILT, &threshold) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t APDS9960_getProxIntHighThresh(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from PIHT register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_PIHT, &val) ) 
    {
        return HAL_ERROR;
    }

    return val;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setProxIntHighThresh(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_PIHT, &threshold) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t APDS9960_getLEDDrive(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONTROL, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Shift and mask out LED drive bits */
    val = val >> 6;
    val = val & 0x03;

    return val;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setLEDDrive(I2C_HandleTypeDef *hi2c, uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONTROL, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 6;
    val &= 0x6F;
    val |= drive;

    /* Write register value back into CONTROL register */
    if(HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONTROL, &val) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t APDS9960_getProximityGain(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONTROL, &val) )
    {
        return HAL_ERROR;
    }

    /* Shift and mask out PDRIVE bits */
    val = (val >> 2) & 0x03;

    return HAL_OK;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setProximityGain(I2C_HandleTypeDef *hi2c, uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONTROL, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 2;
    val &= 0xF3;
    val |= drive;

    /* Write register value back into CONTROL register */
    if(HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONTROL, &val) ) 
    {
        return HAL_ERROR;
    }

    return val;
}

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t APDS9960_getAmbientLightGain(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONTROL, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Shift and mask out ADRIVE bits */
    val &= 0x03;

    return val;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setAmbientLightGain(I2C_HandleTypeDef *hi2c, uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONTROL, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    drive &= 0x03;
    val &= 0xFC;
    val |= drive;

    /* Write register value back into CONTROL register */
    if(HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONTROL, &val) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Get the current LED boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @return The LED boost value. 0xFF on failure.
 */
uint8_t APDS9960_getLEDBoost(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from CONFIG2 register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONFIG2, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Shift and mask out LED_BOOST bits */
    val = (val >> 4) & 0x03;

    return val;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setLEDBoost(I2C_HandleTypeDef *hi2c, uint8_t boost)
{
    uint8_t val;

    /* Read value from CONFIG2 register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONFIG2, &val) )
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    boost &= 0x03;
    boost = boost << 4;
    val &= 0xCF;
    val |= boost;

    /* Write register value back into CONFIG2 register */
    if(HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONFIG2, &val) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets proximity gain compensation enable
 *
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getProxGainCompEnable(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONFIG3, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Shift and mask out PCMP bits */
    val = (val >> 5) & 0x01;

    return val;
}

/**
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @return True if operation successful. False otherwise.
 */
 uint8_t APDS9960_setProxGainCompEnable(I2C_HandleTypeDef *hi2c, uint8_t enable)
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONFIG3, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 5;
    val &= 0xDF;
    val |= enable;

    /* Write register value back into CONFIG3 register */
    if(HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONFIG3, &val) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @return Current proximity mask for photodiodes. 0xFF on error.
 */
uint8_t APDS9960_getProxPhotoMask(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONFIG3, &val) )
    {
        return HAL_ERROR;
    }

    /* Mask out photodiode enable mask bits */
    val &= 0x0F;

    return val;
}

/**
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] mask 4-bit mask value
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setProxPhotoMask(I2C_HandleTypeDef *hi2c, uint8_t mask)
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CONFIG3, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    mask &= 0x0F;
    val &= 0xF0;
    val |= mask;

    /* Write register value back into CONFIG3 register */
    if(HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONFIG3, &val) )
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets the entry proximity threshold for gesture sensing
 *
 * @return Current entry proximity threshold.
 */
uint8_t APDS9960_getGestureEnterThresh(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from GPENTH register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GPENTH, &val) ) 
    {
        return HAL_ERROR;
    }

    return val;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to start gesture mode
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setGestureEnterThresh(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GPENTH, &threshold) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets the exit proximity threshold for gesture sensing
 *
 * @return Current exit proximity threshold.
 */
uint8_t APDS9960_getGestureExitThresh(I2C_HandleTypeDef *hi2c, uint8_t * threshold)
{
    /* Read value from GEXTH register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GEXTH, threshold) ) 
    {
         return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to end gesture mode
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setGestureExitThresh(I2C_HandleTypeDef *hi2c, uint8_t * threshold)
{
    if(  HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GEXTH, threshold) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the current photodiode gain. 0xFF on error.
 */
uint8_t APDS9960_getGestureGain(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF2, &val) )
    {
        return HAL_ERROR;
    }

    /* Shift and mask out GGAIN bits */
    val = (val >> 5) & 0x03;

    return val;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setGestureGain(I2C_HandleTypeDef *hi2c, uint8_t gain)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF2, &val) )
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    gain &= 0x03;
    gain = gain << 5;
    val &= 0xAF;
    val |= gain;

    /* Write register value back into GCONF2 register */
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GCONF2, &val) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the LED drive current value. 0xFF on error.
 */
uint8_t APDS9960_getGestureLEDDrive(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF2, &val) ) 
    {
        return ERROR;
    }

    /* Shift and mask out GLDRIVE bits */
    val = (val >> 3) & 0x03;

    return val;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setGestureLEDDrive(I2C_HandleTypeDef *hi2c, uint8_t drive)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF2, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 3;
    val &= 0xE7;
    val |= drive;

    /* Write register value back into GCONF2 register */
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GCONF2, &val) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t APDS9960_getGestureWaitTime(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF2, &val) )
    {
        return ERROR;
    }

    /* Mask out GWTIME bits */
    val &= 0x07;

    return val;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setGestureWaitTime(I2C_HandleTypeDef *hi2c, uint8_t time)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF2, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    time &= 0x07;
    val &= 0xF8;
    val |= time;

    /* Write register value back into GCONF2 register */
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GCONF2, &val) )
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_getLightIntLowThreshold(I2C_HandleTypeDef *hi2c, uint16_t threshold)
{
    uint8_t val_byte;
    threshold = 0;

    /* Read value from ambient light low threshold, low byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_AILTL, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    threshold = val_byte;

    /* Read value from ambient light low threshold, high byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_AILTH, &val_byte) )
    {
        return HAL_ERROR;
    }
    threshold = threshold + ((uint16_t)val_byte << 8);

    return HAL_OK;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setLightIntLowThreshold(I2C_HandleTypeDef *hi2c, uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_AILTL, &val_low) )
    {
        return HAL_ERROR;
    }

    /* Write high byte */
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_AILTH, &val_high) )
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_getLightIntHighThreshold(I2C_HandleTypeDef *hi2c, uint16_t threshold)
{
    uint8_t val_byte;
    threshold = 0;

    /* Read value from ambient light high threshold, low byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_AIHTL, &val_byte) )
    {
        return HAL_ERROR;
    }
    threshold = val_byte;

    /* Read value from ambient light high threshold, high byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_AIHTH, &val_byte) )
    {
        return HAL_ERROR;
    }
    threshold = threshold + ((uint16_t)val_byte << 8);

    return HAL_OK;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setLightIntHighThreshold(I2C_HandleTypeDef *hi2c, uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_AIHTL, &val_low) ) 
    {
        return HAL_ERROR;
    }

    /* Write high byte */
    return APDS9960_wireWriteDataByte(hi2c, APDS9960_AIHTH, &val_high);

}

/**
 * @brief Gets the low threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_getProximityIntLowThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    threshold = 0;

    /* Read value from proximity low threshold register */
    return APDS9960_wireReadDataByte(hi2c, APDS9960_PILT, &threshold);
}

/**
 * @brief Sets the low threshold for proximity interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setProximityIntLowThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{

    /* Write threshold value to register */
    return APDS9960_wireWriteDataByte(hi2c, APDS9960_PILT, &threshold);
}

/**
 * @brief Gets the high threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_getProximityIntHighThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    threshold = 0;

    /* Read value from proximity low threshold register */
    return APDS9960_wireReadDataByte(hi2c, APDS9960_PIHT, &threshold);
}

/**
 * @brief Sets the high threshold for proximity interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setProximityIntHighThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{

    /* Write threshold value to register */
    return APDS9960_wireWriteDataByte(hi2c, APDS9960_PIHT, &threshold) ;
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getAmbientLightIntEnable(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_ENABLE, &val) ) 
    {
        return ERROR;
    }

    /* Shift and mask out AIEN bit */
    val = (val >> 4) & 0x01;

    return val;
}

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setAmbientLightIntEnable(I2C_HandleTypeDef *hi2c, uint8_t enable)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_ENABLE, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 4;
    val &= 0xEF;
    val |= enable;

    /* Write register value back into ENABLE register */
    return APDS9960_wireWriteDataByte(hi2c, APDS9960_ENABLE, &val);
}

/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getProximityIntEnable(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_ENABLE, &val) )
    {
        return ERROR;
    }

    /* Shift and mask out PIEN bit */
    val = (val >> 5) & 0x01;

    return val;
}

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setProximityIntEnable(I2C_HandleTypeDef *hi2c, uint8_t enable)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_ENABLE, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 5;
    val &= 0xDF;
    val |= enable;

    /* Write register value back into ENABLE register */
    return APDS9960_wireWriteDataByte(hi2c, APDS9960_ENABLE, &val);
}

/**
 * @brief Gets if gesture interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getGestureIntEnable(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF4, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Shift and mask out GIEN bit */
    val = (val >> 1) & 0x01;

    return val;
}

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setGestureIntEnable(I2C_HandleTypeDef *hi2c, uint8_t enable)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF4, &val) ) 
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 1;
    val &= 0xFD;
    val |= enable;

    /* Write register value back into GCONF4 register */
    return APDS9960_wireWriteDataByte(hi2c, APDS9960_GCONF4, &val);
}

/**
 * @brief Clears the ambient light interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
uint8_t APDS9960_clearAmbientLightInt(I2C_HandleTypeDef *hi2c)
{
    uint8_t throwaway = 0;
    return APDS9960_wireReadDataByte(hi2c, APDS9960_AICLEAR, &throwaway);
}

/**
 * @brief Clears the proximity interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
uint8_t APDS9960_clearProximityInt(I2C_HandleTypeDef *hi2c)
{
    uint8_t throwaway = 0;
    return APDS9960_wireReadDataByte(hi2c, APDS9960_PICLEAR, &throwaway);
}

/**
 * @brief Tells if the gesture state machine is currently running
 *
 * @return 1 if gesture state machine is running, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getGestureMode(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( HAL_ERROR ==APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF4, &val) )
    {
        return HAL_ERROR;
    }

    /* Mask out GMODE bit */
    val &= 0x01;

    return val;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 *
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_setGestureMode(I2C_HandleTypeDef *hi2c, uint8_t mode)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GCONF4, &val) )
    {
        return HAL_ERROR;
    }

    /* Set bits in register to given value */
    mode &= 0x01;
    val &= 0xFE;
    val |= mode;

    /* Write register value back into GCONF4 register */
    return APDS9960_wireWriteDataByte(hi2c, APDS9960_GCONF4, &val);
}


/*******************************************************************************
 * Public methods for controlling the APDS-9960
 ******************************************************************************/

/**
 * @brief Configures I2C communications and initializes registers to defaults
 *
 * @return True if initialized successfully. False otherwise.
 */
uint8_t APDS9960_init(I2C_HandleTypeDef *hi2c)
{
    
    uint8_t id;

    /* Read ID register and check against known values for APDS-9960 */
    if (HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_ID, &id))
    {
        return HAL_ERROR;
    }
    if( !(id == APDS9960_ID_1 || id == APDS9960_ID_2) ) {
        return HAL_ERROR;
    }

    /* Set ENABLE register to 0 (disable all features) */
    if (HAL_ERROR == APDS9960_setMode(hi2c, ALL, OFF))
    {
        return HAL_ERROR;
    }
    id = DEFAULT_ATIME;
    /* Set default values for ambient light and proximity registers */
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_ATIME, &id))
    {
        return HAL_ERROR;
    }
    id = DEFAULT_WTIME;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_WTIME, &id))
    {
        return HAL_ERROR;
    }
    id = DEFAULT_PROX_PPULSE;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_PPULSE, &id))
    {
        return HAL_ERROR;
    }
    id = DEFAULT_POFFSET_UR;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_POFFSET_UR, &id))
    {
        return HAL_ERROR;
    }
    id = DEFAULT_POFFSET_DL;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_POFFSET_DL, &id))
    {
      return HAL_ERROR;
    }
    id = DEFAULT_CONFIG1;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONFIG1, &id))
    {
      return HAL_ERROR;
    }
    
    if (HAL_ERROR == APDS9960_setLEDDrive(hi2c, DEFAULT_LDRIVE))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setProximityGain(hi2c, DEFAULT_PGAIN))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setAmbientLightGain(hi2c, DEFAULT_AGAIN))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setProxIntLowThresh(hi2c, DEFAULT_PILT))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setProxIntHighThresh(hi2c, DEFAULT_PIHT))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setLightIntLowThreshold(hi2c, DEFAULT_AILT))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setLightIntHighThreshold(hi2c, DEFAULT_AIHT))
    {
      return HAL_ERROR;
    }
    
    id = DEFAULT_PERS;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_PERS, &id))
    {
      return HAL_ERROR;
    }
    
    id = DEFAULT_CONFIG2;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONFIG2, &id))
    {
      return HAL_ERROR;
    }
    
    id = DEFAULT_CONFIG3;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_CONFIG3, &id))
    {
      return HAL_ERROR;
    }
    /* Set default values for gesture sense registers */
    if (HAL_ERROR == APDS9960_setGestureEnterThresh(hi2c, DEFAULT_GPENTH))
    {
      return HAL_ERROR;
    }
    
    id = DEFAULT_GEXTH;
    if (HAL_ERROR == APDS9960_setGestureExitThresh(hi2c, &id))
    {
      return HAL_ERROR;
    }
    id = DEFAULT_CONFIG1;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GCONF1, &id))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setGestureGain(hi2c, DEFAULT_GGAIN))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setGestureLEDDrive(hi2c, DEFAULT_GLDRIVE))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setGestureWaitTime(hi2c, DEFAULT_GWTIME))
    {
      return HAL_ERROR;
    }
    id = DEFAULT_GOFFSET;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GOFFSET_U, &id))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GOFFSET_D, &id))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GOFFSET_L, &id))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GOFFSET_R, &id))
    {
      return HAL_ERROR;
    }
    
    id = DEFAULT_GPULSE;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GPULSE, &id))
    {
      return HAL_ERROR;
    }
    
    id = DEFAULT_GCONF3;
    if (HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_GCONF3, &id))
    {
      return HAL_ERROR;
    }
    if (HAL_ERROR == APDS9960_setGestureIntEnable(hi2c, DEFAULT_GIEN))
    {
      return HAL_ERROR;
    }

    return HAL_OK;
}



/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_readAmbientLight(I2C_HandleTypeDef *hi2c, uint16_t * val)
{
    uint8_t val_byte;
    *val = 0;

    /* Read value from clear channel, low byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CDATAL, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    *val = val_byte;

    /* Read value from clear channel, high byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_CDATAH, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    *val = *val + ((uint16_t)val_byte << 8);

    return HAL_OK;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_readRedLight(I2C_HandleTypeDef *hi2c, uint16_t * val)
{
    uint8_t val_byte;
    *val = 0;

    /* Read value from clear channel, low byte register */
    if( HAL_ERROR ==  APDS9960_wireReadDataByte(hi2c, APDS9960_RDATAL, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    *val = val_byte;

    /* Read value from clear channel, high byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_RDATAH, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    *val = *val + ((uint16_t)val_byte << 8);

    return HAL_OK;
}

/**
 * @brief Reads the green light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_readGreenLight(I2C_HandleTypeDef *hi2c, uint16_t * val)
{
    uint8_t val_byte;
    *val = 0;

    /* Read value from clear channel, low byte register */
    if(HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GDATAL, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    *val = val_byte;

    /* Read value from clear channel, high byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_GDATAH, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    *val = *val + ((uint16_t)val_byte << 8);

    return HAL_OK;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_readBlueLight(I2C_HandleTypeDef *hi2c, uint16_t * val)
{
    uint8_t val_byte;
    *val = 0;

    /* Read value from clear channel, low byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_BDATAL, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    *val = val_byte;

    /* Read value from clear channel, high byte register */
    if( HAL_ERROR == APDS9960_wireReadDataByte(hi2c, APDS9960_BDATAH, &val_byte) ) 
    {
        return HAL_ERROR;
    }
    *val = *val + ((uint16_t)val_byte << 8);

    return HAL_OK;
}

/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @param[out] val value of the proximity sensor.
 * @return True if operation successful. False otherwise.
 */
uint8_t APDS9960_readProximity(I2C_HandleTypeDef *hi2c, uint8_t * val)
{
    *val = 0;

    /* Read value from proximity data register */
    return APDS9960_wireReadDataByte(hi2c, APDS9960_PDATA, val);

}




/**
 * @brief Starts the light (R/G/B/Ambient) sensor on the APDS-9960
 *
 * @param[in] interrupts HAL_OK to enable hardware interrupt on high or low light
 * @return True if sensor enabled correctly. False on error.
 */
uint8_t APDS9960_enableLightSensor(I2C_HandleTypeDef *hi2c, uint8_t interrupts)
{

    /* Set default gain, interrupts, enable power, and enable sensor */
    if( HAL_ERROR == APDS9960_setAmbientLightGain(hi2c, DEFAULT_AGAIN) ) 
    {
        return HAL_ERROR;
    }
    if( interrupts ) {
        if( HAL_ERROR == APDS9960_setAmbientLightIntEnable(hi2c, 1) )
        {
            return HAL_ERROR;
        }
    } else {
        if( HAL_ERROR == APDS9960_setAmbientLightIntEnable(hi2c, 0) ) 
        {
            return HAL_ERROR;
        }
    }
    if( HAL_ERROR == APDS9960_enablePower(hi2c) )
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setMode(hi2c, AMBIENT_LIGHT, 1) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;

}

/**
 * @brief Ends the light sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
uint8_t APDS9960_disableLightSensor(I2C_HandleTypeDef *hi2c)
{
    if( HAL_ERROR == APDS9960_setAmbientLightIntEnable(hi2c, 0) ) 
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setMode(hi2c, AMBIENT_LIGHT, 0) ) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts HAL_OK to enable hardware external interrupt on proximity
 * @return True if sensor enabled correctly. False on error.
 */
uint8_t APDS9960_enableProximitySensor(I2C_HandleTypeDef *hi2c, uint8_t interrupts)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if( HAL_ERROR == APDS9960_setProximityGain(hi2c, DEFAULT_PGAIN) ) 
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setLEDDrive(hi2c, DEFAULT_LDRIVE) ) 
    {
        return HAL_ERROR;
    }
    if( interrupts ) {
        if( HAL_ERROR == APDS9960_setProximityIntEnable(hi2c, 1) )
        {
            return HAL_ERROR;
        }
    } else {
        if( HAL_ERROR == APDS9960_setProximityIntEnable(hi2c, 0) ) 
        {
            return HAL_ERROR;
        }
    }
    if( HAL_ERROR == APDS9960_enablePower(hi2c) )
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setMode(hi2c, PROXIMITY, 1) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Ends the proximity sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
uint8_t APDS9960_disableProximitySensor(I2C_HandleTypeDef *hi2c)
{
	if( HAL_ERROR == APDS9960_setProximityIntEnable(hi2c, 0) )
        {
		return HAL_ERROR;
	}
	if( HAL_ERROR == APDS9960_setMode(hi2c, PROXIMITY, 0) ) 
        {
		return HAL_ERROR;
	}

	return HAL_OK;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 *
 * @param[in] interrupts HAL_OK to enable hardware external interrupt on gesture
 * @return True if engine enabled correctly. False on error.
 */
uint8_t APDS9960_enableGestureSensor(I2C_HandleTypeDef *hi2c, uint8_t interrupts)
{

    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE
    */
  
   uint8_t value;
   
    APDS9960_resetGestureParameters();
    value = 0xFF;
    
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_WTIME, &value) ) 
    {
        return HAL_ERROR;
    }
    
    value = DEFAULT_GESTURE_PPULSE;
    if( HAL_ERROR == APDS9960_wireWriteDataByte(hi2c, APDS9960_PPULSE, &value) ) 
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setLEDBoost(hi2c, LED_BOOST_300) ) 
    {
        return HAL_ERROR;
    }
    if( interrupts ) 
    {
        if( HAL_ERROR == APDS9960_setGestureIntEnable(hi2c, 1) ) 
        {
            return HAL_ERROR;
        }
    } 
    else 
    {
        if( HAL_ERROR == APDS9960_setGestureIntEnable(hi2c, 0) ) 
        {
            return HAL_ERROR;
        }
    }
    if( HAL_ERROR == APDS9960_setGestureMode(hi2c, 1) ) 
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_enablePower(hi2c) )
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setMode(hi2c, WAIT, 1) ) 
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setMode(hi2c, PROXIMITY, 1) ) 
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setMode(hi2c, GESTURE, 1) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Ends the gesture recognition engine on the APDS-9960
 *
 * @return True if engine disabled correctly. False on error.
 */
uint8_t APDS9960_disableGestureSensor(I2C_HandleTypeDef *hi2c)
{
    APDS9960_resetGestureParameters();
    if( HAL_ERROR == APDS9960_setGestureIntEnable(hi2c, 0) )
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setGestureMode(hi2c, 0) ) 
    {
        return HAL_ERROR;
    }
    if( HAL_ERROR == APDS9960_setMode(hi2c, GESTURE, 0) ) 
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}



/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/


/**
 * @brief Writes a single byte to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
uint8_t APDS9960_wireWriteDataByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t * val)
{

  uint8_t result;
  result = HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
  
  return result;
}

/**
 * @brief Writes a block (array) of bytes to the I2C device and register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val pointer to the beginning of the data byte array
 * @param[in] len the length (in bytes) of the data to write
 * @return True if successful write operation. False otherwise.
 */
uint8_t APDS9960_wireWriteDataBlock(I2C_HandleTypeDef *hi2c,  uint8_t reg,
                                        uint8_t *val,
                                        unsigned int len)
{

    uint8_t result;

    /* Indicate which register we want to read from */


    /* Write block data */
    result = HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, len, 100);
    

    return result;
}

/**
 * @brief Reads a single byte from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
uint8_t APDS9960_wireReadDataByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t * val)
{
    uint8_t result;

    /* Indicate which register we want to read from */


    /* Write block data */
    result = HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
    

    return result;
}

/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */
uint8_t APDS9960_wireReadDataBlock( I2C_HandleTypeDef *hi2c,  uint8_t reg,
                                        uint8_t *val,
                                        uint8_t len)
{
    uint8_t result;

    /* Indicate which register we want to read from */


    /* Read block data */
    result = HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, len, 100);
    

    return result;
}
