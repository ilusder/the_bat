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
    if( !APDS9960_wireReadDataByte(hi2c, APDS9960_ID, &id) ) {
        return HAL_ERROR;
    }
    if( !(id == APDS9960_ID_1 || id == APDS9960_ID_2) ) {
        return HAL_ERROR;
    }

    /* Set ENABLE register to 0 (disable all features) */
    if( !APDS9960_setMode(hi2c, ALL, OFF) ) {
        return HAL_ERROR;
    }

//    /* Set default values for ambient light and proximity registers */
//    if( !APDS9960_wireWriteDataByte(hi2c, APDS9960_ATIME, (uint8_t) DEFAULT_ATIME) ) {
//        return HAL_ERROR;
//    }
//    if( !APDS9960_wireWriteDataByte(hi2c, APDS9960_WTIME, DEFAULT_WTIME) ) {
//        return HAL_ERROR;
//    }
//    if( !APDS9960_wireWriteDataByte(hi2c, APDS9960_PPULSE, DEFAULT_PROX_PPULSE) ) {
//        return HAL_ERROR;
//    }
//    if( !APDS9960_wireWriteDataByte(hi2c, APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR) ) {
//        return HAL_ERROR;
//    }
//    if( !APDS9960_wireWriteDataByte(hi2c, APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL) ) {
//        return HAL_ERROR;
//    }
//    if( !APDS9960_wireWriteDataByte(hi2c, APDS9960_CONFIG1, DEFAULT_CONFIG1) ) {
//        return HAL_ERROR;
//    }
//    if( !setLEDDrive(DEFAULT_LDRIVE) ) {
//        return HAL_ERROR;
//    }
//    if( !setProximityGain(DEFAULT_PGAIN) ) {
//        return HAL_ERROR;
//    }
//    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
//        return HAL_ERROR;
//    }
//    if( !setProxIntLowThresh(DEFAULT_PILT) ) {
//        return HAL_ERROR;
//    }
//    if( !setProxIntHighThresh(DEFAULT_PIHT) ) {
//        return HAL_ERROR;
//    }
//    if( !setLightIntLowThreshold(DEFAULT_AILT) ) {
//        return HAL_ERROR;
//    }
//    if( !setLightIntHighThreshold(DEFAULT_AIHT) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_PERS, DEFAULT_PERS) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3) ) {
//        return HAL_ERROR;
//    }
//
//    /* Set default values for gesture sense registers */
//    if( !setGestureEnterThresh(DEFAULT_GPENTH) ) {
//        return HAL_ERROR;
//    }
//    if( !setGestureExitThresh(DEFAULT_GEXTH) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_GCONF1, DEFAULT_GCONF1) ) {
//        return HAL_ERROR;
//    }
//    if( !setGestureGain(DEFAULT_GGAIN) ) {
//        return HAL_ERROR;
//    }
//    if( !setGestureLEDDrive(DEFAULT_GLDRIVE) ) {
//        return HAL_ERROR;
//    }
//    if( !setGestureWaitTime(DEFAULT_GWTIME) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_GPULSE, DEFAULT_GPULSE) ) {
//        return HAL_ERROR;
//    }
//    if( !wireWriteDataByte(APDS9960_GCONF3, DEFAULT_GCONF3) ) {
//        return HAL_ERROR;
//    }
//    if( !setGestureIntEnable(DEFAULT_GIEN) ) {
//        return HAL_ERROR;
//    }

    return HAL_OK;
}


uint8_t APDS9960_getMode(I2C_HandleTypeDef *hi2c)
{
    uint8_t enable_value;

    /* Read current ENABLE register */
    if( !APDS9960_wireReadDataByte(hi2c, APDS9960_ENABLE, &enable_value) ) {
        return HAL_ERROR;
    }

    return enable_value;
}


uint8_t APDS9960_setMode(I2C_HandleTypeDef *hi2c, uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = APDS9960_getMode(hi2c);
    if( reg_val == HAL_ERROR ) {
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
    if( !APDS9960_wireWriteDataByte(hi2c, APDS9960_ENABLE, &reg_val) ) {
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
  result = HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, reg, 1, val, 1, 100);
  
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
    result = HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, reg, 1, val, len, 100);
    

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
    result = HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, reg, 1, val, 1, 100);
    

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
    result = HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, reg, 1, val, len, 100);
    

    return result;
}
