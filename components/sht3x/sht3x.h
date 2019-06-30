/*
 * Driver for Sensirion SHT3x digital temperature and humidity sensor
 * connected to I2C
 */
 
#ifndef __SHT3x_H__
#define __SHT3x_H__
#endif
// Uncomment to enable debug output
// #define SHT3x_DEBUG_LEVEL_1     // only error messages
// #define SHT3x_DEBUG_LEVEL_2     // error and debug messages

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_SHT3X_MASTER_SCL_IO             5                /*!< gpio number for I2C master clock */
#define I2C_SHT3X_MASTER_SDA_IO             4                /*!< gpio number for I2C master data  */
#define I2C_SHT3X_MASTER_NUM                I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_SHT3X_MASTER_TX_BUF_DISABLE     0                /*!< I2C master do not need buffer */
#define I2C_SHT3X_MASTER_RX_BUF_DISABLE     0                /*!< I2C master do not need buffer */

#define SHT3X_SENSOR_ADDR                   0x44             /*!< slave address for SHT3X sensor */
#define SHT3X_CMD_START                     0x41             /*!< Command to set measure mode */
#define SHT3X_WHO_AM_I                      0x75             /*!< Command to read WHO_AM_I reg */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

// Generator polynomial for CRC
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

// definition of possible I2C slave addresses
#define SHT3x_ADDR_1 0x44        // ADDR pin connected to GND/VSS (default)
#define SHT3x_ADDR_2 0x45        // ADDR pin connected to VDD

// definition of error codes
#define SHT3x_OK                     0
#define SHT3x_NOK                    -1

#define SHT3x_I2C_ERROR_MASK         0x000f
#define SHT3x_DRV_ERROR_MASK         0xfff0

// error codes for I2C interface ORed with SHT3x error codes
#define SHT3x_I2C_READ_FAILED        1
#define SHT3x_I2C_SEND_CMD_FAILED    2
#define SHT3x_I2C_BUSY               3

// SHT3x driver error codes OR ed with error codes for I2C interface
#define SHT3x_MEAS_NOT_STARTED       (1  << 8)
#define SHT3x_MEAS_ALREADY_RUNNING   (2  << 8)
#define SHT3x_MEAS_STILL_RUNNING     (3  << 8)
#define SHT3x_READ_RAW_DATA_FAILED   (4  << 8)

#define SHT3x_SEND_MEAS_CMD_FAILED   (5  << 8)
#define SHT3x_SEND_RESET_CMD_FAILED  (6  << 8)
#define SHT3x_SEND_STATUS_CMD_FAILED (7  << 8)
#define SHT3x_SEND_FETCH_CMD_FAILED  (8  << 8)

#define SHT3x_WRONG_CRC_TEMPERATURE  (9  << 8)
#define SHT3x_WRONG_CRC_HUMIDITY     (10 << 8)

#define SHT3x_RAW_DATA_SIZE 6

//-- Enumerations -------------------------------------------------------------
// Sensor Commands
typedef enum{
  CMD_READ_SERIALNBR  = 0x3780, // read serial number
  CMD_READ_STATUS     = 0xF32D, // read status register
  CMD_CLEAR_STATUS    = 0x3041, // clear status register
  CMD_HEATER_ENABLE   = 0x306D, // enabled heater
  CMD_HEATER_DISABLE  = 0x3066, // disable heater
  CMD_SOFT_RESET      = 0x30A2, // soft reset
  CMD_MEAS_CLOCKSTR_H = 0x2C06, // measurement: clock stretching, high repeatability
  CMD_MEAS_CLOCKSTR_M = 0x2C0D, // measurement: clock stretching, medium repeatability
  CMD_MEAS_CLOCKSTR_L = 0x2C10, // measurement: clock stretching, low repeatability
  CMD_MEAS_POLLING_H  = 0x2400, // measurement: polling, high repeatability
  CMD_MEAS_POLLING_M  = 0x240B, // measurement: polling, medium repeatability
  CMD_MEAS_POLLING_L  = 0x2416, // measurement: polling, low repeatability
  CMD_MEAS_PERI_05_H  = 0x2032, // measurement: periodic 0.5 mps, high repeatability
  CMD_MEAS_PERI_05_M  = 0x2024, // measurement: periodic 0.5 mps, medium repeatability
  CMD_MEAS_PERI_05_L  = 0x202F, // measurement: periodic 0.5 mps, low repeatability
  CMD_MEAS_PERI_1_H   = 0x2130, // measurement: periodic 1 mps, high repeatability
  CMD_MEAS_PERI_1_M   = 0x2126, // measurement: periodic 1 mps, medium repeatability
  CMD_MEAS_PERI_1_L   = 0x212D, // measurement: periodic 1 mps, low repeatability
  CMD_MEAS_PERI_2_H   = 0x2236, // measurement: periodic 2 mps, high repeatability
  CMD_MEAS_PERI_2_M   = 0x2220, // measurement: periodic 2 mps, medium repeatability
  CMD_MEAS_PERI_2_L   = 0x222B, // measurement: periodic 2 mps, low repeatability
  CMD_MEAS_PERI_4_H   = 0x2334, // measurement: periodic 4 mps, high repeatability
  CMD_MEAS_PERI_4_M   = 0x2322, // measurement: periodic 4 mps, medium repeatability
  CMD_MEAS_PERI_4_L   = 0x2329, // measurement: periodic 4 mps, low repeatability
  CMD_MEAS_PERI_10_H  = 0x2737, // measurement: periodic 10 mps, high repeatability
  CMD_MEAS_PERI_10_M  = 0x2721, // measurement: periodic 10 mps, medium repeatability
  CMD_MEAS_PERI_10_L  = 0x272A, // measurement: periodic 10 mps, low repeatability
  CMD_FETCH_DATA      = 0xE000, // readout measurements for periodic mode
  CMD_R_AL_LIM_LS     = 0xE102, // read alert limits, low set
  CMD_R_AL_LIM_LC     = 0xE109, // read alert limits, low clear
  CMD_R_AL_LIM_HS     = 0xE11F, // read alert limits, high set
  CMD_R_AL_LIM_HC     = 0xE114, // read alert limits, high clear
  CMD_W_AL_LIM_HS     = 0x611D, // write alert limits, high set
  CMD_W_AL_LIM_HC     = 0x6116, // write alert limits, high clear
  CMD_W_AL_LIM_LC     = 0x610B, // write alert limits, low clear
  CMD_W_AL_LIM_LS     = 0x6100, // write alert limits, low set
  CMD_NO_SLEEP        = 0x303E,
}etCommands;
// Measurement Repeatability
typedef enum{
  REPEATAB_HIGH,   // high repeatability
  REPEATAB_MEDIUM, // medium repeatability
  REPEATAB_LOW,    // low repeatability
}etRepeatability;

// Measurement Mode
typedef enum{
  MODE_CLKSTRETCH, // clock stretching
  MODE_POLLING,    // polling
}etMode;

typedef enum{
  FREQUENCY_HZ5,  //  0.5 measurements per seconds
  FREQUENCY_1HZ,  //  1.0 measurements per seconds
  FREQUENCY_2HZ,  //  2.0 measurements per seconds
  FREQUENCY_4HZ,  //  4.0 measurements per seconds
  FREQUENCY_10HZ, // 10.0 measurements per seconds
}etFrequency;
/**
 * @brief	raw data type
 */
typedef uint8_t sht3x_raw_data_t [SHT3x_RAW_DATA_SIZE];


/**
 * @brief   possible measurement modes
 */
typedef enum {
    sht3x_single_shot = 0,  // one single measurement
    sht3x_periodic_05mps,   // periodic with 0.5 measurements per second (mps)
    sht3x_periodic_1mps,    // periodic with   1 measurements per second (mps)
    sht3x_periodic_2mps,    // periodic with   2 measurements per second (mps)
    sht3x_periodic_4mps,    // periodic with   4 measurements per second (mps)
    sht3x_periodic_10mps    // periodic with  10 measurements per second (mps)
} sht3x_mode_t;
    
    
/**
 * @brief   possible repeatability modes
 */
typedef enum {
    sht3x_high = 0,
    sht3x_medium,
    sht3x_low
} sht3x_repeat_t;

/**
 * @brief 	SHT3x sensor device data structure type
 */
typedef struct {

    uint32_t        error_code;      // combined error codes
    
    uint8_t         bus;             // I2C bus at which sensor is connected
    uint8_t         addr;            // I2C slave address of the sensor
    
    sht3x_mode_t    mode;            // used measurement mode
    sht3x_repeat_t  repeatability;   // used repeatability
 
    bool            meas_started;    // indicates whether measurement started
    uint32_t        meas_start_time; // measurement start time in us
    bool            meas_first;      // first measurement in periodic mode
    
} sht3x_sensor_t;    


/**
 * @brief	Initialize a SHT3x sensor
 * 
 * The function creates a data structure describing the sensor and
 * initializes the sensor device.
 *  
 * @param   bus       I2C bus at which the sensor is connected
 * @param   addr      I2C slave address of the sensor
 * @return            pointer to sensor data structure, or NULL on error
 */
sht3x_sensor_t* sht3x_init_sensor (uint8_t bus, uint8_t addr);


/**
 * @brief   High level measurement function
 *
 * For convenience this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. Starts a measurement in single shot mode with high reliability
 * 2. Waits using *vTaskDelay* until measurement results are available 
 * 3. Returns the results in kind of floating point sensor values 
 *
 * This function is the easiest way to use the sensor. It is most suitable
 * for users that don't want to have the control on sensor details.
 *
 * Please note: The function delays the calling task up to 30 ms to wait for
 * the  the measurement results. This might lead to problems when the function
 * is called from a software timer callback function.
 *
 * @param   dev         pointer to sensor device data structure
 * @param   temperature returns temperature in degree Celsius   
 * @param   humidity    returns humidity in percent
 * @return              true on success, false on error
 */
bool sht3x_measure (sht3x_sensor_t* dev, float* temperature, float* humidity);


/**
 * @brief	Start the measurement in single shot or periodic mode
 *
 * The function starts the measurement either in *single shot mode* 
 * (exactly one measurement) or *periodic mode* (periodic measurements)
 * with given repeatabilty.
 *
 * In the *single shot mode*, this function has to be called for each
 * measurement. The measurement duration has to be waited every time
 * before the results can be fetched. 
 *
 * In the *periodic mode*, this function has to be called only once. Also 
 * the measurement duration has to be waited only once until the first
 * results are available. After this first measurement, the sensor then
 * automatically performs all subsequent measurements. The rate of periodic
 * measurements can be 10, 4, 2, 1 or 0.5 measurements per second (mps).
 * 
 * Please note: Due to inaccuracies in timing of the sensor, the user task
 * should fetch the results at a lower rate. The rate of the periodic
 * measurements is defined by the parameter *mode*.
 *
 * @param   dev         pointer to sensor device data structure
 * @param   mode        measurement mode, see type *sht3x_mode_t*
 * @param   repeat      repeatability, see type *sht3x_repeat_t*
 * @return              true on success, false on error
 */
bool sht3x_start_measurement (sht3x_sensor_t* dev, sht3x_mode_t mode,
                              sht3x_repeat_t repeat);

/**
 * @brief   Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability. Once a measurement is
 * started with function *sht3x_start_measurement* the user task can use this
 * duration in RTOS ticks directly to wait with function *vTaskDelay* until
 * the measurement results can be fetched.
 *
 * Please note: The duration only depends on repeatability level. Therefore,
 * it can be considered as constant for a repeatibility.
 *
 * @param   repeat      repeatability, see type *sht3x_repeat_t*
 * @return              measurement duration given in RTOS ticks
 */
uint8_t sht3x_get_measurement_duration (sht3x_repeat_t repeat);


/**
 * @brief	Read measurement results from sensor as raw data
 *
 * The function read measurement results from the sensor, checks the CRC
 * checksum and stores them in the byte array as following.
 *
 *      data[0] = Temperature MSB
 *      data[1] = Temperature LSB
 *      data[2] = Temperature CRC
 *      data[3] = Pressure MSB
 *      data[4] = Pressure LSB 
 *      data[2] = Pressure CRC
 *
 * In case that there are no new data that can be read, the function fails.
 * 
 * @param   dev         pointer to sensor device data structure
 * @param   raw_data    byte array in which raw data are stored 
 * @return              true on success, false on error
 */
bool sht3x_get_raw_data(sht3x_sensor_t* dev, sht3x_raw_data_t raw_data);


/**
 * @brief	Computes sensor values from raw data
 *
 * @param   raw_data    byte array that contains raw data  
 * @param   temperature returns temperature in degree Celsius   
 * @param   humidity    returns humidity in percent
 * @return              true on success, false on error
 */
bool sht3x_compute_values (sht3x_raw_data_t raw_data, 
                           float* temperature, float* humidity);


/**
 * @brief	Get measurement results in form of sensor values
 *
 * The function combines function *sht3x_read_raw_data* and function 
 * *sht3x_compute_values* to get the measurement results.
 * 
 * In case that there are no results that can be read, the function fails.
 *
 * @param   dev         pointer to sensor device data structure
 * @param   temperature returns temperature in degree Celsius   
 * @param   humidity    returns humidity in percent
 * @return              true on success, false on error
 */
bool sht3x_get_results (sht3x_sensor_t* dev, 
                        float* temperature, float* humidity);

void I2C_SHT3x_Task(void *arg);
#ifdef __cplusplus
}
#endif
