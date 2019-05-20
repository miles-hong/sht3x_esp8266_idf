/*
 * Driver for Sensirion SHT3x digital temperature and humidity sensor
 * connected to I2C
 *
 * This driver is for the usage with the ESP8266 and FreeRTOS (esp-open-rtos)
 * [https://github.com/SuperHouse/esp-open-rtos]. It is also working with ESP32
 * and ESP-IDF [https://github.com/espressif/esp-idf.git] as well as Linux
 * based systems using a wrapper library for ESP8266 functions.
 *
 * ----------------------------------------------------------------
 *
 * The BSD License (3-clause license)
 *
 * Copyright (c) 2017 Gunar Schorcht (https://github.com/gschorcht)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sht3x.h"

static const char *TAG = "SHT3x";

static esp_err_t SHT3x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
static float SHT3x_CalcTemperature(uint16_t rawValue);
static float SHT3x_CalcHumidity(uint16_t rawValue);
/**
 * @brief i2c master initialization
 */
esp_err_t I2C_Init()
{
    int i2c_master_port = I2C_SHT3X_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SHT3X_MASTER_SDA_IO;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = I2C_SHT3X_MASTER_SCL_IO;
    conf.scl_pullup_en = 0;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief test code to write sht3x
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param I2C_SHT3X_MASTER_NUM I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t I2C_SHT3x_Write(uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT3X_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_SHT3X_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}
/**
 * @brief send command to sht3x
 */
esp_err_t I2C_SHT3x_WriteCommand(etCommands command)
{
    uint8_t cmd_data[2] = { command >> 8, command & 0xff };
    
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT3X_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, cmd_data, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_SHT3X_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);   

    if(ret == ESP_OK)
    {
       ESP_LOGI(TAG, "Command: 0x%0x Send OK.\n", command);
    }
    return ret;
}
/**
 * @brief test code to read sht3x
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param I2C_SHT3X_MASTER_NUM I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t I2C_SHT3x_Read(uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT3X_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_SHT3X_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT3X_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_SHT3X_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}
esp_err_t I2C_SHT3x_Read2BytesAndCrc(uint16_t *data1, uint16_t *data2)
{
    int ret;
    uint8_t bytes[6];
    uint8_t crcdata1[2], crcdata2[2];
    uint8_t checksum1, checksum2;

    ESP_ERROR_CHECK(I2C_SHT3x_WriteCommand(CMD_MEAS_POLLING_H));
    ESP_ERROR_CHECK(I2C_SHT3x_WriteCommand(CMD_FETCH_DATA));
    ESP_ERROR_CHECK(I2C_SHT3x_Read(0, bytes, 6));
    ESP_LOGI(TAG, "bytes[0]= 0x%0x, bytes[1]= 0x%0x, bytes[2]= 0x%0x\n", bytes[0], bytes[1], bytes[2]);
    ESP_LOGI(TAG, "bytes[3]= 0x%0x, bytes[4]= 0x%0x, bytes[5]= 0x%0x\n", bytes[3], bytes[4], bytes[5]);
    
    memcpy(crcdata1, bytes, 2);
    checksum1 = bytes[2];
    ESP_LOGI(TAG, "crcdata1[0]= 0x%0x, crcdata1[1]= 0x%0x, checksum1= 0x%0x\n", crcdata1[0], crcdata1[1], checksum1);
    ret = SHT3x_CheckCrc(crcdata1, 2, checksum1);
    *data1 = (crcdata1[0] << 8) | crcdata1[1];

    memcpy(crcdata2, bytes+3, 2);
    checksum2 = bytes[5];
    ESP_LOGI(TAG, "crcdata2[0]= 0x%0x, crcdata2[1]= 0x%0x, checksum2= 0x%0x\n", crcdata2[0], crcdata2[1], checksum2);
    ret = SHT3x_CheckCrc(crcdata2, 2, checksum2);
    *data2 = (crcdata2[0] << 8) | crcdata2[1];

    return ret;
}
esp_err_t I2C_SHT3x_ReadMeasurementBuffer(float *temperature, float *humidity)
{
    int ret;
    uint16_t rawValueTemp;       //temperature raw value from sensor
    uint16_t rawValueHumi;       //humidity raw value from sensor

    uint8_t rawData[6];
    uint8_t rawDataTemp[2], rawDataHumi[2];
    uint8_t checksumTemp, checksumHumi;

    ESP_ERROR_CHECK(I2C_SHT3x_WriteCommand(CMD_MEAS_POLLING_H));    
    vTaskDelay(100 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(I2C_SHT3x_WriteCommand(CMD_FETCH_DATA));
    vTaskDelay(100 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(I2C_SHT3x_Read(0, rawData, 6));

    ESP_LOGI(TAG, "rawData[0]= 0x%0x, rawData[1]= 0x%0x, rawData[2]= 0x%0x\n", rawData[0], rawData[1], rawData[2]);
    ESP_LOGI(TAG, "rawData[3]= 0x%0x, rawData[4]= 0x%0x, rawData[5]= 0x%0x\n", rawData[3], rawData[4], rawData[5]);
    
    memcpy(rawDataTemp, rawData, 2);
    checksumTemp = rawData[2];
    ESP_LOGI(TAG, "rawDataTemp[0]= 0x%0x, rawDataTemp[1]= 0x%0x, checksumTemp= 0x%0x\n", rawDataTemp[0], rawDataTemp[1], checksumTemp);
    ret = SHT3x_CheckCrc(rawDataTemp, 2, checksumTemp);
    if(ret == ESP_OK)
    {
    rawValueTemp = (rawDataTemp[0] << 8) | rawDataTemp[1];
    ESP_LOGI(TAG, "rawValueTemp= 0x%0x\n", rawValueTemp);
    *temperature = SHT3x_CalcTemperature(rawValueTemp);
    }

    memcpy(rawDataHumi, rawData+3, 2);
    checksumHumi = rawData[5];
    ESP_LOGI(TAG, "rawDataHumi[0]= 0x%0x, rawDataHumi[1]= 0x%0x, checksumHumi= 0x%0x\n", rawDataHumi[0], rawDataHumi[1], checksumHumi);
    ret = SHT3x_CheckCrc(rawDataHumi, 2, checksumHumi);
    if(ret == ESP_OK)
    {
    rawValueHumi = (rawDataHumi[0] << 8) | rawDataHumi[1];
    ESP_LOGI(TAG, "rawValueHumi= 0x%0x\n", rawValueHumi);
    *humidity = SHT3x_CalcHumidity(rawValueHumi);
    }

    return ret;
}
static uint8_t SHT3x_CalcCrc(uint8_t data[], uint8_t nbrOfBytes)
{
    uint8_t bit;        //bit mask
    uint8_t crc = 0xFF; //calculated checksum
    uint8_t bytectr;    //byte counter

    //calculates 8-Bit checksum with given polynomial
    for(bytectr = 0; bytectr < nbrOfBytes; bytectr++)
    {
        crc ^= (data[bytectr]);
        for(bit = 8; bit > 0; --bit)
        {
            if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else           crc = (crc << 1);
        }
    }
    return crc;
}
static esp_err_t SHT3x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
    esp_err_t ret;
    uint8_t crc;        //calculated checksum

    //calculates 8-Bit checksum
    crc = SHT3x_CalcCrc(data, nbrOfBytes);

    //verify checksum
    if(crc == checksum)
    {
        ESP_LOGI(TAG, "CRC OK.\n");
        ret = ESP_OK;
    } 
    else
    {
        ESP_LOGI(TAG, "CRC FAILED.\n");
        ret = ESP_FAIL;
    }
    return ret;
}
static float SHT3x_CalcTemperature(uint16_t rawValue)
{
    //calculate temperature
    //T = -45 + 175 * rawValue /( 2 ^ 16 -1 )
    return 175.0f * (float) rawValue / 65535.0f - 45.0f;
}
static float SHT3x_CalcHumidity(uint16_t rawValue)
{
    //calculate humidity
    //RH = rawValue /(2^16-1) * 100

    return 100.0f * (float) rawValue / 65535.0f;
}
esp_err_t I2C_SHT3x_Init()
{
    uint8_t cmd_data;
    vTaskDelay(100 / portTICK_RATE_MS);
    I2C_Init();
    //clear status register
    ESP_ERROR_CHECK(I2C_SHT3x_WriteCommand(CMD_CLEAR_STATUS));
   //write "read serial number" command
    // ESP_ERROR_CHECK(I2C_SHT3x_WriteCommand(I2C_SHT3X_MASTER_NUM, CMD_READ_SERIALNBR));
    return ESP_OK;
}
esp_err_t SHT3x_ReadSerialNumber(uint32_t *SerialNumber)
{
    esp_err_t ret;
    uint16_t SerialNumWords[2];
    uint8_t data[6];
    uint8_t data1[2];
    uint8_t data2[2];
    uint8_t checknum1;
    uint8_t checknum2;
    uint8_t crc;

    ret = I2C_SHT3x_WriteCommand(CMD_READ_SERIALNBR);
    ret = I2C_SHT3x_Read(0, data, 6);
    ESP_LOGI(TAG, "data[0]= 0x%0x, data[1]= 0x%0x, data[2]= 0x%0x\n", data[0], data[1], data[2]);
    ESP_LOGI(TAG, "data[3]= 0x%0x, data[4]= 0x%0x, data[5]= 0x%0x\n", data[3], data[4], data[5]);
    memcpy(data1, data, 2);
    checknum1 = data[2];
    ESP_LOGI(TAG, "data1[0]= 0x%0x, data1[1]= 0x%0x, checknum1= 0x%0x\n", data1[0], data1[1], checknum1);
    crc = SHT3x_CalcCrc(data1, 2);
    ESP_LOGI(TAG, "CRC: 0x%0x\n", crc);
    memcpy(data2, data+3, 2);
    checknum2 = data[5];
    ESP_LOGI(TAG, "data2[0]= 0x%0x, data2[1]= 0x%0x, checknum2= 0x%0x\n", data2[0], data2[1], checknum2);
    crc = SHT3x_CalcCrc(data2, 2);
    ESP_LOGI(TAG, "CRC: 0x%0x\n", crc);
    //ret = I2C_SHT3x_Read2BytesAndCrc(&SerialNumWords[1], ACK_VAL);
    // ret = I2C_SHT3x_Read2BytesAndCrc(&SerialNumWords[2], ACK_VAL);

    // *SerialNumber = (SerialNumWords[0] << 16 | SerialNumWords[1]);
    
    return ret;
}

void I2C_SHT3x_Task(void *arg)
{
    // uint8_t data[3];
    float temperature;
    float humidity;
    uint32_t SerialNumber;
    I2C_SHT3x_Init();
    // ESP_ERROR_CHECK(I2C_SHT3x_WriteCommand(CMD_MEAS_POLLING_H));
    int ret;
    while(1)
    {
    ret = I2C_SHT3x_ReadMeasurementBuffer(&temperature, &humidity);
    // ESP_ERROR_CHECK(I2C_SHT3x_Read(I2C_SHT3X_MASTER_NUM, 0, data, 3));
    // ESP_LOGI(TAG, "data[0]= 0x%0x, data[1]= 0x%0x, data[2]= 0x%0x\n", data[0], data[1], data[2]);
    // ESP_ERROR_CHECK(SHT3x_ReadSerialNumber(&SerialNum));
    if(ret == ESP_OK)
        {
            ESP_LOGI(TAG, "temperature= %d.%d, humidity= %d.%d\n", (uint16_t)temperature, (uint16_t)(temperature *100) %100,(uint16_t)humidity,(uint16_t)(humidity *100) %100);
            // SerialNumber = (SerialNumWords[0] << 16 | SerialNumWords[1]);
            // ESP_LOGI(TAG, "SerialNumber = 0x%0x\n", SerialNumber);
        }
    vTaskDelay(1000 / portTICK_RATE_MS);
    }
}