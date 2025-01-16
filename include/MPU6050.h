/**
 * @author Crispin Mukalay
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * MPU-6050 triple-axis MEMS gyroscope and triple-axis MEMS accelerometer.
 *
 * Datasheet:
 *
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 */

#ifndef MPU6050_H
#define MPU6050_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */
#define MPU6050_I2C_ADDRESS 0x68 // 7-bit address of slave device (MPU6050).

//-----------
// Registers
//-----------
#define SELF_TEST_X_REG        0x0D
#define SELF_TEST_Y_REG        0x0E
#define SELF_TEST_Z_REG        0x0F
#define SELF_TEST_A_REG        0x10
#define SMPLRT_DIV_REG         0x19
#define CONFIG_REG             0x1A
#define GYRO_CONFIG_REG        0x1B
#define ACCEL_CONFIG_REG       0x1C
#define FIFO_EN_REG            0x23
#define I2C_MST_CTRL_REG       0x24
#define I2C_SLV0_ADDR_REG      0x25
#define I2C_SLV0_REG           0x26
#define I2C_SLV0_CTRL_REG      0x27
#define I2C_SLV1_ADDR_REG      0x28
#define I2C_SLV1_REG           0x29
#define I2C_SLV1_CTRL_REG      0x2A
#define I2C_SLV2_ADDR_REG      0x2B
#define I2C_SLV2_REG           0x2C
#define I2C_SLV2_CTRL_REG      0x2D
#define I2C_SLV3_ADDR_REG      0x2E
#define I2C_SLV3_REG           0x2F
#define I2C_SLV3_CTRL_REG      0x30
#define I2C_SLV4_ADDR_REG      0x31
#define I2C_SLV4_REG           0x32
#define I2C_SLV4_DO_REG        0x33
#define I2C_SLV4_CTRL_REG      0x34
#define I2C_SLV4_DI_REG        0x35
#define I2C_MST_STATUS_REG     0x36
#define INT_PIN_CFG_REG        0x37
#define INT_ENABLE_REG         0x38
#define INT_STATUS_REG         0x3A
#define ACCEL_XOUT_H_REG       0x3B
#define ACCEL_XOUT_L_REG       0x3C
#define ACCEL_YOUT_H_REG       0x3D
#define ACCEL_YOUT_L_REG       0x3E
#define ACCEL_ZOUT_H_REG       0x3F
#define ACCEL_ZOUT_L_REG       0x40
#define TEMP_OUT_H_REG         0x41
#define TEMP_OUT_L_REG         0x42
#define GYRO_XOUT_H_REG        0x43
#define GYRO_XOUT_L_REG        0x44
#define GYRO_YOUT_H_REG        0x45
#define GYRO_YOUT_L_REG        0x46
#define GYRO_ZOUT_H_REG        0x47
#define GYRO_ZOUT_L_REG        0x48
#define EXT_SENS_DATA_00_REG   0x49
#define EXT_SENS_DATA_01_REG   0x4A
#define EXT_SENS_DATA_02_REG   0x4B
#define EXT_SENS_DATA_03_REG   0x4C
#define EXT_SENS_DATA_04_REG   0x4D
#define EXT_SENS_DATA_05_REG   0x4E
#define EXT_SENS_DATA_06_REG   0x4F
#define EXT_SENS_DATA_07_REG   0x50
#define EXT_SENS_DATA_08_REG   0x51
#define EXT_SENS_DATA_09_REG   0x52
#define EXT_SENS_DATA_10_REG   0x53
#define EXT_SENS_DATA_11_REG   0x54
#define EXT_SENS_DATA_12_REG   0x55
#define EXT_SENS_DATA_13_REG   0x56
#define EXT_SENS_DATA_14_REG   0x57
#define EXT_SENS_DATA_15_REG   0x58
#define EXT_SENS_DATA_16_REG   0x59
#define EXT_SENS_DATA_17_REG   0x5A
#define EXT_SENS_DATA_18_REG   0x5B
#define EXT_SENS_DATA_19_REG   0x5C
#define EXT_SENS_DATA_20_REG   0x5D
#define EXT_SENS_DATA_21_REG   0x5E
#define EXT_SENS_DATA_22_REG   0x5F
#define EXT_SENS_DATA_23_REG   0x60
#define I2C_SLV0_DO_REG        0x63
#define I2C_SLV1_DO_REG        0x64
#define I2C_SLV2_DO_REG        0x65
#define I2C_SLV3_DO_REG        0x66
#define I2C_MST_DELAY_CTRL_REG 0x67
#define SIGNAL_PATH_RESET_REG  0x68
#define USER_CTRL_REG          0x6A
#define PWR_MGMT_1_REG         0x6B
#define PWR_MGMT_2_REG         0x6C
#define FIFO_COUNTH_REG        0x72
#define FIFO_COUNTL_REG        0x73
#define FIFO_R_W_REG           0x74
#define WHO_AM_I_REG           0x75

//----------------------------------------------
// External Frame Synchronization Configuration
//----------------------------------------------
#define FSYNC_DISABLE      0x00
#define FSYNC_TEMP_OUT_L   0x01
#define FSYNC_GYRO_XOUT_L  0x02
#define FSYNC_GYRO_YOUT_L  0x03
#define FSYNC_GYRO_ZOUT_L  0x04
#define FSYNC_ACCEL_XOUT_L 0x05
#define FSYNC_ACCEL_YOUT_L 0x06
#define FSYNC_ACCEL_ZOUT_L 0x07

//---------------------------------------
// Digital Low Pass Filter Configuration
//---------------------------------------
#define DLPF_260_256 0x00
#define DLPF_184_188 0x01
#define DLPF_94_98   0x02
#define DLPF_44_42   0x03
#define DLPF_21_20   0x04
#define DLPF_10_10   0x05
#define DLPF_5_5     0x06

//------------------------------------
// Gyroscope Selft-Test Configuration
//------------------------------------
#define GFS_DISABLE 0x00
#define GFS_250dps  0x00
#define GFS_500dps  0x08
#define GFS_1000dps 0x10
#define GFS_2000dps 0x18
#define GYRO_ST_ON  0xE0
#define GYRO_ST_OFF 0x00

//----------------------------------------
// Accelerometer Selft-Test Configuration
//----------------------------------------
#define AFS_DSIABLE 0x00
#define AFS_2g      0x00
#define AFS_4g      0x08
#define AFS_8g      0x10
#define AFS_16g     0x18
#define ACC_ST_ON   0xE0
#define ACC_ST_OFF  0x00

//----------------------------------------------
// Sensor Measurements To Load In FIFO Register
//----------------------------------------------
#define TEMP_FIFO_EN              0x80
#define GYRO_FIFO_EN              0x70
#define ACCEL_FIFO_EN             0x08
#define SLV2_FIFO_EN              0x04
#define SLV1_FIFO_EN              0x02
#define SLV0_FIFO_EN              0x01
#define SLVZ_FIFO_EN              0x07
#define TEMPGYROACCEL_FIFO_EN     0xF8
#define GYROACCEL_FIFO_EN         0x78
#define TEMPGYROACCELSLVZ_FIFO_EN 0xFF

//---------------------------
// Signal Path Reset Values
//---------------------------
#define GYRRO_RESET 0x04
#define ACCEL_RESET 0x02
#define TEMP_RESET  0x01
#define ALL_RESET   0x07

//-------------------------------
// User Control settings
//-------------------------------
#define FIFO_EN        0x40
#define FIFO_DIS       0x00
#define I2C_MST_EN     0x60
#define I2C_MST_DIS    0x00
#define FIFO_RESET     0x04
#define I2C_MST_RESET  0x02
#define SIG_COND_RESET 0x41

//---------------------------------
// Power Management Setings One
//---------------------------------
#define DEVICE_RESET      0x80
#define SEEP              0x40
#define CYCLE             0x20
#define TEMP_DIS          0x08
#define INT_8MHz_OSC      0x00
#define PLL_GYROX_REF     0x01
#define PLL_GYROY_REF     0x02
#define PLL_GYROZ_REF     0x03
#define PLL_EXT_32KHz_REF 0x04
#define PLL_EXT_19KHz_REF 0x05

//---------------------------------
// Power Management Settings Two
//---------------------------------
#define LP_WAKE_CTRL_1Hz  0x00
#define LP_WAKE_CTRL_5Hz  0x40
#define LP_WAKE_CTRL_20Hz 0x80
#define LP_WAKE_CTRL_40Hz 0xC0

/**
 * MPU6050 triple axis digital gyroscope and triple axis digital accelerometer.
 */
class MPU6050 {

 public:
  /**
   *
   * @param sda - mbed pin to use for the SDA I2C line.
   * @param scl - mbed pin to use for the SCL I2C line.
   */
  MPU6050(PinName sda, PinName scl);

  bool initialize();

  /**
   * Self-test permits users to test the mechanical and electrical portions of the MPU6050.
   *
   * @return Contents of the self-test registers.
   *         Array of XA_TEST, YA_TEST, ZA_TEST, XG_TEST, YG_TEST and ZG_TEST.
   */
  void selfTest(uint8_t* TestReadings);

  /**
   * Set Sample Rate for the MPU-6050 senor output, FIFO, and DMP registers.
   *
   * @param Divider - The sample rate divider as a number from 0x00 - 0xFF.
   *
   */
  void setSampleRate(uint8_t Divider);

  /**
   * Get Sample Rate for the MPU-6050 senor output, FIFO, and DMP registers.
   *
   * @return Sample rate.
   *
   */
  uint8_t getSampleRate(void);

  /**
   * Set the external Frame Synchronization (FSYNC) pin sampling.
   *
   * @param Conf - The configuration setting as a number from 0x00 - 0xFF.
   *
   */
  void setFSYNCConfig(uint8_t Conf);

  /**
   * Get the Frame Synchronization (FSYNC) configuration.
   *
   * @return Current Frame Synchronization (FSYNC) configuration.
   *
   */
  uint8_t getFSYNCConfig(void);

  /**
   * Set the Digital Low Pass Filter (DLPF) setting for both gyro and accelemometer.
   *
   * @param Conf - The configuration setting as a number from 0-7.
   *
   */
  void setDLPFConfig(uint8_t Conf);

  /**
   * Get the Digital Low Pass Filter (DLPF) setting for both gyro and accelemometer.
   *
   * @return Current Digital Low Pass Filter (DLPF) setting.
   *
   */
  uint8_t getDLPFConfig(void);

  /**
   * Set the gyroscope self-test trigger and scale range configuration.
   *
   * @param GyroST - Trigger Gyroscope self-test setting. GYRO_ST_ON or GYRO_ST_OFF.
   * @param Scale - The full scale range setting for the gyroscope GFS_DISABLE, GFS_250dps,
   * GFS_500dps, GFS_1000dps, GFS_2000dps.
   *
   */
  void setGyroConfig(uint8_t GyroST, uint8_t Scale);

  /**
   * Get the gyroscope self-test trigger and scale range configuration.
   *
   * @return Current gyroscope self-test configuration.
   *
   */
  uint8_t getGyroConfig(void);

  /**
   * Set the gyroscope self-test trigger and scale range configuration.
   *
   * @param AccST - Trigger Accelerometer self-test setting. ACC_ST_ON or ACC_ST_OFF.
   * @param Scale - The full scale range setting for the accelemometer AFS_DSIABLE, AFS_2g, AFS_4g,
   * AFS_8g, AFS_16g.
   *
   */
  void setAccelConfig(uint8_t AccST, uint8_t Scale);

  /**
   * Get the accelerometer self-test trigger and scale range configuration.
   *
   * @return Current accelerometer self-test configuration.
   *
   */
  uint8_t getAccelConfig(void);

  /**
   * Set the setting for sensor measurements to be loaded to the FIFO register
   *
   * @param Setting - The configuration settings which determines the sensor measurements loaded
   * into the FIFO register. TEMP_FIFO_EN, GYRO_FIFO_EN, ACCEL_FIFO_EN, SLV2_FIFO_EN, SLV1_FIFO_EN,
   * SLV0_FIFO_EN, SLVZ_FIFO_EN, TEMPGYROACCEL_FIFO_EN, GYROACCEL_FIFO_EN,
   * TEMPGYROACCELSLVZ_FIFO_EN.
   */
  void setFIFO_Enable(uint8_t Setting);
  void enableGyroFIFO();

  /**
   * Get the setting for sensor measurements to be loaded to the FIFO register.
   *
   * @return Current setting which determines the sensor measurements loaded into the FIFO register.
   *
   */
  uint8_t getFIFO_Enable(void);

  /**
   * The most recent accelemometer measurements.
   *
   * @return Contents of Accelerometer measurement registers.
   *         Array of ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT as 16-bit 2's complement values.
   */
  void readAccelRaw(int16_t* accReadings);

  /**
   * The most recent accelemometer measurements.
   *
   * @return Contents of Accelerometer measurement registers.
   *         Array of ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT converted to g (m/(s*s)).
   */
  void readAccel(double* accReadings);

  /**
   * The most recent temperature measurements.
   *
   * @return TempReadings - Contents Temperature measurement registers.
   *         TEMP_OUT as 16-bit 2's complement value.
   */
  void readTemp(int16_t* TempReadings);

  /**
   * The most recent gyroscope measurements.
   *
   * @return Contents of Gyroscope measurement registers.
   *         Array of GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT as 16-bit 2's complement values.
   */
  void readGyroRaw(int16_t* gyroReadings);

  /**
   * The most recent gyroscope measurements.
   *
   * @return Contents of Gyroscope measurement registers.
   *         Array of GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT as 16-bit 2's complement values.
   */
  void readGyro(double* gyroReadings);

  /**
   * Resets the analog and digital paths of the gyroscope, accelemometer and temperature sensors.
   *
   * @param ResVal - The reset settings GYRO_RESET, ACCEL_RESET, TEMP_RESET, ALL_RESET.
   *
   */
  void sigPathReset(uint8_t ResVal);

  /**
   * Enable or disable FIFO buffer, I2C Master Mode, and primary I2C interface.
   *
   * @param Settings - The configuration settings for user control register.
   *           FIFO_EN, FIFO_DIS, I2C_MST_EN, I2C_MST_DIS, FIFO_RESET, I2C_MST_RESET,
   * SIG_COND_RESET.
   *
   */
  void setUserCtl(uint8_t Settings);

  /**
   * Get the user setting for FIFO buffer, I2C Master Mode, and primary I2C interface.
   *
   * @return Current user settings for FIFO buffer, I2C Master Mode, and primary I2C interface.
   *
   */
  uint8_t getUserCtl(void);

  /**
   * Set the power mode control settings.
   *
   * @param DevRes - Resets all internal resgisters to their default values. DEVICE_RESET or 0x00.
   * @param Sleep - Puts the MPU6050 into sleep mode. SEEP or 0x00.
   * @param Cycle - Cycles MPU6050 between sleep mode and waking up to take a single sample of data
   * from active sensors at a rate determined by LP_WAKE_CTRL register. CYCLE or 0x00.
   * @param Temp - Disables the temperature sensor. TEMP_DIS or 0x00.
   * @param Clock - Specifies the clock source of the device.
   *                      Settings INT_8MHz_OSC, PLL_GYROX_REF, PLL_GYROY_REF, PLL_GYROZ_REF,
   * PLL_EXT_32KHz_REF, PLL_EXT_19KHz_REF.
   *
   */
  void setPowerCtl_1(uint8_t DevRes, uint8_t Sleep, uint8_t Cycle, uint8_t Temp, uint8_t Clock);

  /**
   * Get the power mode and clock source.
   *
   * @return Current power mode control settings.
   *
   */
  uint8_t getPowerCtl_1(void);

  /**
   * Set frequency of wake-ups in Accelerometer Only Low Power Mode, as well as individual gyroscope
   * and accelemometer axes standby configuration.
   *
   * @param Conf -  Wake-up and axes standby configuration.
   *                      Configurations LP_WAKE_CTRL_1Hz, LP_WAKE_CTRL_5Hz, LP_WAKE_CTRL_20Hz,
   * LP_WAKE_CTRL_40Hz.
   *
   */
  void setPowerCtl_2(uint8_t Conf);

  /**
   * Get frequency of wake-ups in Accelerometer Only Low Power Mode, as well as individual gyroscope
   * and accelemometer axes standby status.
   *
   * @return Current wake-up and individual axes standby status.
   *
   */
  uint8_t getPowerCtl_2(void);

  /**
   * Read the number of samples currenly in the FIFO buffer.
   *
   * @return Current number of samples in the FIFO buffer.
   *
   */
  uint16_t getFIFOCount(void);

  /**
   * This register is used to write data to the FIFO buffer.
   *
   * @param Data -  Data to write to the FIFO buffer. Value from 0x00 - 0xFF.
   *
   */
  void FIFODataWrite(uint8_t Data);

  /**
   * This register is used to read data from the FIFO buffer.
   *
   * @return Data located in the FIFO buffer.
   *
   */
  uint8_t FIFODataRead(void);

  /**
   * Get the identity of the device.
   *
   * @return
   *
   */
  uint8_t getWhoAmI(void);

 private:
  I2C i2c_;
  double _accel_scale = 2048.0;
  double _gyro_scale = 16.4;

  /**
   * Read one byte from a register on the device.
   *
   * @param: - the address to be read from
   *
   * @return: the value of the data read
   */
  char SingleByteRead(char address);

  /**
   * Write one byte to a register on the device.
   *
   * @param:
      - address of the register to write to.
      - the value of the data to store
   */
  uint8_t SingleByteWrite(char address, char data);

  /**
   * Read several consecutive bytes on the device and store them in a given location.
   *
   * @param startAddress: The address of the first register to read from.
   * @param ptr_output: a pointer to the location to store the data being read
   * @param size: The number of bytes to read.
   */
  void multiByteRead(char startAddress, char* ptr_output, int size);

  /**
   * Write several consecutive bytes  on the device.
   *
   * @param startAddress: The address of the first register to write to.
   * @param ptr_data: Pointer to a location which contains the data to write.
   * @param size: The number of bytes to write.
   */
  uint8_t multiByteWrite(char startAddress, char* ptr_data, int size);
};

#endif /* MPU6050_H */
