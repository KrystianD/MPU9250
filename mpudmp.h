// Original Arduino I2C Code by Jeff Rowberg Converted to Raspberry PI/SPI by Mat Oxenham converted to plain C++ by Krystian Dużyński

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef __MPUDMP_H__
#define __MPUDMP_H__

#include <stdint.h>
#include <cmath>
#include <sys/time.h>
#include <stdio.h>
#include "helper_3dmath.h"
#include "mpudmp_registers.h"

extern void mpuDelayMs(int delayMs);
extern void mpuDelayUs(int delayUs);

#ifndef MPU9250_CLASSNAME
#define MPU9250_CLASSNAME MPU9250
#endif

#ifdef MPU9250_MULTIPLE_INSTANCES
extern uint8_t mpuReadCommand(uint8_t cmd, uint8_t* data, uint16_t len, void* userdata);
extern uint8_t mpuSendCommand(uint8_t cmd, const uint8_t* data, uint16_t len, void* userdata);
#else
extern uint8_t mpuReadCommand(uint8_t cmd, uint8_t* data, uint16_t len);
extern uint8_t mpuSendCommand(uint8_t cmd, const uint8_t* data, uint16_t len);
#endif

#ifdef MPU9250_MULTIPLE_INSTANCES
#define METHOD
#else
#define METHOD static
#endif

class MPU9250_CLASSNAME
{
public:
#ifdef MPU9250_MULTIPLE_INSTANCES
	void *userdata;
#endif

	METHOD bool writeSlaveReg(uint8_t addr, uint8_t reg, uint8_t val);
	METHOD bool readSlaveReg(uint8_t addr, uint8_t reg, uint8_t& val);

// dmp functions
	METHOD void flushFIFO();
	METHOD uint8_t dmpInitialize();
	METHOD bool dmpPacketAvailable();

	METHOD uint8_t dmpSetFIFORate(uint8_t fifoRate);
	METHOD uint8_t dmpGetFIFORate();
	METHOD uint8_t dmpGetSampleStepSizeMS();
	METHOD uint8_t dmpGetSampleFrequency();
	METHOD int32_t dmpDecodeTemperature(int8_t tempReg);

// Register callbacks after a packet of FIFO data is processed
//uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
	METHOD uint8_t dmpRunFIFORateProcesses();

// Get Fixed Point data from FIFO
	METHOD uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetMag(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
	METHOD uint8_t dmpGetLinearAccel(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetLinearAccel(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetGyroSensor(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetGyroSensor(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetControlData(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetTemperature(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetGravity(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetEIS(int32_t *data, const uint8_t* packet = 0);

// Get Floating Point data from FIFO
	METHOD uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet = 0);
	METHOD uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet = 0);

	METHOD uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
	METHOD uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed = NULL);

	METHOD uint8_t dmpSetFIFOProcessedCallback(void (*func)(void));

	METHOD uint8_t dmpInitFIFOParam();
	METHOD uint8_t dmpCloseFIFO();
	METHOD uint8_t dmpSetGyroDataSource(uint8_t source);
	METHOD uint8_t dmpDecodeQuantizedAccel();
	METHOD uint32_t dmpGetGyroSumOfSquare();
	METHOD uint32_t dmpGetAccelSumOfSquare();
	METHOD void dmpOverrideQuaternion(long *q);
	METHOD uint16_t dmpGetFIFOPacketSize();
// mpu functions
	METHOD void initialize();
	METHOD bool testConnection();

// AUX_VDDIO register
	METHOD uint8_t getAuxVDDIOLevel();
	METHOD void setAuxVDDIOLevel(uint8_t level);

// SMPLRT_DIV register
	METHOD uint8_t getRate();
	METHOD void setRate(uint8_t rate);

// CONFIG register
	METHOD uint8_t getExternalFrameSync();
	METHOD void setExternalFrameSync(uint8_t sync);
	METHOD uint8_t getDLPFMode();
	METHOD void setDLPFMode(uint8_t bandwidth);

// GYRO_CONFIG register
	METHOD uint8_t getFullScaleGyroRange();
	METHOD void setFullScaleGyroRange(uint8_t range);

// ACCEL_CONFIG register
	METHOD bool getAccelXSelfTest();
	METHOD void setAccelXSelfTest(bool enabled);
	METHOD bool getAccelYSelfTest();
	METHOD void setAccelYSelfTest(bool enabled);
	METHOD bool getAccelZSelfTest();
	METHOD void setAccelZSelfTest(bool enabled);
	METHOD uint8_t getFullScaleAccelRange();
	METHOD void setFullScaleAccelRange(uint8_t range);
	METHOD uint8_t getDHPFMode();
	METHOD void setDHPFMode(uint8_t mode);

// FF_THR register
	METHOD uint8_t getFreefallDetectionThreshold();
	METHOD void setFreefallDetectionThreshold(uint8_t threshold);

// FF_DUR register
	METHOD uint8_t getFreefallDetectionDuration();
	METHOD void setFreefallDetectionDuration(uint8_t duration);

// MOT_THR register
	METHOD uint8_t getMotionDetectionThreshold();
	METHOD void setMotionDetectionThreshold(uint8_t threshold);

// MOT_DUR register
	METHOD uint8_t getMotionDetectionDuration();
	METHOD void setMotionDetectionDuration(uint8_t duration);

// ZRMOT_THR register
	METHOD uint8_t getZeroMotionDetectionThreshold();
	METHOD void setZeroMotionDetectionThreshold(uint8_t threshold);

// ZRMOT_DUR register
	METHOD uint8_t getZeroMotionDetectionDuration();
	METHOD void setZeroMotionDetectionDuration(uint8_t duration);

// FIFO_EN register
	METHOD bool getTempFIFOEnabled();
	METHOD void setTempFIFOEnabled(bool enabled);
	METHOD bool getXGyroFIFOEnabled();
	METHOD void setXGyroFIFOEnabled(bool enabled);
	METHOD bool getYGyroFIFOEnabled();
	METHOD void setYGyroFIFOEnabled(bool enabled);
	METHOD bool getZGyroFIFOEnabled();
	METHOD void setZGyroFIFOEnabled(bool enabled);
	METHOD bool getAccelFIFOEnabled();
	METHOD void setAccelFIFOEnabled(bool enabled);
	METHOD bool getSlave2FIFOEnabled();
	METHOD void setSlave2FIFOEnabled(bool enabled);
	METHOD bool getSlave1FIFOEnabled();
	METHOD void setSlave1FIFOEnabled(bool enabled);
	METHOD bool getSlave0FIFOEnabled();
	METHOD void setSlave0FIFOEnabled(bool enabled);

// I2C_MST_CTRL register
	METHOD bool getMultiMasterEnabled();
	METHOD void setMultiMasterEnabled(bool enabled);
	METHOD bool getWaitForExternalSensorEnabled();
	METHOD void setWaitForExternalSensorEnabled(bool enabled);
	METHOD bool getSlave3FIFOEnabled();
	METHOD void setSlave3FIFOEnabled(bool enabled);
	METHOD bool getSlaveReadWriteTransitionEnabled();
	METHOD void setSlaveReadWriteTransitionEnabled(bool enabled);
	METHOD uint8_t getMasterClockSpeed();
	METHOD void setMasterClockSpeed(uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
	METHOD uint8_t getSlaveAddress(uint8_t num);
	METHOD void setSlaveAddress(uint8_t num, uint8_t address);
	METHOD uint8_t getSlaveRegister(uint8_t num);
	METHOD void setSlaveRegister(uint8_t num, uint8_t reg);
	METHOD bool getSlaveEnabled(uint8_t num);
	METHOD void setSlaveEnabled(uint8_t num, bool enabled);
	METHOD bool getSlaveWordByteSwap(uint8_t num);
	METHOD void setSlaveWordByteSwap(uint8_t num, bool enabled);
	METHOD bool getSlaveWriteMode(uint8_t num);
	METHOD void setSlaveWriteMode(uint8_t num, bool mode);
	METHOD bool getSlaveWordGroupOffset(uint8_t num);
	METHOD void setSlaveWordGroupOffset(uint8_t num, bool enabled);
	METHOD uint8_t getSlaveDataLength(uint8_t num);
	METHOD void setSlaveDataLength(uint8_t num, uint8_t length);

// I2C_SLV* registers (Slave 4)
	METHOD uint8_t getSlave4Address();
	METHOD void setSlave4Address(uint8_t address);
	METHOD uint8_t getSlave4Register();
	METHOD void setSlave4Register(uint8_t reg);
	METHOD void setSlave4OutputByte(uint8_t data);
	METHOD bool getSlave4Enabled();
	METHOD void setSlave4Enabled(bool enabled);
	METHOD bool getSlave4InterruptEnabled();
	METHOD void setSlave4InterruptEnabled(bool enabled);
	METHOD bool getSlave4WriteMode();
	METHOD void setSlave4WriteMode(bool mode);
	METHOD uint8_t getSlave4MasterDelay();
	METHOD void setSlave4MasterDelay(uint8_t delay);
	METHOD uint8_t getSlate4InputByte();

// I2C_MST_STATUS register
	METHOD bool getPassthroughStatus();
	METHOD bool getSlave4IsDone();
	METHOD bool getLostArbitration();
	METHOD bool getSlave4Nack();
	METHOD bool getSlave3Nack();
	METHOD bool getSlave2Nack();
	METHOD bool getSlave1Nack();
	METHOD bool getSlave0Nack();

// INT_PIN_CFG register
	METHOD bool getInterruptMode();
	METHOD void setInterruptMode(bool mode);
	METHOD bool getInterruptDrive();
	METHOD void setInterruptDrive(bool drive);
	METHOD bool getInterruptLatch();
	METHOD void setInterruptLatch(bool latch);
	METHOD bool getInterruptLatchClear();
	METHOD void setInterruptLatchClear(bool clear);
	METHOD bool getFSyncInterruptLevel();
	METHOD void setFSyncInterruptLevel(bool level);
	METHOD bool getFSyncInterruptEnabled();
	METHOD void setFSyncInterruptEnabled(bool enabled);
	METHOD bool getI2CBypassEnabled();
	METHOD void setI2CBypassEnabled(bool enabled);
	METHOD bool getClockOutputEnabled();
	METHOD void setClockOutputEnabled(bool enabled);

// INT_ENABLE register
	METHOD uint8_t getIntEnabled();
	METHOD void setIntEnabled(uint8_t enabled);
	METHOD bool getIntFreefallEnabled();
	METHOD void setIntFreefallEnabled(bool enabled);
	METHOD bool getIntMotionEnabled();
	METHOD void setIntMotionEnabled(bool enabled);
	METHOD bool getIntZeroMotionEnabled();
	METHOD void setIntZeroMotionEnabled(bool enabled);
	METHOD bool getIntFIFOBufferOverflowEnabled();
	METHOD void setIntFIFOBufferOverflowEnabled(bool enabled);
	METHOD bool getIntI2CMasterEnabled();
	METHOD void setIntI2CMasterEnabled(bool enabled);
	METHOD bool getIntDataReadyEnabled();
	METHOD void setIntDataReadyEnabled(bool enabled);

// INT_STATUS register
	METHOD uint8_t getIntStatus();
	METHOD bool getIntFreefallStatus();
	METHOD bool getIntMotionStatus();
	METHOD bool getIntZeroMotionStatus();
	METHOD bool getIntFIFOBufferOverflowStatus();
	METHOD bool getIntI2CMasterStatus();
	METHOD bool getIntDataReadyStatus();

// ACCEL_*OUT_* registers
	METHOD void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
	METHOD void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
	METHOD void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
	METHOD int16_t getAccelerationX();
	METHOD int16_t getAccelerationY();
	METHOD int16_t getAccelerationZ();

// TEMP_OUT_* registers
	METHOD int16_t getTemperature();

// GYRO_*OUT_* registers
	METHOD void getRotation(int16_t* x, int16_t* y, int16_t* z);
	METHOD int16_t getRotationX();
	METHOD int16_t getRotationY();
	METHOD int16_t getRotationZ();

// EXT_SENS_DATA_* registers
	METHOD uint8_t getExternalSensorByte(int position);
	METHOD uint16_t getExternalSensorWord(int position);
	METHOD uint32_t getExternalSensorDWord(int position);

// MOT_DETECT_STATUS register
	METHOD bool getXNegMotionDetected();
	METHOD bool getXPosMotionDetected();
	METHOD bool getYNegMotionDetected();
	METHOD bool getYPosMotionDetected();
	METHOD bool getZNegMotionDetected();
	METHOD bool getZPosMotionDetected();
	bool getZeroMotionDetected();

// I2C_SLV*_DO register
	METHOD void setSlaveOutputByte(uint8_t num, uint8_t data);

// I2C_MST_DELAY_CTRL register
	METHOD bool getExternalShadowDelayEnabled();
	METHOD void setExternalShadowDelayEnabled(bool enabled);
	METHOD bool getSlaveDelayEnabled(uint8_t num);
	METHOD void setSlaveDelayEnabled(uint8_t num, bool enabled);

// SIGNAL_PATH_RESET register
	METHOD void resetGyroscopePath();
	METHOD void resetAccelerometerPath();
	METHOD void resetTemperaturePath();

// MOT_DETECT_CTRL register
	METHOD uint8_t getAccelerometerPowerOnDelay();
	METHOD void setAccelerometerPowerOnDelay(uint8_t delay);
	METHOD uint8_t getFreefallDetectionCounterDecrement();
	METHOD void setFreefallDetectionCounterDecrement(uint8_t decrement);
	METHOD uint8_t getMotionDetectionCounterDecrement();
	METHOD void setMotionDetectionCounterDecrement(uint8_t decrement);

// USER_CTRL register
	METHOD bool getFIFOEnabled();
	METHOD void setFIFOEnabled(bool enabled);
	METHOD bool getI2CMasterModeEnabled();
	METHOD void setI2CMasterModeEnabled(bool enabled);
	METHOD void switchSPIEnabled(bool enabled);
	METHOD void resetFIFO();
	METHOD void resetI2CMaster();
	METHOD void resetSensors();

// PWR_MGMT_1 register
	METHOD void reset();
	METHOD bool getSleepEnabled();
	METHOD void setSleepEnabled(bool enabled);
	METHOD bool getWakeCycleEnabled();
	METHOD void setWakeCycleEnabled(bool enabled);
	METHOD bool getTempSensorEnabled();
	METHOD void setTempSensorEnabled(bool enabled);
	METHOD uint8_t getClockSource();
	METHOD void setClockSource(uint8_t source);

// PWR_MGMT_2 register
	METHOD uint8_t getWakeFrequency();
	METHOD void setWakeFrequency(uint8_t frequency);
	METHOD bool getStandbyXAccelEnabled();
	METHOD void setStandbyXAccelEnabled(bool enabled);
	METHOD bool getStandbyYAccelEnabled();
	METHOD void setStandbyYAccelEnabled(bool enabled);
	METHOD bool getStandbyZAccelEnabled();
	METHOD void setStandbyZAccelEnabled(bool enabled);
	METHOD bool getStandbyXGyroEnabled();
	METHOD void setStandbyXGyroEnabled(bool enabled);
	METHOD bool getStandbyYGyroEnabled();
	METHOD void setStandbyYGyroEnabled(bool enabled);
	METHOD bool getStandbyZGyroEnabled();
	METHOD void setStandbyZGyroEnabled(bool enabled);

// FIFO_COUNT_* registers
	METHOD uint16_t getFIFOCount();

// FIFO_R_W register
	METHOD uint8_t getFIFOByte();
	METHOD void setFIFOByte(uint8_t data);
	METHOD void getFIFOBytes(uint8_t *data, uint16_t length);

// WHO_AM_I register
	METHOD uint8_t getDeviceID();
	METHOD void setDeviceID(uint8_t id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
	METHOD uint8_t getOTPBankValid();
	METHOD void setOTPBankValid(bool enabled);
	METHOD int8_t getXGyroOffsetTC();
	METHOD void setXGyroOffsetTC(int8_t offset);

// YG_OFFS_TC register
	METHOD int8_t getYGyroOffsetTC();
	METHOD void setYGyroOffsetTC(int8_t offset);

// ZG_OFFS_TC register
	METHOD int8_t getZGyroOffsetTC();
	METHOD void setZGyroOffsetTC(int8_t offset);

// X_FINE_GAIN register
	METHOD int8_t getXFineGain();
	METHOD void setXFineGain(int8_t gain);

// Y_FINE_GAIN register
	METHOD int8_t getYFineGain();
	METHOD void setYFineGain(int8_t gain);

// Z_FINE_GAIN register
	METHOD int8_t getZFineGain();
	METHOD void setZFineGain(int8_t gain);

// XA_OFFS_* registers
	METHOD int16_t getXAccelOffset();
	METHOD void setXAccelOffset(int16_t offset);

// YA_OFFS_* register
	METHOD int16_t getYAccelOffset();
	METHOD void setYAccelOffset(int16_t offset);

// ZA_OFFS_* register
	METHOD int16_t getZAccelOffset();
	METHOD void setZAccelOffset(int16_t offset);

// XG_OFFS_USR* registers
	METHOD int16_t getXGyroOffset();
	METHOD void setXGyroOffset(int16_t offset);

// YG_OFFS_USR* register
	METHOD int16_t getYGyroOffset();
	METHOD void setYGyroOffset(int16_t offset);

// ZG_OFFS_USR* register
	METHOD int16_t getZGyroOffset();
	METHOD void setZGyroOffset(int16_t offset);

// INT_ENABLE register (DMP functions)
	METHOD bool getIntPLLReadyEnabled();
	METHOD void setIntPLLReadyEnabled(bool enabled);
	METHOD bool getIntDMPEnabled();
	METHOD void setIntDMPEnabled(bool enabled);

// DMP_INT_STATUS
	METHOD bool getDMPInt5Status();
	METHOD bool getDMPInt4Status();
	METHOD bool getDMPInt3Status();
	METHOD bool getDMPInt2Status();
	METHOD bool getDMPInt1Status();
	METHOD bool getDMPInt0Status();

// INT_STATUS register (DMP functions)
	METHOD bool getIntPLLReadyStatus();
	METHOD bool getIntDMPStatus();

// USER_CTRL register (DMP functions)
	METHOD bool getDMPEnabled();
	METHOD void setDMPEnabled(bool enabled);
	METHOD void resetDMP();

// BANK_SEL register
	METHOD void setMemoryBank(uint8_t bank, bool prefetchEnabled = false, bool userBank = false);

// MEM_START_ADDR register
	METHOD void setMemoryStartAddress(uint8_t address);

// MEM_R_W register
	METHOD uint8_t readMemoryByte();
	METHOD void writeMemoryByte(uint8_t data);
	METHOD void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0);
	METHOD bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0, bool verify = true, bool useProgMem = false);
	METHOD bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0, bool verify = true);

	METHOD bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem = false);
	METHOD bool writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

// DMP_CFG_1 register
	METHOD uint8_t getDMPConfig1();
	METHOD void setDMPConfig1(uint8_t config);

// DMP_CFG_2 register
	METHOD uint8_t getDMPConfig2();
	METHOD void setDMPConfig2(uint8_t config);

// magnetometer
METHOD bool initMagnetometer();

// private:
	METHOD int8_t readBit(uint8_t regAddr, uint8_t bitStart, uint8_t *data);
	METHOD int8_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
	METHOD bool writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
	METHOD bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
	METHOD int16_t readByte(uint8_t regAddr, uint8_t *data);
	METHOD bool writeByte(uint8_t regAddr, uint8_t data);
	METHOD int16_t readBytes(uint8_t regAddr, uint16_t length, uint8_t *data);
	METHOD bool writeBytes(uint8_t regAddr, uint16_t length, uint8_t* data);
	METHOD void writeWord(uint8_t regAddr, uint16_t data);
};

int mymain();

#endif
