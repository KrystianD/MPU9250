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

extern uint8_t mpuDelayMs(int delayMs);
extern uint8_t mpuDelayUs(int delayUs);
extern uint8_t mpuReadCommand(uint8_t cmd, uint8_t* data, uint8_t len);
extern uint8_t mpuSendCommand(uint8_t cmd, const uint8_t* data, uint8_t len);

int8_t writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

uint8_t mpuWriteSlaveReg(uint8_t addr, uint8_t reg, uint8_t val);
uint8_t mpuReadSlaveReg(uint8_t addr, uint8_t reg, uint8_t& val);

// dmp functions
uint8_t dmpInitialize();
bool dmpPacketAvailable();

uint8_t dmpSetFIFORate(uint8_t fifoRate);
uint8_t dmpGetFIFORate();
uint8_t dmpGetSampleStepSizeMS();
uint8_t dmpGetSampleFrequency();
int32_t dmpDecodeTemperature(int8_t tempReg);

// Register callbacks after a packet of FIFO data is processed
//uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
uint8_t dmpRunFIFORateProcesses();

// Setup FIFO for various output
uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

// Get Fixed Point data from FIFO
uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet = 0);
uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet = 0);
uint8_t dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet = 0);
uint8_t dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet = 0);
uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet = 0);
uint8_t dmpGetMag(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
uint8_t dmpGetLinearAccel(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetLinearAccel(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet = 0);
uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
uint8_t dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet = 0);
uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
uint8_t dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet = 0);
uint8_t dmpGetGyroSensor(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGyroSensor(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet = 0);
uint8_t dmpGetControlData(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetTemperature(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGravity(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetGravity(VectorInt16 *v, const uint8_t* packet = 0);
uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
uint8_t dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet = 0);
uint8_t dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet = 0);
uint8_t dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet = 0);
uint8_t dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet = 0);
uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet = 0);
uint8_t dmpGetEIS(int32_t *data, const uint8_t* packet = 0);

uint8_t dmpGetEuler(float *data, Quaternion *q);
uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

// Get Floating Point data from FIFO
uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet = 0);
uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet = 0);

uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed = NULL);

uint8_t dmpSetFIFOProcessedCallback(void (*func)(void));

uint8_t dmpInitFIFOParam();
uint8_t dmpCloseFIFO();
uint8_t dmpSetGyroDataSource(uint8_t source);
uint8_t dmpDecodeQuantizedAccel();
uint32_t dmpGetGyroSumOfSquare();
uint32_t dmpGetAccelSumOfSquare();
void dmpOverrideQuaternion(long *q);
uint16_t dmpGetFIFOPacketSize();
// mpu functions
void initialize();
bool testConnection();

// AUX_VDDIO register
uint8_t getAuxVDDIOLevel();
void setAuxVDDIOLevel(uint8_t level);

// SMPLRT_DIV register
uint8_t getRate();
void setRate(uint8_t rate);

// CONFIG register
uint8_t getExternalFrameSync();
void setExternalFrameSync(uint8_t sync);
uint8_t getDLPFMode();
void setDLPFMode(uint8_t bandwidth);

// GYRO_CONFIG register
uint8_t getFullScaleGyroRange();
void setFullScaleGyroRange(uint8_t range);

// ACCEL_CONFIG register
bool getAccelXSelfTest();
void setAccelXSelfTest(bool enabled);
bool getAccelYSelfTest();
void setAccelYSelfTest(bool enabled);
bool getAccelZSelfTest();
void setAccelZSelfTest(bool enabled);
uint8_t getFullScaleAccelRange();
void setFullScaleAccelRange(uint8_t range);
uint8_t getDHPFMode();
void setDHPFMode(uint8_t mode);

// FF_THR register
uint8_t getFreefallDetectionThreshold();
void setFreefallDetectionThreshold(uint8_t threshold);

// FF_DUR register
uint8_t getFreefallDetectionDuration();
void setFreefallDetectionDuration(uint8_t duration);

// MOT_THR register
uint8_t getMotionDetectionThreshold();
void setMotionDetectionThreshold(uint8_t threshold);

// MOT_DUR register
uint8_t getMotionDetectionDuration();
void setMotionDetectionDuration(uint8_t duration);

// ZRMOT_THR register
uint8_t getZeroMotionDetectionThreshold();
void setZeroMotionDetectionThreshold(uint8_t threshold);

// ZRMOT_DUR register
uint8_t getZeroMotionDetectionDuration();
void setZeroMotionDetectionDuration(uint8_t duration);

// FIFO_EN register
bool getTempFIFOEnabled();
void setTempFIFOEnabled(bool enabled);
bool getXGyroFIFOEnabled();
void setXGyroFIFOEnabled(bool enabled);
bool getYGyroFIFOEnabled();
void setYGyroFIFOEnabled(bool enabled);
bool getZGyroFIFOEnabled();
void setZGyroFIFOEnabled(bool enabled);
bool getAccelFIFOEnabled();
void setAccelFIFOEnabled(bool enabled);
bool getSlave2FIFOEnabled();
void setSlave2FIFOEnabled(bool enabled);
bool getSlave1FIFOEnabled();
void setSlave1FIFOEnabled(bool enabled);
bool getSlave0FIFOEnabled();
void setSlave0FIFOEnabled(bool enabled);

// I2C_MST_CTRL register
bool getMultiMasterEnabled();
void setMultiMasterEnabled(bool enabled);
bool getWaitForExternalSensorEnabled();
void setWaitForExternalSensorEnabled(bool enabled);
bool getSlave3FIFOEnabled();
void setSlave3FIFOEnabled(bool enabled);
bool getSlaveReadWriteTransitionEnabled();
void setSlaveReadWriteTransitionEnabled(bool enabled);
uint8_t getMasterClockSpeed();
void setMasterClockSpeed(uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
uint8_t getSlaveAddress(uint8_t num);
void setSlaveAddress(uint8_t num, uint8_t address);
uint8_t getSlaveRegister(uint8_t num);
void setSlaveRegister(uint8_t num, uint8_t reg);
bool getSlaveEnabled(uint8_t num);
void setSlaveEnabled(uint8_t num, bool enabled);
bool getSlaveWordByteSwap(uint8_t num);
void setSlaveWordByteSwap(uint8_t num, bool enabled);
bool getSlaveWriteMode(uint8_t num);
void setSlaveWriteMode(uint8_t num, bool mode);
bool getSlaveWordGroupOffset(uint8_t num);
void setSlaveWordGroupOffset(uint8_t num, bool enabled);
uint8_t getSlaveDataLength(uint8_t num);
void setSlaveDataLength(uint8_t num, uint8_t length);

// I2C_SLV* registers (Slave 4)
uint8_t getSlave4Address();
void setSlave4Address(uint8_t address);
uint8_t getSlave4Register();
void setSlave4Register(uint8_t reg);
void setSlave4OutputByte(uint8_t data);
bool getSlave4Enabled();
void setSlave4Enabled(bool enabled);
bool getSlave4InterruptEnabled();
void setSlave4InterruptEnabled(bool enabled);
bool getSlave4WriteMode();
void setSlave4WriteMode(bool mode);
uint8_t getSlave4MasterDelay();
void setSlave4MasterDelay(uint8_t delay);
uint8_t getSlate4InputByte();

// I2C_MST_STATUS register
bool getPassthroughStatus();
bool getSlave4IsDone();
bool getLostArbitration();
bool getSlave4Nack();
bool getSlave3Nack();
bool getSlave2Nack();
bool getSlave1Nack();
bool getSlave0Nack();

// INT_PIN_CFG register
bool getInterruptMode();
void setInterruptMode(bool mode);
bool getInterruptDrive();
void setInterruptDrive(bool drive);
bool getInterruptLatch();
void setInterruptLatch(bool latch);
bool getInterruptLatchClear();
void setInterruptLatchClear(bool clear);
bool getFSyncInterruptLevel();
void setFSyncInterruptLevel(bool level);
bool getFSyncInterruptEnabled();
void setFSyncInterruptEnabled(bool enabled);
bool getI2CBypassEnabled();
void setI2CBypassEnabled(bool enabled);
bool getClockOutputEnabled();
void setClockOutputEnabled(bool enabled);

// INT_ENABLE register
uint8_t getIntEnabled();
void setIntEnabled(uint8_t enabled);
bool getIntFreefallEnabled();
void setIntFreefallEnabled(bool enabled);
bool getIntMotionEnabled();
void setIntMotionEnabled(bool enabled);
bool getIntZeroMotionEnabled();
void setIntZeroMotionEnabled(bool enabled);
bool getIntFIFOBufferOverflowEnabled();
void setIntFIFOBufferOverflowEnabled(bool enabled);
bool getIntI2CMasterEnabled();
void setIntI2CMasterEnabled(bool enabled);
bool getIntDataReadyEnabled();
void setIntDataReadyEnabled(bool enabled);

// INT_STATUS register
uint8_t getIntStatus();
bool getIntFreefallStatus();
bool getIntMotionStatus();
bool getIntZeroMotionStatus();
bool getIntFIFOBufferOverflowStatus();
bool getIntI2CMasterStatus();
bool getIntDataReadyStatus();

// ACCEL_*OUT_* registers
void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
int16_t getAccelerationX();
int16_t getAccelerationY();
int16_t getAccelerationZ();

// TEMP_OUT_* registers
int16_t getTemperature();

// GYRO_*OUT_* registers
void getRotation(int16_t* x, int16_t* y, int16_t* z);
int16_t getRotationX();
int16_t getRotationY();
int16_t getRotationZ();

// EXT_SENS_DATA_* registers
uint8_t getExternalSensorByte(int position);
uint16_t getExternalSensorWord(int position);
uint32_t getExternalSensorDWord(int position);

// MOT_DETECT_STATUS register
bool getXNegMotionDetected();
bool getXPosMotionDetected();
bool getYNegMotionDetected();
bool getYPosMotionDetected();
bool getZNegMotionDetected();
bool getZPosMotionDetected();
bool getZeroMotionDetected();

// I2C_SLV*_DO register
void setSlaveOutputByte(uint8_t num, uint8_t data);

// I2C_MST_DELAY_CTRL register
bool getExternalShadowDelayEnabled();
void setExternalShadowDelayEnabled(bool enabled);
bool getSlaveDelayEnabled(uint8_t num);
void setSlaveDelayEnabled(uint8_t num, bool enabled);

// SIGNAL_PATH_RESET register
void resetGyroscopePath();
void resetAccelerometerPath();
void resetTemperaturePath();

// MOT_DETECT_CTRL register
uint8_t getAccelerometerPowerOnDelay();
void setAccelerometerPowerOnDelay(uint8_t delay);
uint8_t getFreefallDetectionCounterDecrement();
void setFreefallDetectionCounterDecrement(uint8_t decrement);
uint8_t getMotionDetectionCounterDecrement();
void setMotionDetectionCounterDecrement(uint8_t decrement);

// USER_CTRL register
bool getFIFOEnabled();
void setFIFOEnabled(bool enabled);
bool getI2CMasterModeEnabled();
void setI2CMasterModeEnabled(bool enabled);
void switchSPIEnabled(bool enabled);
void resetFIFO();
void resetI2CMaster();
void resetSensors();

// PWR_MGMT_1 register
void reset();
bool getSleepEnabled();
void setSleepEnabled(bool enabled);
bool getWakeCycleEnabled();
void setWakeCycleEnabled(bool enabled);
bool getTempSensorEnabled();
void setTempSensorEnabled(bool enabled);
uint8_t getClockSource();
void setClockSource(uint8_t source);

// PWR_MGMT_2 register
uint8_t getWakeFrequency();
void setWakeFrequency(uint8_t frequency);
bool getStandbyXAccelEnabled();
void setStandbyXAccelEnabled(bool enabled);
bool getStandbyYAccelEnabled();
void setStandbyYAccelEnabled(bool enabled);
bool getStandbyZAccelEnabled();
void setStandbyZAccelEnabled(bool enabled);
bool getStandbyXGyroEnabled();
void setStandbyXGyroEnabled(bool enabled);
bool getStandbyYGyroEnabled();
void setStandbyYGyroEnabled(bool enabled);
bool getStandbyZGyroEnabled();
void setStandbyZGyroEnabled(bool enabled);

// FIFO_COUNT_* registers
uint16_t getFIFOCount();

// FIFO_R_W register
uint8_t getFIFOByte();
void setFIFOByte(uint8_t data);
void getFIFOBytes(uint8_t *data, uint8_t length);

// WHO_AM_I register
uint8_t getDeviceID();
void setDeviceID(uint8_t id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
uint8_t getOTPBankValid();
void setOTPBankValid(bool enabled);
int8_t getXGyroOffsetTC();
void setXGyroOffsetTC(int8_t offset);

// YG_OFFS_TC register
int8_t getYGyroOffsetTC();
void setYGyroOffsetTC(int8_t offset);

// ZG_OFFS_TC register
int8_t getZGyroOffsetTC();
void setZGyroOffsetTC(int8_t offset);

// X_FINE_GAIN register
int8_t getXFineGain();
void setXFineGain(int8_t gain);

// Y_FINE_GAIN register
int8_t getYFineGain();
void setYFineGain(int8_t gain);

// Z_FINE_GAIN register
int8_t getZFineGain();
void setZFineGain(int8_t gain);

// XA_OFFS_* registers
int16_t getXAccelOffset();
void setXAccelOffset(int16_t offset);

// YA_OFFS_* register
int16_t getYAccelOffset();
void setYAccelOffset(int16_t offset);

// ZA_OFFS_* register
int16_t getZAccelOffset();
void setZAccelOffset(int16_t offset);

// XG_OFFS_USR* registers
int16_t getXGyroOffset();
void setXGyroOffset(int16_t offset);

// YG_OFFS_USR* register
int16_t getYGyroOffset();
void setYGyroOffset(int16_t offset);

// ZG_OFFS_USR* register
int16_t getZGyroOffset();
void setZGyroOffset(int16_t offset);

// INT_ENABLE register (DMP functions)
bool getIntPLLReadyEnabled();
void setIntPLLReadyEnabled(bool enabled);
bool getIntDMPEnabled();
void setIntDMPEnabled(bool enabled);

// DMP_INT_STATUS
bool getDMPInt5Status();
bool getDMPInt4Status();
bool getDMPInt3Status();
bool getDMPInt2Status();
bool getDMPInt1Status();
bool getDMPInt0Status();

// INT_STATUS register (DMP functions)
bool getIntPLLReadyStatus();
bool getIntDMPStatus();

// USER_CTRL register (DMP functions)
bool getDMPEnabled();
void setDMPEnabled(bool enabled);
void resetDMP();

// BANK_SEL register
void setMemoryBank(uint8_t bank, bool prefetchEnabled = false, bool userBank = false);

// MEM_START_ADDR register
void setMemoryStartAddress(uint8_t address);

// MEM_R_W register
uint8_t readMemoryByte();
void writeMemoryByte(uint8_t data);
void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0);
bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0, bool verify = true, bool useProgMem = false);
bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0, bool verify = true);

bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem = false);
bool writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

// DMP_CFG_1 register
uint8_t getDMPConfig1();
void setDMPConfig1(uint8_t config);

// DMP_CFG_2 register
uint8_t getDMPConfig2();
void setDMPConfig2(uint8_t config);

int mymain();

#endif
