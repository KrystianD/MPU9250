#include "mpudmp.h"
#include "mpudmp_internal.h"

#include <string.h>

/////// IO FUNCTIONS ////////

#define READ_FLAG 0x80

// bit access
int8_t MPU9250_CLASSNAME::readBit(uint8_t regAddr, uint8_t bitStart, uint8_t *data)
{
	uint8_t val;
	readByte(regAddr, &val);
	int8_t ret = val & bitStart;
	data[0] = ret;
	return 1;
}
int8_t MPU9250_CLASSNAME::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//    ->010 shifted
	uint8_t count, b;
	if ((count = readByte(regAddr, &b)) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
	return count;
}

bool MPU9250_CLASSNAME::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t b;
	readByte(regAddr, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return writeByte(regAddr, b);
}
bool MPU9250_CLASSNAME::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t b;
	if (readByte(regAddr, &b) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		return writeByte(regAddr, b);
	}
	else
	{
		return false;
	}
}

// byte access
int8_t MPU9250_CLASSNAME::readByte(uint8_t regAddr, uint8_t *data)
{
	return readBytes(regAddr, 1, data);
}

int8_t MPU9250_CLASSNAME::writeByte(uint8_t regAddr, uint8_t data)
{
	return writeBytes(regAddr, 1, &data);
}

int8_t MPU9250_CLASSNAME::readBytes(uint8_t regAddr, uint8_t length, uint8_t *data)
{
	bool r;
#ifdef MPU9250_MULTIPLE_INSTANCES
	r = mpuReadCommand(regAddr, data, length, userdata);
#else
	r = mpuReadCommand(regAddr, data, length);
#endif
	if (!r)
	{
		printf("err");
		// for (;;);
	}
	// mpuDelayUs(50);
	return length;
}

bool MPU9250_CLASSNAME::writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data)
{
	bool r;
#ifdef MPU9250_MULTIPLE_INSTANCES
	r = mpuSendCommand(regAddr, data, length, userdata);
#else
	r = mpuSendCommand(regAddr, data, length);
#endif
	if (!r)
	{
		printf("err");
		// for (;;);
	}
	return true;
}

// word access
void MPU9250_CLASSNAME::writeWord(uint8_t regAddr, uint16_t data)
{
	uint8_t tbuffer[2];
	tbuffer[0] = data >> 8;
	tbuffer[1] = data & 0xff;
	writeBytes(regAddr, 2, tbuffer);
}

/////// END IO FUNCTIONS ////////

// DMP code
uint8_t *dmpPacketBuffer;
uint16_t dmpPacketSize;

#include "helper_3dmath.h"

bool MPU9250_CLASSNAME::dmpPacketAvailable()
{
	return getFIFOCount() >= dmpGetFIFOPacketSize();
}

uint8_t MPU9250_CLASSNAME::dmpGetQuaternion(int32_t *data, const uint8_t* packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0) packet = dmpPacketBuffer;
	data[0] = ((packet[0] << 24) + (packet[1] << 16) + (packet[2] << 8) + packet[3]);
	data[1] = ((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8) + packet[7]);
	data[2] = ((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11]);
	data[3] = ((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8) + packet[15]);
	return 0;
}
uint8_t MPU9250_CLASSNAME::dmpGetQuaternion(int16_t *data, const uint8_t* packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	if (packet == 0) packet = dmpPacketBuffer;
	data[0] = ((packet[0] << 8) + packet[1]);
	data[1] = ((packet[4] << 8) + packet[5]);
	data[2] = ((packet[8] << 8) + packet[9]);
	data[3] = ((packet[12] << 8) + packet[13]);
	return 0;
}
// uint8_t MPU9250_CLASSNAME::dmpGetQuaternion(Quaternion *q, const uint8_t* packet)
uint8_t MPU9250_CLASSNAME::dmpGetQuaternionFloat(float *data, const uint8_t* packet)
{
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	int16_t qI[4];
	uint8_t status = dmpGetQuaternion(qI, packet);
	if (status == 0)
	{
		data[0] = (float)qI[0] / 16384.0f;
		data[1] = (float)qI[1] / 16384.0f;
		data[2] = (float)qI[2] / 16384.0f;
		data[3] = (float)qI[3] / 16384.0f;
		return 0;
	}
	return status; // int16 return value, indicates error if this line is reached
}
// uint8_t MPU9250_CLASSNAME::dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t MPU9250_CLASSNAME::dmpGetLinearAccel(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity)
// {
	// // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
	// v->x = vRaw->x - gravity->x * 8192;
	// v->y = vRaw->y - gravity->y * 8192;
	// v->z = vRaw->z - gravity->z * 8192;
	// return 0;
// }
// uint8_t MPU9250_CLASSNAME::dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q)
// {
	// // rotate measured 3D acceleration vector into original state
	// // frame of reference based on orientation quaternion
	// memcpy(v, vReal, sizeof(VectorInt16));
	// v->rotate(q);
	// return 0;
// }
// uint8_t MPU9250_CLASSNAME::dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetGravity(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetGravity(VectorFloat *v, Quaternion *q)
// {
	// v->x = 2 * (q->x * q->z - q->w * q->y);
	// v->y = 2 * (q->w * q->x + q->y * q->z);
	// v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
	// return 0;
// }
// uint8_t MPU9250_CLASSNAME::dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetEIS(long *data, const uint8_t* packet);

// uint8_t MPU9250_CLASSNAME::dmpGetEuler(float *data, Quaternion *q)
// {
	// data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1); // psi
	// data[1] = -asin(2 * q->x * q->z + 2 * q->w * q->y);                      // theta
	// data[2] = atan2(2 * q->y * q->z - 2 * q->w * q->x, 2 * q->w * q->w + 2 * q->z * q->z - 1); // phi
	// return 0;
// }
// uint8_t MPU9250_CLASSNAME::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
// {
	// // yaw: (about Z axis)
	// data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);
	// // pitch: (nose up/down, about Y axis)
	// data[1] = atan(gravity->x / sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
	// // roll: (tilt left/right, about X axis)
	// data[2] = atan(gravity->y / sqrt(gravity->x * gravity->x + gravity->z * gravity->z));
	// return 0;
// }

// uint8_t MPU9250_CLASSNAME::dmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t MPU9250_CLASSNAME::dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t MPU9250_CLASSNAME::dmpProcessFIFOPacket(const unsigned char *dmpData)
{
	/*for (uint8_t k = 0; k < dmpPacketSize; k++) {
	    if (dmpData[k] < 0x10) Serial.print("0");
	    Serial.print(dmpData[k], HEX);
	    Serial.print(" ");
	}
	Serial.print("\n");*/
	//Serial.println((uint16_t)dmpPacketBuffer);
	return 0;
}
uint8_t MPU9250_CLASSNAME::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed)
{
	uint8_t status;
	uint8_t buf[dmpPacketSize];
	for (uint8_t i = 0; i < numPackets; i++)
	{
		// read packet from FIFO
		getFIFOBytes(buf, dmpPacketSize);

		// process packet
		if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;

		// increment external process count variable, if supplied
		if (processed != 0) (*processed)++;
	}
	return 0;
}

// uint8_t MPU9250_CLASSNAME::dmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t MPU9250_CLASSNAME::dmpInitFIFOParam();
// uint8_t MPU9250_CLASSNAME::dmpCloseFIFO();
// uint8_t MPU9250_CLASSNAME::dmpSetGyroDataSource(uint_fast8_t source);
// uint8_t MPU9250_CLASSNAME::dmpDecodeQuantizedAccel();
// uint32_t MPU9250_CLASSNAME::dmpGetGyroSumOfSquare();
// uint32_t MPU9250_CLASSNAME::dmpGetAccelSumOfSquare();
// void MPU9250_CLASSNAME::dmpOverrideQuaternion(long *q);
uint16_t MPU9250_CLASSNAME::dmpGetFIFOPacketSize()
{
	return dmpPacketSize;
}

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU9250_CLASSNAME::initialize()
{
	setClockSource(MPU9250_CLOCK_PLL_XGYRO);
#ifdef	DEBUG
	printf("Clock source: %i %i\r\n", MPU9250_CLOCK_PLL_XGYRO, getClockSource());
#endif
	setFullScaleGyroRange(MPU9250_GYRO_FS_250);
#ifdef	DEBUG
	printf("FullScaleGyroRange: %i %i\r\n", MPU9250_GYRO_FS_250, getFullScaleGyroRange());
#endif
	setFullScaleAccelRange(MPU9250_ACCEL_FS_2);
#ifdef	DEBUG
	printf("FullScaleAccelRange: %i %i\r\n", MPU9250_ACCEL_FS_2, getFullScaleAccelRange());
#endif
	setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU9250_CLASSNAME::testConnection()
{
	return getDeviceID() == 0x38;
}

// AUX_VDDIO register (InvenSense demo code calls this RA_*G_OFFS_TC)

/** Get the auxiliary I2C supply voltage level.
 * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
 * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
 * the MPU-6000, which does not have a VLOGIC pin.
 * @return I2C supply voltage level (0=VLOGIC, 1=VDD)
 */
uint8_t MPU9250_CLASSNAME::getAuxVDDIOLevel()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_YG_OFFS_TC, MPU9250_TC_PWR_MODE_BIT, buffer);
	return buffer[0];
}
/** Set the auxiliary I2C supply voltage level.
 * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
 * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
 * the MPU-6000, which does not have a VLOGIC pin.
 * @param level I2C supply voltage level (0=VLOGIC, 1=VDD)
 */
void MPU9250_CLASSNAME::setAuxVDDIOLevel(uint8_t level)
{
	writeBit(MPU9250_RA_YG_OFFS_TC, MPU9250_TC_PWR_MODE_BIT, level);
}

// SMPLRT_DIV register

/** Get gyroscope output rate divider.
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current sample rate
 * @see MPU9250_RA_SMPLRT_DIV
 */
uint8_t MPU9250_CLASSNAME::getRate()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_SMPLRT_DIV, buffer);
	return buffer[0];
}
/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU9250_RA_SMPLRT_DIV
 */
void MPU9250_CLASSNAME::setRate(uint8_t rate)
{
	writeByte(MPU9250_RA_SMPLRT_DIV, rate);
}

// CONFIG register

/** Get external FSYNC configuration.
 * Configures the external Frame Synchronization (FSYNC) pin sampling. An
 * external signal connected to the FSYNC pin can be sampled by configuring
 * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
 * strobes may be captured. The latched FSYNC signal will be sampled at the
 * Sampling Rate, as defined in register 25. After sampling, the latch will
 * reset to the current FSYNC signal state.
 *
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table.
 *
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 *
 * @return FSYNC configuration value
 */
uint8_t MPU9250_CLASSNAME::getExternalFrameSync()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_CONFIG, MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LENGTH, buffer);
	return buffer[0];
}
/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU9250_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void MPU9250_CLASSNAME::setExternalFrameSync(uint8_t sync)
{
	writeBits(MPU9250_RA_CONFIG, MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LENGTH, sync);
}
/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see MPU9250_RA_CONFIG
 * @see MPU9250_CFG_DLPF_CFG_BIT
 * @see MPU9250_CFG_DLPF_CFG_LENGTH
 */
uint8_t MPU9250_CLASSNAME::getDLPFMode()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LENGTH, buffer);
	return buffer[0];
}
/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU9250_DLPF_BW_256
 * @see MPU9250_RA_CONFIG
 * @see MPU9250_CFG_DLPF_CFG_BIT
 * @see MPU9250_CFG_DLPF_CFG_LENGTH
 */
void MPU9250_CLASSNAME::setDLPFMode(uint8_t mode)
{
	writeBits(MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU9250_GYRO_FS_250
 * @see MPU9250_RA_GYRO_CONFIG
 * @see MPU9250_GCONFIG_FS_SEL_BIT
 * @see MPU9250_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU9250_CLASSNAME::getFullScaleGyroRange()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, buffer);
	return buffer[0];
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU9250_GYRO_FS_250
 * @see MPU9250_RA_GYRO_CONFIG
 * @see MPU9250_GCONFIG_FS_SEL_BIT
 * @see MPU9250_GCONFIG_FS_SEL_LENGTH
 */
void MPU9250_CLASSNAME::setFullScaleGyroRange(uint8_t range)
{
	writeBits(MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, range);
}

// ACCEL_CONFIG register

/** Get self-test enabled setting for accelerometer X axis.
 * @return Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
bool MPU9250_CLASSNAME::getAccelXSelfTest()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_XA_ST_BIT, buffer);
	return buffer[0];
}
/** Get self-test enabled setting for accelerometer X axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
void MPU9250_CLASSNAME::setAccelXSelfTest(bool enabled)
{
	writeBit(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_XA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Y axis.
 * @return Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
bool MPU9250_CLASSNAME::getAccelYSelfTest()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_YA_ST_BIT, buffer);
	return buffer[0];
}
/** Get self-test enabled value for accelerometer Y axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
void MPU9250_CLASSNAME::setAccelYSelfTest(bool enabled)
{
	writeBit(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_YA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Z axis.
 * @return Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
bool MPU9250_CLASSNAME::getAccelZSelfTest()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ZA_ST_BIT, buffer);
	return buffer[0];
}
/** Set self-test enabled value for accelerometer Z axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
void MPU9250_CLASSNAME::setAccelZSelfTest(bool enabled)
{
	writeBit(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ZA_ST_BIT, enabled);
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU9250_ACCEL_FS_2
 * @see MPU9250_RA_ACCEL_CONFIG
 * @see MPU9250_ACONFIG_AFS_SEL_BIT
 * @see MPU9250_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU9250_CLASSNAME::getFullScaleAccelRange()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, buffer);
	return buffer[0];
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void MPU9250_CLASSNAME::setFullScaleAccelRange(uint8_t range)
{
	writeBits(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, range);
}
/** Get the high-pass filter configuration.
 * The DHPF is a filter module in the path leading to motion detectors (Free
 * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
 * available to the data registers (see Figure in Section 8 of the MPU-6000/
 * MPU-6050 Product Specification document).
 *
 * The high pass filter has three modes:
 *
 * <pre>
 *    Reset: The filter output settles to zero within one sample. This
 *           effectively disables the high pass filter. This mode may be toggled
 *           to quickly settle the filter.
 *
 *    On:    The high pass filter will pass signals above the cut off frequency.
 *
 *    Hold:  When triggered, the filter holds the present sample. The filter
 *           output will be the difference between the input sample and the held
 *           sample.
 * </pre>
 *
 * <pre>
 * ACCEL_HPF | Filter Mode | Cut-off Frequency
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 * </pre>
 *
 * @return Current high-pass filter configuration
 * @see MPU9250_DHPF_RESET
 * @see MPU9250_RA_ACCEL_CONFIG
 */
uint8_t MPU9250_CLASSNAME::getDHPFMode()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ACCEL_HPF_BIT, MPU9250_ACONFIG_ACCEL_HPF_LENGTH, buffer);
	return buffer[0];
}
/** Set the high-pass filter configuration.
 * @param bandwidth New high-pass filter configuration
 * @see setDHPFMode()
 * @see MPU9250_DHPF_RESET
 * @see MPU9250_RA_ACCEL_CONFIG
 */
void MPU9250_CLASSNAME::setDHPFMode(uint8_t bandwidth)
{
	writeBits(MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ACCEL_HPF_BIT, MPU9250_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}

// FF_THR register

/** Get free-fall event acceleration threshold.
 * This register configures the detection threshold for Free Fall event
 * detection. The unit of FF_THR is 1LSB = 2mg. Free Fall is detected when the
 * absolute value of the accelerometer measurements for the three axes are each
 * less than the detection threshold. This condition increments the Free Fall
 * duration counter (Register 30). The Free Fall interrupt is triggered when the
 * Free Fall duration counter reaches the time specified in FF_DUR.
 *
 * For more details on the Free Fall detection interrupt, see Section 8.2 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current free-fall acceleration threshold value (LSB = 2mg)
 * @see MPU9250_RA_FF_THR
 */
uint8_t MPU9250_CLASSNAME::getFreefallDetectionThreshold()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_FF_THR, buffer);
	return buffer[0];
}
/** Get free-fall event acceleration threshold.
 * @param threshold New free-fall acceleration threshold value (LSB = 2mg)
 * @see getFreefallDetectionThreshold()
 * @see MPU9250_RA_FF_THR
 */
void MPU9250_CLASSNAME::setFreefallDetectionThreshold(uint8_t threshold)
{
	writeByte(MPU9250_RA_FF_THR, threshold);
}

// FF_DUR register

/** Get free-fall event duration threshold.
 * This register configures the duration counter threshold for Free Fall event
 * detection. The duration counter ticks at 1kHz, therefore FF_DUR has a unit
 * of 1 LSB = 1 ms.
 *
 * The Free Fall duration counter increments while the absolute value of the
 * accelerometer measurements are each less than the detection threshold
 * (Register 29). The Free Fall interrupt is triggered when the Free Fall
 * duration counter reaches the time specified in this register.
 *
 * For more details on the Free Fall detection interrupt, see Section 8.2 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current free-fall duration threshold value (LSB = 1ms)
 * @see MPU9250_RA_FF_DUR
 */
uint8_t MPU9250_CLASSNAME::getFreefallDetectionDuration()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_FF_DUR, buffer);
	return buffer[0];
}
/** Get free-fall event duration threshold.
 * @param duration New free-fall duration threshold value (LSB = 1ms)
 * @see getFreefallDetectionDuration()
 * @see MPU9250_RA_FF_DUR
 */
void MPU9250_CLASSNAME::setFreefallDetectionDuration(uint8_t duration)
{
	writeByte(MPU9250_RA_FF_DUR, duration);
}

// MOT_THR register

/** Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
 * absolute value of any of the accelerometer measurements exceeds this Motion
 * detection threshold. This condition increments the Motion detection duration
 * counter (Register 32). The Motion detection interrupt is triggered when the
 * Motion Detection counter reaches the time count specified in MOT_DUR
 * (Register 32).
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in MOT_DETECT_STATUS (Register 97).
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU9250_RA_MOT_THR
 */
uint8_t MPU9250_CLASSNAME::getMotionDetectionThreshold()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_MOT_THR, buffer);
	return buffer[0];
}
/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU9250_RA_MOT_THR
 */
void MPU9250_CLASSNAME::setMotionDetectionThreshold(uint8_t threshold)
{
	writeByte(MPU9250_RA_MOT_THR, threshold);
}

// MOT_DUR register

/** Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
 * of 1LSB = 1ms. The Motion detection duration counter increments when the
 * absolute value of any of the accelerometer measurements exceeds the Motion
 * detection threshold (Register 31). The Motion detection interrupt is
 * triggered when the Motion detection counter reaches the time count specified
 * in this register.
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current motion detection duration threshold value (LSB = 1ms)
 * @see MPU9250_RA_MOT_DUR
 */
uint8_t MPU9250_CLASSNAME::getMotionDetectionDuration()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_MOT_DUR, buffer);
	return buffer[0];
}
/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU9250_RA_MOT_DUR
 */
void MPU9250_CLASSNAME::setMotionDetectionDuration(uint8_t duration)
{
	writeByte(MPU9250_RA_MOT_DUR, duration);
}

// ZRMOT_THR register

/** Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
 * the absolute value of the accelerometer measurements for the 3 axes are each
 * less than the detection threshold. This condition increments the Zero Motion
 * duration counter (Register 34). The Zero Motion interrupt is triggered when
 * the Zero Motion duration counter reaches the time count specified in
 * ZRMOT_DUR (Register 34).
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
 * interrupt both when Zero Motion is first detected and when Zero Motion is no
 * longer detected.
 *
 * When a zero motion event is detected, a Zero Motion Status will be indicated
 * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
 * condition is detected, the status bit is set to 1. When a zero-motion-to-
 * motion condition is detected, the status bit is set to 0.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU9250_RA_ZRMOT_THR
 */
uint8_t MPU9250_CLASSNAME::getZeroMotionDetectionThreshold()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_ZRMOT_THR, buffer);
	return buffer[0];
}
/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU9250_RA_ZRMOT_THR
 */
void MPU9250_CLASSNAME::setZeroMotionDetectionThreshold(uint8_t threshold)
{
	writeByte(MPU9250_RA_ZRMOT_THR, threshold);
}

// ZRMOT_DUR register

/** Get zero motion detection event duration threshold.
 * This register configures the duration counter threshold for Zero Motion
 * interrupt generation. The duration counter ticks at 16 Hz, therefore
 * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
 * increments while the absolute value of the accelerometer measurements are
 * each less than the detection threshold (Register 33). The Zero Motion
 * interrupt is triggered when the Zero Motion duration counter reaches the time
 * count specified in this register.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection duration threshold value (LSB = 64ms)
 * @see MPU9250_RA_ZRMOT_DUR
 */
uint8_t MPU9250_CLASSNAME::getZeroMotionDetectionDuration()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_ZRMOT_DUR, buffer);
	return buffer[0];
}
/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU9250_RA_ZRMOT_DUR
 */
void MPU9250_CLASSNAME::setZeroMotionDetectionDuration(uint8_t duration)
{
	writeByte(MPU9250_RA_ZRMOT_DUR, duration);
}

// FIFO_EN register

/** Get temperature FIFO enabled value.
 * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
 * 66) to be written into the FIFO buffer.
 * @return Current temperature FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250_CLASSNAME::getTempFIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_FIFO_EN, MPU9250_TEMP_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set temperature FIFO enabled value.
 * @param enabled New temperature FIFO enabled value
 * @see getTempFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250_CLASSNAME::setTempFIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_FIFO_EN, MPU9250_TEMP_FIFO_EN_BIT, enabled);
}
/** Get gyroscope X-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
 * 68) to be written into the FIFO buffer.
 * @return Current gyroscope X-axis FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250_CLASSNAME::getXGyroFIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_FIFO_EN, MPU9250_XG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set gyroscope X-axis FIFO enabled value.
 * @param enabled New gyroscope X-axis FIFO enabled value
 * @see getXGyroFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250_CLASSNAME::setXGyroFIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_FIFO_EN, MPU9250_XG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Y-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
 * 70) to be written into the FIFO buffer.
 * @return Current gyroscope Y-axis FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250_CLASSNAME::getYGyroFIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_FIFO_EN, MPU9250_YG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set gyroscope Y-axis FIFO enabled value.
 * @param enabled New gyroscope Y-axis FIFO enabled value
 * @see getYGyroFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250_CLASSNAME::setYGyroFIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_FIFO_EN, MPU9250_YG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Z-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
 * 72) to be written into the FIFO buffer.
 * @return Current gyroscope Z-axis FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250_CLASSNAME::getZGyroFIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_FIFO_EN, MPU9250_ZG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set gyroscope Z-axis FIFO enabled value.
 * @param enabled New gyroscope Z-axis FIFO enabled value
 * @see getZGyroFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250_CLASSNAME::setZGyroFIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_FIFO_EN, MPU9250_ZG_FIFO_EN_BIT, enabled);
}
/** Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
 * written into the FIFO buffer.
 * @return Current accelerometer FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250_CLASSNAME::getAccelFIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_FIFO_EN, MPU9250_ACCEL_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250_CLASSNAME::setAccelFIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_FIFO_EN, MPU9250_ACCEL_FIFO_EN_BIT, enabled);
}
/** Get Slave 2 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 2 to be written into the FIFO buffer.
 * @return Current Slave 2 FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250_CLASSNAME::getSlave2FIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_FIFO_EN, MPU9250_SLV2_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set Slave 2 FIFO enabled value.
 * @param enabled New Slave 2 FIFO enabled value
 * @see getSlave2FIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250_CLASSNAME::setSlave2FIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_FIFO_EN, MPU9250_SLV2_FIFO_EN_BIT, enabled);
}
/** Get Slave 1 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 1 to be written into the FIFO buffer.
 * @return Current Slave 1 FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250_CLASSNAME::getSlave1FIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_FIFO_EN, MPU9250_SLV1_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set Slave 1 FIFO enabled value.
 * @param enabled New Slave 1 FIFO enabled value
 * @see getSlave1FIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250_CLASSNAME::setSlave1FIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_FIFO_EN, MPU9250_SLV1_FIFO_EN_BIT, enabled);
}
/** Get Slave 0 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 0 to be written into the FIFO buffer.
 * @return Current Slave 0 FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250_CLASSNAME::getSlave0FIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_FIFO_EN, MPU9250_SLV0_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set Slave 0 FIFO enabled value.
 * @param enabled New Slave 0 FIFO enabled value
 * @see getSlave0FIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250_CLASSNAME::setSlave0FIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_FIFO_EN, MPU9250_SLV0_FIFO_EN_BIT, enabled);
}

// I2C_MST_CTRL register

/** Get multi-master enabled value.
 * Multi-master capability allows multiple I2C masters to operate on the same
 * bus. In circuits where multi-master capability is required, set MULT_MST_EN
 * to 1. This will increase current drawn by approximately 30uA.
 *
 * In circuits where multi-master capability is required, the state of the I2C
 * bus must always be monitored by each separate I2C Master. Before an I2C
 * Master can assume arbitration of the bus, it must first confirm that no other
 * I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
 * MPU-60X0's bus arbitration detection logic is turned on, enabling it to
 * detect when the bus is available.
 *
 * @return Current multi-master enabled value
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250_CLASSNAME::getMultiMasterEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_CTRL, MPU9250_MULT_MST_EN_BIT, buffer);
	return buffer[0];
}
/** Set multi-master enabled value.
 * @param enabled New multi-master enabled value
 * @see getMultiMasterEnabled()
 * @see MPU9250_RA_I2C_MST_CTRL
 */
void MPU9250_CLASSNAME::setMultiMasterEnabled(bool enabled)
{
	writeBit(MPU9250_RA_I2C_MST_CTRL, MPU9250_MULT_MST_EN_BIT, enabled);
}
/** Get wait-for-external-sensor-data enabled value.
 * When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
 * delayed until External Sensor data from the Slave Devices are loaded into the
 * EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
 * data (i.e. from gyro and accel) and external sensor data have been loaded to
 * their respective data registers (i.e. the data is synced) when the Data Ready
 * interrupt is triggered.
 *
 * @return Current wait-for-external-sensor-data enabled value
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250_CLASSNAME::getWaitForExternalSensorEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_CTRL, MPU9250_WAIT_FOR_ES_BIT, buffer);
	return buffer[0];
}
/** Set wait-for-external-sensor-data enabled value.
 * @param enabled New wait-for-external-sensor-data enabled value
 * @see getWaitForExternalSensorEnabled()
 * @see MPU9250_RA_I2C_MST_CTRL
 */
void MPU9250_CLASSNAME::setWaitForExternalSensorEnabled(bool enabled)
{
	writeBit(MPU9250_RA_I2C_MST_CTRL, MPU9250_WAIT_FOR_ES_BIT, enabled);
}
/** Get Slave 3 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 3 to be written into the FIFO buffer.
 * @return Current Slave 3 FIFO enabled value
 * @see MPU9250_RA_MST_CTRL
 */
bool MPU9250_CLASSNAME::getSlave3FIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_CTRL, MPU9250_SLV_3_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set Slave 3 FIFO enabled value.
 * @param enabled New Slave 3 FIFO enabled value
 * @see getSlave3FIFOEnabled()
 * @see MPU9250_RA_MST_CTRL
 */
void MPU9250_CLASSNAME::setSlave3FIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_I2C_MST_CTRL, MPU9250_SLV_3_FIFO_EN_BIT, enabled);
}
/** Get slave read/write transition enabled value.
 * The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave
 * read to the next slave read. If the bit equals 0, there will be a restart
 * between reads. If the bit equals 1, there will be a stop followed by a start
 * of the following read. When a write transaction follows a read transaction,
 * the stop followed by a start of the successive write will be always used.
 *
 * @return Current slave read/write transition enabled value
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250_CLASSNAME::getSlaveReadWriteTransitionEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_BIT, buffer);
	return buffer[0];
}
/** Set slave read/write transition enabled value.
 * @param enabled New slave read/write transition enabled value
 * @see getSlaveReadWriteTransitionEnabled()
 * @see MPU9250_RA_I2C_MST_CTRL
 */
void MPU9250_CLASSNAME::setSlaveReadWriteTransitionEnabled(bool enabled)
{
	writeBit(MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_BIT, enabled);
}
/** Get I2C master clock speed.
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 *
 * @return Current I2C master clock speed
 * @see MPU9250_RA_I2C_MST_CTRL
 */
uint8_t MPU9250_CLASSNAME::getMasterClockSpeed()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CLK_LENGTH, buffer);
	return buffer[0];
}
/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU9250_RA_I2C_MST_CTRL
 */
void MPU9250_CLASSNAME::setMasterClockSpeed(uint8_t speed)
{
	writeBits(MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CLK_LENGTH, speed);
}

// I2C_SLV* registers (Slave 0-3)

/** Get the I2C address of the specified slave (0-3).
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * In read mode, the result of the read is placed in the lowest available
 * EXT_SENS_DATA register. For further information regarding the allocation of
 * read results, please refer to the EXT_SENS_DATA register description
 * (Registers 73 - 96).
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions (getSlave4* and setSlave4*).
 *
 * I2C data transactions are performed at the Sample Rate, as defined in
 * Register 25. The user is responsible for ensuring that I2C data transactions
 * to and from each enabled Slave can be completed within a single period of the
 * Sample Rate.
 *
 * The I2C slave access rate can be reduced relative to the Sample Rate. This
 * reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
 * slave's access rate is reduced relative to the Sample Rate is determined by
 * I2C_MST_DELAY_CTRL (Register 103).
 *
 * The processing order for the slaves is fixed. The sequence followed for
 * processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a
 * particular Slave is disabled it will be skipped.
 *
 * Each slave can either be accessed at the sample rate or at a reduced sample
 * rate. In a case where some slaves are accessed at the Sample Rate and some
 * slaves are accessed at the reduced rate, the sequence of accessing the slaves
 * (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will
 * be skipped if their access rate dictates that they should not be accessed
 * during that particular cycle. For further information regarding the reduced
 * access rate, please refer to Register 52. Whether a slave is accessed at the
 * Sample Rate or at the reduced rate is determined by the Delay Enable bits in
 * Register 103.
 *
 * @param num Slave number (0-3)
 * @return Current address for specified slave
 * @see MPU9250_RA_I2C_SLV0_ADDR
 */
uint8_t MPU9250_CLASSNAME::getSlaveAddress(uint8_t num)
{
	uint8_t buffer[1];
	if (num > 3) return 0;
	readByte(MPU9250_RA_I2C_SLV0_ADDR + num * 3, buffer);
	return buffer[0];
}
/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU9250_RA_I2C_SLV0_ADDR
 */
void MPU9250_CLASSNAME::setSlaveAddress(uint8_t num, uint8_t address)
{
	if (num > 3) return;
	writeByte(MPU9250_RA_I2C_SLV0_ADDR + num * 3, address);
}
/** Get the active internal register for the specified slave (0-3).
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions.
 *
 * @param num Slave number (0-3)
 * @return Current active register for specified slave
 * @see MPU9250_RA_I2C_SLV0_REG
 */
uint8_t MPU9250_CLASSNAME::getSlaveRegister(uint8_t num)
{
	uint8_t buffer[1];
	if (num > 3) return 0;
	readByte(MPU9250_RA_I2C_SLV0_REG + num * 3, buffer);
	return buffer[0];
}
/** Set the active internal register for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param reg New active register for specified slave
 * @see getSlaveRegister()
 * @see MPU9250_RA_I2C_SLV0_REG
 */
void MPU9250_CLASSNAME::setSlaveRegister(uint8_t num, uint8_t reg)
{
	if (num > 3) return;
	writeByte(MPU9250_RA_I2C_SLV0_REG + num * 3, reg);
}
/** Get the enabled value for the specified slave (0-3).
 * When set to 1, this bit enables Slave 0 for data transfer operations. When
 * cleared to 0, this bit disables Slave 0 from data transfer operations.
 * @param num Slave number (0-3)
 * @return Current enabled value for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250_CLASSNAME::getSlaveEnabled(uint8_t num)
{
	uint8_t buffer[1];
	if (num > 3) return 0;
	readBit(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_EN_BIT, buffer);
	return buffer[0];
}
/** Set the enabled value for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New enabled value for specified slave
 * @see getSlaveEnabled()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250_CLASSNAME::setSlaveEnabled(uint8_t num, bool enabled)
{
	if (num > 3) return;
	writeBit(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_EN_BIT, enabled);
}
/** Get word pair byte-swapping enabled for the specified slave (0-3).
 * When set to 1, this bit enables byte swapping. When byte swapping is enabled,
 * the high and low bytes of a word pair are swapped. Please refer to
 * I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
 * bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
 * registers in the order they were transferred.
 *
 * @param num Slave number (0-3)
 * @return Current word pair byte-swapping enabled value for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250_CLASSNAME::getSlaveWordByteSwap(uint8_t num)
{
	uint8_t buffer[1];
	if (num > 3) return 0;
	readBit(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_BYTE_SW_BIT, buffer);
	return buffer[0];
}
/** Set word pair byte-swapping enabled for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair byte-swapping enabled value for specified slave
 * @see getSlaveWordByteSwap()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250_CLASSNAME::setSlaveWordByteSwap(uint8_t num, bool enabled)
{
	if (num > 3) return;
	writeBit(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_BYTE_SW_BIT, enabled);
}
/** Get write mode for the specified slave (0-3).
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @param num Slave number (0-3)
 * @return Current write mode for specified slave (0 = register address + data, 1 = data only)
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250_CLASSNAME::getSlaveWriteMode(uint8_t num)
{
	uint8_t buffer[1];
	if (num > 3) return 0;
	readBit(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_REG_DIS_BIT, buffer);
	return buffer[0];
}
/** Set write mode for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param mode New write mode for specified slave (0 = register address + data, 1 = data only)
 * @see getSlaveWriteMode()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250_CLASSNAME::setSlaveWriteMode(uint8_t num, bool mode)
{
	if (num > 3) return;
	writeBit(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_REG_DIS_BIT, mode);
}
/** Get word pair grouping order offset for the specified slave (0-3).
 * This sets specifies the grouping order of word pairs received from registers.
 * When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
 * then odd register addresses) are paired to form a word. When set to 1, bytes
 * from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
 * register addresses) are paired to form a word.
 *
 * @param num Slave number (0-3)
 * @return Current word pair grouping order offset for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250_CLASSNAME::getSlaveWordGroupOffset(uint8_t num)
{
	uint8_t buffer[1];
	if (num > 3) return 0;
	readBit(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_GRP_BIT, buffer);
	return buffer[0];
}
/** Set word pair grouping order offset for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair grouping order offset for specified slave
 * @see getSlaveWordGroupOffset()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250_CLASSNAME::setSlaveWordGroupOffset(uint8_t num, bool enabled)
{
	if (num > 3) return;
	writeBit(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_GRP_BIT, enabled);
}
/** Get number of bytes to read for the specified slave (0-3).
 * Specifies the number of bytes transferred to and from Slave 0. Clearing this
 * bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN.
 * @param num Slave number (0-3)
 * @return Number of bytes to read for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
uint8_t MPU9250_CLASSNAME::getSlaveDataLength(uint8_t num)
{
	uint8_t buffer[1];
	if (num > 3) return 0;
	readBits(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_LEN_BIT, MPU9250_I2C_SLV_LEN_LENGTH, buffer);
	return buffer[0];
}
/** Set number of bytes to read for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see getSlaveDataLength()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250_CLASSNAME::setSlaveDataLength(uint8_t num, uint8_t length)
{
	if (num > 3) return;
	writeBits(MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_LEN_BIT, MPU9250_I2C_SLV_LEN_LENGTH, length);
}

// I2C_SLV* registers (Slave 4)

/** Get the I2C address of Slave 4.
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * @return Current address for Slave 4
 * @see getSlaveAddress()
 * @see MPU9250_RA_I2C_SLV4_ADDR
 */
uint8_t MPU9250_CLASSNAME::getSlave4Address()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_I2C_SLV4_ADDR, buffer);
	return buffer[0];
}
/** Set the I2C address of Slave 4.
 * @param address New address for Slave 4
 * @see getSlave4Address()
 * @see MPU9250_RA_I2C_SLV4_ADDR
 */
void MPU9250_CLASSNAME::setSlave4Address(uint8_t address)
{
	writeByte(MPU9250_RA_I2C_SLV4_ADDR, address);
}
/** Get the active internal register for the Slave 4.
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * @return Current active register for Slave 4
 * @see MPU9250_RA_I2C_SLV4_REG
 */
uint8_t MPU9250_CLASSNAME::getSlave4Register()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_I2C_SLV4_REG, buffer);
	return buffer[0];
}
/** Set the active internal register for Slave 4.
 * @param reg New active register for Slave 4
 * @see getSlave4Register()
 * @see MPU9250_RA_I2C_SLV4_REG
 */
void MPU9250_CLASSNAME::setSlave4Register(uint8_t reg)
{
	writeByte(MPU9250_RA_I2C_SLV4_REG, reg);
}
/** Set new byte to write to Slave 4.
 * This register stores the data to be written into the Slave 4. If I2C_SLV4_RW
 * is set 1 (set to read), this register has no effect.
 * @param data New byte to write to Slave 4
 * @see MPU9250_RA_I2C_SLV4_DO
 */
void MPU9250_CLASSNAME::setSlave4OutputByte(uint8_t data)
{
	writeByte(MPU9250_RA_I2C_SLV4_DO, data);
}
/** Get the enabled value for the Slave 4.
 * When set to 1, this bit enables Slave 4 for data transfer operations. When
 * cleared to 0, this bit disables Slave 4 from data transfer operations.
 * @return Current enabled value for Slave 4
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250_CLASSNAME::getSlave4Enabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_EN_BIT, buffer);
	return buffer[0];
}
/** Set the enabled value for Slave 4.
 * @param enabled New enabled value for Slave 4
 * @see getSlave4Enabled()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
void MPU9250_CLASSNAME::setSlave4Enabled(bool enabled)
{
	writeBit(MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_EN_BIT, enabled);
}
/** Get the enabled value for Slave 4 transaction interrupts.
 * When set to 1, this bit enables the generation of an interrupt signal upon
 * completion of a Slave 4 transaction. When cleared to 0, this bit disables the
 * generation of an interrupt signal upon completion of a Slave 4 transaction.
 * The interrupt status can be observed in Register 54.
 *
 * @return Current enabled value for Slave 4 transaction interrupts.
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250_CLASSNAME::getSlave4InterruptEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_INT_EN_BIT, buffer);
	return buffer[0];
}
/** Set the enabled value for Slave 4 transaction interrupts.
 * @param enabled New enabled value for Slave 4 transaction interrupts.
 * @see getSlave4InterruptEnabled()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
void MPU9250_CLASSNAME::setSlave4InterruptEnabled(bool enabled)
{
	writeBit(MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_INT_EN_BIT, enabled);
}
/** Get write mode for Slave 4.
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @return Current write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250_CLASSNAME::getSlave4WriteMode()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_REG_DIS_BIT, buffer);
	return buffer[0];
}
/** Set write mode for the Slave 4.
 * @param mode New write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see getSlave4WriteMode()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
void MPU9250_CLASSNAME::setSlave4WriteMode(bool mode)
{
	writeBit(MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_REG_DIS_BIT, mode);
}
/** Get Slave 4 master delay value.
 * This configures the reduced access rate of I2C slaves relative to the Sample
 * Rate. When a slave's access rate is decreased relative to the Sample Rate,
 * the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and
 * DLPF_CFG (register 26). Whether a slave's access rate is reduced relative to
 * the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103). For
 * further information regarding the Sample Rate, please refer to register 25.
 *
 * @return Current Slave 4 master delay value
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
uint8_t MPU9250_CLASSNAME::getSlave4MasterDelay()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_MST_DLY_BIT, MPU9250_I2C_SLV4_MST_DLY_LENGTH, buffer);
	return buffer[0];
}
/** Set Slave 4 master delay value.
 * @param delay New Slave 4 master delay value
 * @see getSlave4MasterDelay()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
void MPU9250_CLASSNAME::setSlave4MasterDelay(uint8_t delay)
{
	writeBits(MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_MST_DLY_BIT, MPU9250_I2C_SLV4_MST_DLY_LENGTH, delay);
}
/** Get last available byte read from Slave 4.
 * This register stores the data read from Slave 4. This field is populated
 * after a read transaction.
 * @return Last available byte read from to Slave 4
 * @see MPU9250_RA_I2C_SLV4_DI
 */
uint8_t MPU9250_CLASSNAME::getSlate4InputByte()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_I2C_SLV4_DI, buffer);
	return buffer[0];
}

// I2C_MST_STATUS register

/** Get FSYNC interrupt status.
 * This bit reflects the status of the FSYNC interrupt from an external device
 * into the MPU-60X0. This is used as a way to pass an external interrupt
 * through the MPU-60X0 to the host application processor. When set to 1, this
 * bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
 * (Register 55).
 * @return FSYNC interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250_CLASSNAME::getPassthroughStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_PASS_THROUGH_BIT, buffer);
	return buffer[0];
}
/** Get Slave 4 transaction done status.
 * Automatically sets to 1 when a Slave 4 transaction has completed. This
 * triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
 * (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
 * I2C_SLV4_CTRL register (Register 52).
 * @return Slave 4 transaction done status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250_CLASSNAME::getSlave4IsDone()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV4_DONE_BIT, buffer);
	return buffer[0];
}
/** Get master arbitration lost status.
 * This bit automatically sets to 1 when the I2C Master has lost arbitration of
 * the auxiliary I2C bus (an error condition). This triggers an interrupt if the
 * I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Master arbitration lost status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250_CLASSNAME::getLostArbitration()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_LOST_ARB_BIT, buffer);
	return buffer[0];
}
/** Get Slave 4 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 4 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250_CLASSNAME::getSlave4Nack()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV4_NACK_BIT, buffer);
	return buffer[0];
}
/** Get Slave 3 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 3 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250_CLASSNAME::getSlave3Nack()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV3_NACK_BIT, buffer);
	return buffer[0];
}
/** Get Slave 2 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 2 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250_CLASSNAME::getSlave2Nack()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV2_NACK_BIT, buffer);
	return buffer[0];
}
/** Get Slave 1 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 1 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250_CLASSNAME::getSlave1Nack()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV1_NACK_BIT, buffer);
	return buffer[0];
}
/** Get Slave 0 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 0 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250_CLASSNAME::getSlave0Nack()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV0_NACK_BIT, buffer);
	return buffer[0];
}

// INT_PIN_CFG register

/** Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_LEVEL_BIT
 */
bool MPU9250_CLASSNAME::getInterruptMode()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_LEVEL_BIT, buffer);
	return buffer[0];
}
/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_LEVEL_BIT
 */
void MPU9250_CLASSNAME::setInterruptMode(bool mode)
{
	writeBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_LEVEL_BIT, mode);
}
/** Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 * @return Current interrupt drive mode (0=push-pull, 1=open-drain)
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_OPEN_BIT
 */
bool MPU9250_CLASSNAME::getInterruptDrive()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_OPEN_BIT, buffer);
	return buffer[0];
}
/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_OPEN_BIT
 */
void MPU9250_CLASSNAME::setInterruptDrive(bool drive)
{
	writeBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_OPEN_BIT, drive);
}
/** Get interrupt latch mode.
 * Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
 * @return Current latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_LATCH_INT_EN_BIT
 */
bool MPU9250_CLASSNAME::getInterruptLatch()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_LATCH_INT_EN_BIT, buffer);
	return buffer[0];
}
/** Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_LATCH_INT_EN_BIT
 */
void MPU9250_CLASSNAME::setInterruptLatch(bool latch)
{
	writeBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_LATCH_INT_EN_BIT, latch);
}
/** Get interrupt latch clear mode.
 * Will be set 0 for status-read-only, 1 for any-register-read.
 * @return Current latch clear mode (0=status-read-only, 1=any-register-read)
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_RD_CLEAR_BIT
 */
bool MPU9250_CLASSNAME::getInterruptLatchClear()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_RD_CLEAR_BIT, buffer);
	return buffer[0];
}
/** Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_RD_CLEAR_BIT
 */
void MPU9250_CLASSNAME::setInterruptLatchClear(bool clear)
{
	writeBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_RD_CLEAR_BIT, clear);
}
/** Get FSYNC interrupt logic level mode.
 * @return Current FSYNC interrupt mode (0=active-high, 1=active-low)
 * @see getFSyncInterruptMode()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT
 */
bool MPU9250_CLASSNAME::getFSyncInterruptLevel()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT, buffer);
	return buffer[0];
}
/** Set FSYNC interrupt logic level mode.
 * @param mode New FSYNC interrupt mode (0=active-high, 1=active-low)
 * @see getFSyncInterruptMode()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT
 */
void MPU9250_CLASSNAME::setFSyncInterruptLevel(bool level)
{
	writeBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
/** Get FSYNC pin interrupt enabled setting.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled setting
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_FSYNC_INT_EN_BIT
 */
bool MPU9250_CLASSNAME::getFSyncInterruptEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_EN_BIT, buffer);
	return buffer[0];
}
/** Set FSYNC pin interrupt enabled setting.
 * @param enabled New FSYNC pin interrupt enabled setting
 * @see getFSyncInterruptEnabled()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_FSYNC_INT_EN_BIT
 */
void MPU9250_CLASSNAME::setFSyncInterruptEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_EN_BIT, enabled);
}
/** Get I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @return Current I2C bypass enabled status
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_I2C_BYPASS_EN_BIT
 */
bool MPU9250_CLASSNAME::getI2CBypassEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, buffer);
	return buffer[0];
}
/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_I2C_BYPASS_EN_BIT
 */
void MPU9250_CLASSNAME::setI2CBypassEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/** Get reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
 * further information regarding CLKOUT, please refer to the MPU-60X0 Product
 * Specification document.
 * @return Current reference clock output enabled status
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_CLKOUT_EN_BIT
 */
bool MPU9250_CLASSNAME::getClockOutputEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_CLKOUT_EN_BIT, buffer);
	return buffer[0];
}
/** Set reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
 * further information regarding CLKOUT, please refer to the MPU-60X0 Product
 * Specification document.
 * @param enabled New reference clock output enabled status
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_CLKOUT_EN_BIT
 */
void MPU9250_CLASSNAME::setClockOutputEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_CLKOUT_EN_BIT, enabled);
}

// INT_ENABLE register

/** Get full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit will be
 * set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
uint8_t MPU9250_CLASSNAME::getIntEnabled()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_INT_ENABLE, buffer);
	return buffer[0];
}
/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
void MPU9250_CLASSNAME::setIntEnabled(uint8_t enabled)
{
	writeByte(MPU9250_RA_INT_ENABLE, enabled);
}
/** Get Free Fall interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
bool MPU9250_CLASSNAME::getIntFreefallEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FF_BIT, buffer);
	return buffer[0];
}
/** Set Free Fall interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
void MPU9250_CLASSNAME::setIntFreefallEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FF_BIT, enabled);
}
/** Get Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_MOT_BIT
 **/
bool MPU9250_CLASSNAME::getIntMotionEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_MOT_BIT, buffer);
	return buffer[0];
}
/** Set Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntMotionEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_MOT_BIT
 **/
void MPU9250_CLASSNAME::setIntMotionEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_MOT_BIT, enabled);
}
/** Get Zero Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 **/
bool MPU9250_CLASSNAME::getIntZeroMotionEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_ZMOT_BIT, buffer);
	return buffer[0];
}
/** Set Zero Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntZeroMotionEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 **/
void MPU9250_CLASSNAME::setIntZeroMotionEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_ZMOT_BIT, enabled);
}
/** Get FIFO Buffer Overflow interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
 **/
bool MPU9250_CLASSNAME::getIntFIFOBufferOverflowEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, buffer);
	return buffer[0];
}
/** Set FIFO Buffer Overflow interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFIFOBufferOverflowEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
 **/
void MPU9250_CLASSNAME::setIntFIFOBufferOverflowEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
/** Get I2C Master interrupt enabled status.
 * This enables any of the I2C Master interrupt sources to generate an
 * interrupt. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
 **/
bool MPU9250_CLASSNAME::getIntI2CMasterEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_I2C_MST_INT_BIT, buffer);
	return buffer[0];
}
/** Set I2C Master interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntI2CMasterEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
 **/
void MPU9250_CLASSNAME::setIntI2CMasterEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_I2C_MST_INT_BIT, enabled);
}
/** Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
bool MPU9250_CLASSNAME::getIntDataReadyEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, buffer);
	return buffer[0];
}
/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU9250_RA_INT_CFG
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
void MPU9250_CLASSNAME::setIntDataReadyEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, enabled);
}

// INT_STATUS register

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 */
uint8_t MPU9250_CLASSNAME::getIntStatus()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_INT_STATUS, buffer);
	return buffer[0];
}
/** Get Free Fall interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_FF_BIT
 */
bool MPU9250_CLASSNAME::getIntFreefallStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_FF_BIT, buffer);
	return buffer[0];
}
/** Get Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Motion Detection interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_MOT_BIT
 */
bool MPU9250_CLASSNAME::getIntMotionStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_MOT_BIT, buffer);
	return buffer[0];
}
/** Get Zero Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Zero Motion Detection interrupt has
 * been generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 */
bool MPU9250_CLASSNAME::getIntZeroMotionStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_ZMOT_BIT, buffer);
	return buffer[0];
}
/** Get FIFO Buffer Overflow interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
 */
bool MPU9250_CLASSNAME::getIntFIFOBufferOverflowStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, buffer);
	return buffer[0];
}
/** Get I2C Master interrupt status.
 * This bit automatically sets to 1 when an I2C Master interrupt has been
 * generated. For a list of I2C Master interrupts, please refer to Register 54.
 * The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
 */
bool MPU9250_CLASSNAME::getIntI2CMasterStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_I2C_MST_INT_BIT, buffer);
	return buffer[0];
}
/** Get Data Ready interrupt status.
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
bool MPU9250_CLASSNAME::getIntDataReadyStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_DATA_RDY_BIT, buffer);
	return buffer[0];
}

// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 16-bit signed integer container for magnetometer X-axis value
 * @param my 16-bit signed integer container for magnetometer Y-axis value
 * @param mz 16-bit signed integer container for magnetometer Z-axis value
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
void MPU9250_CLASSNAME::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz)
{
	getMotion6(ax, ay, az, gx, gy, gz);
	// TODO: magnetometer integration
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
void MPU9250_CLASSNAME::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	uint8_t buffer[14];
	readBytes(MPU9250_RA_ACCEL_XOUT_H, 14, buffer);
	*ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	*az = (((int16_t)buffer[4]) << 8) | buffer[5];
	*gx = (((int16_t)buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t)buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}
/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU9250_RA_GYRO_XOUT_H
 */
void MPU9250_CLASSNAME::getAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t buffer[6];
	readBytes(MPU9250_RA_ACCEL_XOUT_H, 6, buffer);
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[2]) << 8) | buffer[3];
	*z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
int16_t MPU9250_CLASSNAME::getAccelerationX()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_ACCEL_XOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_YOUT_H
 */
int16_t MPU9250_CLASSNAME::getAccelerationY()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_ACCEL_YOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_ZOUT_H
 */
int16_t MPU9250_CLASSNAME::getAccelerationZ()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_ACCEL_ZOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// TEMP_OUT_* registers

/** Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see MPU9250_RA_TEMP_OUT_H
 */
int16_t MPU9250_CLASSNAME::getTemperature()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_TEMP_OUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_XOUT_H
 */
void MPU9250_CLASSNAME::getRotation(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t buffer[6];
	readBytes(MPU9250_RA_GYRO_XOUT_H, 6, buffer);
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[2]) << 8) | buffer[3];
	*z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_XOUT_H
 */
int16_t MPU9250_CLASSNAME::getRotationX()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_GYRO_XOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_YOUT_H
 */
int16_t MPU9250_CLASSNAME::getRotationY()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_GYRO_YOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_ZOUT_H
 */
int16_t MPU9250_CLASSNAME::getRotationZ()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_GYRO_ZOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// EXT_SENS_DATA_* registers

/** Read single byte from external sensor data register.
 * These registers store data read from external sensors by the Slave 0, 1, 2,
 * and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
 * I2C_SLV4_DI (Register 53).
 *
 * External sensor data is written to these registers at the Sample Rate as
 * defined in Register 25. This access rate can be reduced by using the Slave
 * Delay Enable registers (Register 103).
 *
 * External sensor data registers, along with the gyroscope measurement
 * registers, accelerometer measurement registers, and temperature measurement
 * registers, are composed of two sets of registers: an internal register set
 * and a user-facing read register set.
 *
 * The data within the external sensors' internal register set is always updated
 * at the Sample Rate (or the reduced access rate) whenever the serial interface
 * is idle. This guarantees that a burst read of sensor registers will read
 * measurements from the same sampling instant. Note that if burst reads are not
 * used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Data is placed in these external sensor data registers according to
 * I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39,
 * 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from
 * an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as
 * defined in Register 25) or delayed rate (if specified in Register 52 and
 * 103). During each Sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the
 * order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * Each enabled slave will have EXT_SENS_DATA registers associated with it by
 * number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from
 * EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may
 * change the higher numbered slaves' associated registers. Furthermore, if
 * fewer total bytes are being read from the external sensors as a result of
 * such a change, then the data remaining in the registers which no longer have
 * an associated slave device (i.e. high numbered registers) will remain in
 * these previously allocated registers unless reset.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of
 * available EXT_SENS_DATA registers, the excess bytes will be dropped. There
 * are 24 EXT_SENS_DATA registers and hence the total read lengths between all
 * the slaves cannot be greater than 24 or some bytes will be lost.
 *
 * Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further
 * information regarding the characteristics of Slave 4, please refer to
 * Registers 49 to 53.
 *
 * EXAMPLE:
 * Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and
 * I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that
 * I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00
 * through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05
 * will be associated with Slave 1. If Slave 2 is enabled as well, registers
 * starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
 *
 * If Slave 2 is disabled while Slave 3 is enabled in this same situation, then
 * registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3
 * instead.
 *
 * REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE:
 * If a slave is disabled at any time, the space initially allocated to the
 * slave in the EXT_SENS_DATA register, will remain associated with that slave.
 * This is to avoid dynamic adjustment of the register allocation.
 *
 * The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all
 * slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).
 *
 * This above is also true if one of the slaves gets NACKed and stops
 * functioning.
 *
 * @param position Starting position (0-23)
 * @return Byte read from register
 */
uint8_t MPU9250_CLASSNAME::getExternalSensorByte(int position)
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_EXT_SENS_DATA_00 + position, buffer);
	return buffer[0];
}
/** Read word (2 bytes) from external sensor data registers.
 * @param position Starting position (0-21)
 * @return Word read from register
 * @see getExternalSensorByte()
 */
uint16_t MPU9250_CLASSNAME::getExternalSensorWord(int position)
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_EXT_SENS_DATA_00 + position, 2, buffer);
	return (((uint16_t)buffer[0]) << 8) | buffer[1];
}
/** Read double word (4 bytes) from external sensor data registers.
 * @param position Starting position (0-20)
 * @return Double word read from registers
 * @see getExternalSensorByte()
 */
uint32_t MPU9250_CLASSNAME::getExternalSensorDWord(int position)
{
	uint8_t buffer[4];
	readBytes(MPU9250_RA_EXT_SENS_DATA_00 + position, 4, buffer);
	return (((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) | (((uint16_t)buffer[2]) << 8) | buffer[3];
}

// MOT_DETECT_STATUS register

/** Get X-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_XNEG_BIT
 */
bool MPU9250_CLASSNAME::getXNegMotionDetected()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_XNEG_BIT, buffer);
	return buffer[0];
}
/** Get X-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_XPOS_BIT
 */
bool MPU9250_CLASSNAME::getXPosMotionDetected()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_XPOS_BIT, buffer);
	return buffer[0];
}
/** Get Y-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_YNEG_BIT
 */
bool MPU9250_CLASSNAME::getYNegMotionDetected()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_YNEG_BIT, buffer);
	return buffer[0];
}
/** Get Y-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_YPOS_BIT
 */
bool MPU9250_CLASSNAME::getYPosMotionDetected()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_YPOS_BIT, buffer);
	return buffer[0];
}
/** Get Z-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZNEG_BIT
 */
bool MPU9250_CLASSNAME::getZNegMotionDetected()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZNEG_BIT, buffer);
	return buffer[0];
}
/** Get Z-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZPOS_BIT
 */
bool MPU9250_CLASSNAME::getZPosMotionDetected()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZPOS_BIT, buffer);
	return buffer[0];
}
/** Get zero motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZRMOT_BIT
 */
bool MPU9250_CLASSNAME::getZeroMotionDetected()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZRMOT_BIT, buffer);
	return buffer[0];
}

// I2C_SLV*_DO register

/** Write byte to Data Output container for specified slave.
 * This register holds the output data written into Slave when Slave is set to
 * write mode. For further information regarding Slave control, please
 * refer to Registers 37 to 39 and immediately following.
 * @param num Slave number (0-3)
 * @param data Byte to write
 * @see MPU9250_RA_I2C_SLV0_DO
 */
void MPU9250_CLASSNAME::setSlaveOutputByte(uint8_t num, uint8_t data)
{
	if (num > 3) return;
	writeByte(MPU9250_RA_I2C_SLV0_DO + num, data);
}

// I2C_MST_DELAY_CTRL register

/** Get external data shadow delay enabled status.
 * This register is used to specify the timing of external sensor data
 * shadowing. When DELAY_ES_SHADOW is set to 1, shadowing of external
 * sensor data is delayed until all data has been received.
 * @return Current external data shadow delay enabled status.
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT
 */
bool MPU9250_CLASSNAME::getExternalShadowDelayEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_I2C_MST_DELAY_CTRL, MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT, buffer);
	return buffer[0];
}
/** Set external data shadow delay enabled status.
 * @param enabled New external data shadow delay enabled status.
 * @see getExternalShadowDelayEnabled()
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT
 */
void MPU9250_CLASSNAME::setExternalShadowDelayEnabled(bool enabled)
{
	writeBit(MPU9250_RA_I2C_MST_DELAY_CTRL, MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}
/** Get slave delay enabled status.
 * When a particular slave delay is enabled, the rate of access for the that
 * slave device is reduced. When a slave's access rate is decreased relative to
 * the Sample Rate, the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) Samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register  * 25)
 * and DLPF_CFG (register 26).
 *
 * For further information regarding I2C_MST_DLY, please refer to register 52.
 * For further information regarding the Sample Rate, please refer to register 25.
 *
 * @param num Slave number (0-4)
 * @return Current slave delay enabled status.
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
bool MPU9250_CLASSNAME::getSlaveDelayEnabled(uint8_t num)
{
	uint8_t buffer[1];
	// MPU9250_DELAYCTRL_I2C_SLV4_DLY_EN_BIT is 4, SLV3 is 3, etc.
	if (num > 4) return 0;
	readBit(MPU9250_RA_I2C_MST_DELAY_CTRL, num, buffer);
	return buffer[0];
}
/** Set slave delay enabled status.
 * @param num Slave number (0-4)
 * @param enabled New slave delay enabled status.
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
void MPU9250_CLASSNAME::setSlaveDelayEnabled(uint8_t num, bool enabled)
{
	writeBit(MPU9250_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

// SIGNAL_PATH_RESET register

/** Reset gyroscope signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_RA_SIGNAL_PATH_RESET
 * @see MPU9250_PATHRESET_GYRO_RESET_BIT
 */
void MPU9250_CLASSNAME::resetGyroscopePath()
{
	writeBit(MPU9250_RA_SIGNAL_PATH_RESET, MPU9250_PATHRESET_GYRO_RESET_BIT, true);
}
/** Reset accelerometer signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_RA_SIGNAL_PATH_RESET
 * @see MPU9250_PATHRESET_ACCEL_RESET_BIT
 */
void MPU9250_CLASSNAME::resetAccelerometerPath()
{
	writeBit(MPU9250_RA_SIGNAL_PATH_RESET, MPU9250_PATHRESET_ACCEL_RESET_BIT, true);
}
/** Reset temperature sensor signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_RA_SIGNAL_PATH_RESET
 * @see MPU9250_PATHRESET_TEMP_RESET_BIT
 */
void MPU9250_CLASSNAME::resetTemperaturePath()
{
	writeBit(MPU9250_RA_SIGNAL_PATH_RESET, MPU9250_PATHRESET_TEMP_RESET_BIT, true);
}

// MOT_DETECT_CTRL register

/** Get accelerometer power-on delay.
 * The accelerometer data path provides samples to the sensor registers, Motion
 * detection, Zero Motion detection, and Free Fall detection modules. The
 * signal path contains filters which must be flushed on wake-up with new
 * samples before the detection modules begin operations. The default wake-up
 * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
 * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
 * any value above zero unless instructed otherwise by InvenSense. Please refer
 * to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
 * further information regarding the detection modules.
 * @return Current accelerometer power-on delay
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_ACCEL_ON_DELAY_BIT
 */
uint8_t MPU9250_CLASSNAME::getAccelerometerPowerOnDelay()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_ACCEL_ON_DELAY_BIT, MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
	return buffer[0];
}
/** Set accelerometer power-on delay.
 * @param delay New accelerometer power-on delay (0-3)
 * @see getAccelerometerPowerOnDelay()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_ACCEL_ON_DELAY_BIT
 */
void MPU9250_CLASSNAME::setAccelerometerPowerOnDelay(uint8_t delay)
{
	writeBits(MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_ACCEL_ON_DELAY_BIT, MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}
/** Get Free Fall detection counter decrement configuration.
 * Detection is registered by the Free Fall detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring FF_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * FF_COUNT | Counter Decrement
 * ---------+------------------
 * 0        | Reset
 * 1        | 1
 * 2        | 2
 * 3        | 4
 * </pre>
 *
 * When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Free Fall detection,
 * please refer to Registers 29 to 32.
 *
 * @return Current decrement configuration
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_FF_COUNT_BIT
 */
uint8_t MPU9250_CLASSNAME::getFreefallDetectionCounterDecrement()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_FF_COUNT_BIT, MPU9250_DETECT_FF_COUNT_LENGTH, buffer);
	return buffer[0];
}
/** Set Free Fall detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getFreefallDetectionCounterDecrement()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_FF_COUNT_BIT
 */
void MPU9250_CLASSNAME::setFreefallDetectionCounterDecrement(uint8_t decrement)
{
	writeBits(MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_FF_COUNT_BIT, MPU9250_DETECT_FF_COUNT_LENGTH, decrement);
}
/** Get Motion detection counter decrement configuration.
 * Detection is registered by the Motion detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring MOT_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * MOT_COUNT | Counter Decrement
 * ----------+------------------
 * 0         | Reset
 * 1         | 1
 * 2         | 2
 * 3         | 4
 * </pre>
 *
 * When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Motion detection,
 * please refer to Registers 29 to 32.
 *
 */
uint8_t MPU9250_CLASSNAME::getMotionDetectionCounterDecrement()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, buffer);
	return buffer[0];
}
/** Set Motion detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getMotionDetectionCounterDecrement()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_MOT_COUNT_BIT
 */
void MPU9250_CLASSNAME::setMotionDetectionCounterDecrement(uint8_t decrement)
{
	writeBits(MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, decrement);
}

// USER_CTRL register

/** Get FIFO enabled status.
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 * @return Current FIFO enabled status
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_FIFO_EN_BIT
 */
bool MPU9250_CLASSNAME::getFIFOEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_EN_BIT, buffer);
	return buffer[0];
}
/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_FIFO_EN_BIT
 */
void MPU9250_CLASSNAME::setFIFOEnabled(bool enabled)
{
	writeBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_EN_BIT, enabled);
}
/** Get I2C Master Mode enabled status.
 * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
 * external sensor slave devices on the auxiliary I2C bus. When this bit is
 * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
 * driven by the primary I2C bus (SDA and SCL). This is a precondition to
 * enabling Bypass Mode. For further information regarding Bypass Mode, please
 * refer to Register 55.
 * @return Current I2C Master Mode enabled status
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_EN_BIT
 */
bool MPU9250_CLASSNAME::getI2CMasterModeEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, buffer);
	return buffer[0];
}
/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_EN_BIT
 */
void MPU9250_CLASSNAME::setI2CMasterModeEnabled(bool enabled)
{
	writeBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, enabled);
}
/** Switch from I2C to SPI mode (MPU-6000 only)
 * If this is set, the primary SPI interface will be enabled in place of the
 * disabled primary I2C interface.
 */
void MPU9250_CLASSNAME::switchSPIEnabled(bool enabled)
{
	writeBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_IF_DIS_BIT, enabled);
}
/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_FIFO_RESET_BIT
 */
void MPU9250_CLASSNAME::resetFIFO()
{
	writeBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_RESET_BIT, true);
}
/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_RESET_BIT
 */
void MPU9250_CLASSNAME::resetI2CMaster()
{
	writeBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_RESET_BIT, true);
}
/** Reset all sensor registers and signal paths.
 * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
 * accelerometers, and temperature sensor). This operation will also clear the
 * sensor registers. This bit automatically clears to 0 after the reset has been
 * triggered.
 *
 * When resetting only the signal path (and not the sensor registers), please
 * use Register 104, SIGNAL_PATH_RESET.
 *
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_SIG_COND_RESET_BIT
 */
void MPU9250_CLASSNAME::resetSensors()
{
	writeBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_SIG_COND_RESET_BIT, true);
}

// PWR_MGMT_1 register

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_DEVICE_RESET_BIT
 */
void MPU9250_CLASSNAME::reset()
{
	writeBit(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT, true);
	mpuDelayMs(50);
}
/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_SLEEP_BIT
 */
bool MPU9250_CLASSNAME::getSleepEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, buffer);
	return buffer[0];
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_SLEEP_BIT
 */
void MPU9250_CLASSNAME::setSleepEnabled(bool enabled)
{
	writeBit(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, enabled);
}
/** Get wake cycle enabled status.
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 * @return Current sleep mode enabled status
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CYCLE_BIT
 */
bool MPU9250_CLASSNAME::getWakeCycleEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CYCLE_BIT, buffer);
	return buffer[0];
}
/** Set wake cycle enabled status.
 * @param enabled New sleep mode enabled status
 * @see getWakeCycleEnabled()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CYCLE_BIT
 */
void MPU9250_CLASSNAME::setWakeCycleEnabled(bool enabled)
{
	writeBit(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CYCLE_BIT, enabled);
}
/** Get temperature sensor enabled status.
 * Control the usage of the internal temperature sensor.
 *
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @return Current temperature sensor enabled status
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_TEMP_DIS_BIT
 */
bool MPU9250_CLASSNAME::getTempSensorEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_TEMP_DIS_BIT, buffer);
	return buffer[0] == 0; // 1 is actually disabled here
}
/** Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param enabled New temperature sensor enabled status
 * @see getTempSensorEnabled()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_TEMP_DIS_BIT
 */
void MPU9250_CLASSNAME::setTempSensorEnabled(bool enabled)
{
	// 1 is actually disabled here
	writeBit(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_TEMP_DIS_BIT, !enabled);
}
/** Get clock source setting.
 * @return Current clock source setting
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CLKSEL_BIT
 * @see MPU9250_PWR1_CLKSEL_LENGTH
 */
uint8_t MPU9250_CLASSNAME::getClockSource()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, buffer);
	return buffer[0];
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CLKSEL_BIT
 * @see MPU9250_PWR1_CLKSEL_LENGTH
 */
void MPU9250_CLASSNAME::setClockSource(uint8_t source)
{
	writeBits(MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, source);
}

// PWR_MGMT_2 register

/** Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * <pre>
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 5 Hz
 * 3            | 10 Hz
 * <pre>
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *
 * @return Current wake frequency
 * @see MPU9250_RA_PWR_MGMT_2
 */
uint8_t MPU9250_CLASSNAME::getWakeFrequency()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_LP_WAKE_CTRL_BIT, MPU9250_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
	return buffer[0];
}
/** Set wake frequency in Accel-Only Low Power Mode.
 * @param frequency New wake frequency
 * @see MPU9250_RA_PWR_MGMT_2
 */
void MPU9250_CLASSNAME::setWakeFrequency(uint8_t frequency)
{
	writeBits(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_LP_WAKE_CTRL_BIT, MPU9250_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}

/** Get X-axis accelerometer standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XA_BIT
 */
bool MPU9250_CLASSNAME::getStandbyXAccelEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XA_BIT, buffer);
	return buffer[0];
}
/** Set X-axis accelerometer standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XA_BIT
 */
void MPU9250_CLASSNAME::setStandbyXAccelEnabled(bool enabled)
{
	writeBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XA_BIT, enabled);
}
/** Get Y-axis accelerometer standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YA_BIT
 */
bool MPU9250_CLASSNAME::getStandbyYAccelEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YA_BIT, buffer);
	return buffer[0];
}
/** Set Y-axis accelerometer standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YA_BIT
 */
void MPU9250_CLASSNAME::setStandbyYAccelEnabled(bool enabled)
{
	writeBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YA_BIT, enabled);
}
/** Get Z-axis accelerometer standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZA_BIT
 */
bool MPU9250_CLASSNAME::getStandbyZAccelEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZA_BIT, buffer);
	return buffer[0];
}
/** Set Z-axis accelerometer standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZA_BIT
 */
void MPU9250_CLASSNAME::setStandbyZAccelEnabled(bool enabled)
{
	writeBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZA_BIT, enabled);
}
/** Get X-axis gyroscope standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XG_BIT
 */
bool MPU9250_CLASSNAME::getStandbyXGyroEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XG_BIT, buffer);
	return buffer[0];
}
/** Set X-axis gyroscope standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XG_BIT
 */
void MPU9250_CLASSNAME::setStandbyXGyroEnabled(bool enabled)
{
	writeBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XG_BIT, enabled);
}
/** Get Y-axis gyroscope standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YG_BIT
 */
bool MPU9250_CLASSNAME::getStandbyYGyroEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YG_BIT, buffer);
	return buffer[0];
}
/** Set Y-axis gyroscope standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YG_BIT
 */
void MPU9250_CLASSNAME::setStandbyYGyroEnabled(bool enabled)
{
	writeBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YG_BIT, enabled);
}
/** Get Z-axis gyroscope standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZG_BIT
 */
bool MPU9250_CLASSNAME::getStandbyZGyroEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZG_BIT, buffer);
	return buffer[0];
}
/** Set Z-axis gyroscope standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZG_BIT
 */
void MPU9250_CLASSNAME::setStandbyZGyroEnabled(bool enabled)
{
	writeBit(MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZG_BIT, enabled);
}

// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU9250_CLASSNAME::getFIFOCount()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_FIFO_COUNTH, 2, buffer);
	return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
uint8_t MPU9250_CLASSNAME::getFIFOByte()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_FIFO_R_W, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::getFIFOBytes(uint8_t *data, uint8_t length)
{
	readBytes(MPU9250_RA_FIFO_R_W, length, data);
}
/** Write byte to FIFO buffer.
 * @see getFIFOByte()
 * @see MPU9250_RA_FIFO_R_W
 */
void MPU9250_CLASSNAME::setFIFOByte(uint8_t data)
{
	writeByte(MPU9250_RA_FIFO_R_W, data);
}

// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34)
 * @see MPU9250_RA_WHO_AM_I
 * @see MPU9250_WHO_AM_I_BIT
 * @see MPU9250_WHO_AM_I_LENGTH
 */
uint8_t MPU9250_CLASSNAME::getDeviceID()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_WHO_AM_I, MPU9250_WHO_AM_I_BIT, MPU9250_WHO_AM_I_LENGTH, buffer);
	return buffer[0];
}
/** Set Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @see getDeviceID()
 * @see MPU9250_RA_WHO_AM_I
 * @see MPU9250_WHO_AM_I_BIT
 * @see MPU9250_WHO_AM_I_LENGTH
 */
void MPU9250_CLASSNAME::setDeviceID(uint8_t id)
{
	writeBits(MPU9250_RA_WHO_AM_I, MPU9250_WHO_AM_I_BIT, MPU9250_WHO_AM_I_LENGTH, id);
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register

uint8_t MPU9250_CLASSNAME::getOTPBankValid()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_XG_OFFS_TC, MPU9250_TC_OTP_BNK_VLD_BIT, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setOTPBankValid(bool enabled)
{
	writeBit(MPU9250_RA_XG_OFFS_TC, MPU9250_TC_OTP_BNK_VLD_BIT, enabled);
}
int8_t MPU9250_CLASSNAME::getXGyroOffsetTC()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_XG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setXGyroOffsetTC(int8_t offset)
{
	writeBits(MPU9250_RA_XG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register

int8_t MPU9250_CLASSNAME::getYGyroOffsetTC()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_YG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setYGyroOffsetTC(int8_t offset)
{
	writeBits(MPU9250_RA_YG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register

int8_t MPU9250_CLASSNAME::getZGyroOffsetTC()
{
	uint8_t buffer[1];
	readBits(MPU9250_RA_ZG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setZGyroOffsetTC(int8_t offset)
{
	writeBits(MPU9250_RA_ZG_OFFS_TC, MPU9250_TC_OFFSET_BIT, MPU9250_TC_OFFSET_LENGTH, offset);
}

// X_FINE_GAIN register

int8_t MPU9250_CLASSNAME::getXFineGain()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_X_FINE_GAIN, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setXFineGain(int8_t gain)
{
	writeByte(MPU9250_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register

int8_t MPU9250_CLASSNAME::getYFineGain()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_Y_FINE_GAIN, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setYFineGain(int8_t gain)
{
	writeByte(MPU9250_RA_Y_FINE_GAIN, gain);
}

// Z_FINE_GAIN register

int8_t MPU9250_CLASSNAME::getZFineGain()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_Z_FINE_GAIN, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setZFineGain(int8_t gain)
{
	writeByte(MPU9250_RA_Z_FINE_GAIN, gain);
}

// XA_OFFS_* registers

int16_t MPU9250_CLASSNAME::getXAccelOffset()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_XA_OFFS_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250_CLASSNAME::setXAccelOffset(int16_t offset)
{
	writeWord(MPU9250_RA_XA_OFFS_H, offset);
}

// YA_OFFS_* register

int16_t MPU9250_CLASSNAME::getYAccelOffset()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_YA_OFFS_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250_CLASSNAME::setYAccelOffset(int16_t offset)
{
	writeWord(MPU9250_RA_YA_OFFS_H, offset);
}

// ZA_OFFS_* register

int16_t MPU9250_CLASSNAME::getZAccelOffset()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_ZA_OFFS_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250_CLASSNAME::setZAccelOffset(int16_t offset)
{
	writeWord(MPU9250_RA_ZA_OFFS_H, offset);
}

// XG_OFFS_USR* registers

int16_t MPU9250_CLASSNAME::getXGyroOffset()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_XG_OFFS_USRH, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250_CLASSNAME::setXGyroOffset(int16_t offset)
{
	writeWord(MPU9250_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register

int16_t MPU9250_CLASSNAME::getYGyroOffset()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_YG_OFFS_USRH, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250_CLASSNAME::setYGyroOffset(int16_t offset)
{
	writeWord(MPU9250_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register

int16_t MPU9250_CLASSNAME::getZGyroOffset()
{
	uint8_t buffer[2];
	readBytes(MPU9250_RA_ZG_OFFS_USRH, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU9250_CLASSNAME::setZGyroOffset(int16_t offset)
{
	writeWord(MPU9250_RA_ZG_OFFS_USRH, offset);
}

// INT_ENABLE register (DMP functions)

bool MPU9250_CLASSNAME::getIntPLLReadyEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_PLL_RDY_INT_BIT, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setIntPLLReadyEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
bool MPU9250_CLASSNAME::getIntDMPEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DMP_INT_BIT, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setIntDMPEnabled(bool enabled)
{
	writeBit(MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DMP_INT_BIT, enabled);
}

// DMP_INT_STATUS

bool MPU9250_CLASSNAME::getDMPInt5Status()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_5_BIT, buffer);
	return buffer[0];
}
bool MPU9250_CLASSNAME::getDMPInt4Status()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_4_BIT, buffer);
	return buffer[0];
}
bool MPU9250_CLASSNAME::getDMPInt3Status()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_3_BIT, buffer);
	return buffer[0];
}
bool MPU9250_CLASSNAME::getDMPInt2Status()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_2_BIT, buffer);
	return buffer[0];
}
bool MPU9250_CLASSNAME::getDMPInt1Status()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_1_BIT, buffer);
	return buffer[0];
}
bool MPU9250_CLASSNAME::getDMPInt0Status()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_DMP_INT_STATUS, MPU9250_DMPINT_0_BIT, buffer);
	return buffer[0];
}

// INT_STATUS register (DMP functions)

bool MPU9250_CLASSNAME::getIntPLLReadyStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_PLL_RDY_INT_BIT, buffer);
	return buffer[0];
}
bool MPU9250_CLASSNAME::getIntDMPStatus()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_DMP_INT_BIT, buffer);
	return buffer[0];
}

// USER_CTRL register (DMP functions)

bool MPU9250_CLASSNAME::getDMPEnabled()
{
	uint8_t buffer[1];
	readBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_DMP_EN_BIT, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setDMPEnabled(bool enabled)
{
	writeBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_DMP_EN_BIT, enabled);
}
void MPU9250_CLASSNAME::resetDMP()
{
	writeBit(MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_DMP_RESET_BIT, true);
}

// BANK_SEL register

void MPU9250_CLASSNAME::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank)
{
	bank &= 0x1F;
	if (userBank) bank |= 0x20;
	if (prefetchEnabled) bank |= 0x40;
	writeByte(MPU9250_RA_BANK_SEL, bank);
}

// MEM_START_ADDR register

void MPU9250_CLASSNAME::setMemoryStartAddress(uint8_t address)
{
	writeByte(MPU9250_RA_MEM_START_ADDR, address);
}

// MEM_R_W register

uint8_t MPU9250_CLASSNAME::readMemoryByte()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_MEM_R_W, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::writeMemoryByte(uint8_t data)
{
	writeByte(MPU9250_RA_MEM_R_W, data);
}
void MPU9250_CLASSNAME::readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
	setMemoryBank(bank);
	setMemoryStartAddress(address);
	uint8_t chunkSize;
	for (uint16_t i = 0; i < dataSize;)
	{
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU9250_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize) chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address) chunkSize = 256 - address;

		// read the chunk of data as specified
		readBytes(MPU9250_RA_MEM_R_W, chunkSize, data + i);

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize)
		{
			if (address == 0) bank++;
			setMemoryBank(bank);
			setMemoryStartAddress(address);
		}
	}
}
bool MPU9250_CLASSNAME::writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem)
{
	setMemoryBank(bank);
	setMemoryStartAddress(address);
	uint8_t chunkSize;
	uint8_t verifyBuffer[MPU9250_DMP_MEMORY_CHUNK_SIZE];
	uint8_t *progBuffer;
	uint16_t i;
	uint8_t j;
	for (i = 0; i < dataSize;)
	{
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU9250_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize) chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address) chunkSize = 256 - address;

		progBuffer = (uint8_t *)data + i;

		writeBytes(MPU9250_RA_MEM_R_W, chunkSize, progBuffer);

		// verify data if needed
		if (verify && verifyBuffer)
		{
			setMemoryBank(bank);
			setMemoryStartAddress(address);
			readBytes(MPU9250_RA_MEM_R_W, chunkSize, verifyBuffer);
			if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0)
			{
				/*Serial.print("Block write verification error, bank ");
				Serial.print(bank, DEC);
				Serial.print(", address ");
				Serial.print(address, DEC);
				Serial.print("!\nExpected:");
				for (j = 0; j < chunkSize; j++) {
				    Serial.print(" 0x");
				    if (progBuffer[j] < 16) Serial.print("0");
				    Serial.print(progBuffer[j], HEX);
				}
				Serial.print("\nReceived:");
				for (uint8_t j = 0; j < chunkSize; j++) {
				    Serial.print(" 0x");
				    if (verifyBuffer[i + j] < 16) Serial.print("0");
				    Serial.print(verifyBuffer[i + j], HEX);
				}
				Serial.print("\n");*/
				return false; // uh oh.
			}
		}

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize)
		{
			if (address == 0) bank++;
			setMemoryBank(bank);
			setMemoryStartAddress(address);
		}
	}
	return true;
}
bool MPU9250_CLASSNAME::writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify)
{
	return writeMemoryBlock(data, dataSize, bank, address, verify, true);
}
bool MPU9250_CLASSNAME::writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem)
{
	uint8_t *progBuffer, success, special;
	uint16_t i, j;

	// config set data is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	uint8_t bank, offset, length;
	for (i = 0; i < dataSize;)
	{
		bank = data[i++];
		offset = data[i++];
		length = data[i++];

		// write data or perform special action
		if (length > 0)
		{
			// regular block of data to write
			/*Serial.print("Writing config block to bank ");
			Serial.print(bank);
			Serial.print(", offset ");
			Serial.print(offset);
			Serial.print(", length=");
			Serial.println(length);*/
			progBuffer = (uint8_t *)data + i;
			success = writeMemoryBlock(progBuffer, length, bank, offset, true);
			i += length;
		}
		else
		{
			// special instruction
			// NOTE: this kind of behavior (what and when to do certain things)
			// is totally undocumented. This code is in here based on observed
			// behavior only, and exactly why (or even whether) it has to be here
			// is anybody's guess for now.
			special = data[i++];
			/*Serial.print("Special command code ");
			Serial.print(special, HEX);
			Serial.println(" found...");*/
			if (special == 0x01)
			{
				// enable DMP-related interrupts

				//setIntZeroMotionEnabled(true);
				//setIntFIFOBufferOverflowEnabled(true);
				//setIntDMPEnabled(true);
				writeByte(MPU9250_RA_INT_ENABLE, 0x32);  // single operation

				success = true;
			}
			else
			{
				// unknown special command
				success = false;
			}
		}

		if (!success)
		{
			return false; // uh oh
		}
	}
	return true;
}
bool MPU9250_CLASSNAME::writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize)
{
	return writeDMPConfigurationSet(data, dataSize, true);
}

// DMP_CFG_1 register

uint8_t MPU9250_CLASSNAME::getDMPConfig1()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_DMP_CFG_1, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setDMPConfig1(uint8_t config)
{
	writeByte(MPU9250_RA_DMP_CFG_1, config);
}

// DMP_CFG_2 register

uint8_t MPU9250_CLASSNAME::getDMPConfig2()
{
	uint8_t buffer[1];
	readByte(MPU9250_RA_DMP_CFG_2, buffer);
	return buffer[0];
}
void MPU9250_CLASSNAME::setDMPConfig2(uint8_t config)
{
	writeByte(MPU9250_RA_DMP_CFG_2, config);
}

uint16_t fifoCount;
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
float last_x_angle;  // These are the filtered angles
float last_y_angle;
float last_z_angle;
uint8_t fifoBuffer[64]; // FIFO storage buffer

// bool MPU9250_CLASSNAME::readmpu()
// {
// bool rd = 0;
// fifoCount = getFIFOCount();
// while (fifoCount >= packetSize)
// {
// getFIFOBytes(fifoBuffer, packetSize);
// fifoCount -= packetSize;
// rd = 1;
// }

// // dmpGetQuaternion(&q, fifoBuffer);
// // dmpGetGravity(&gravity, &q);
// // dmpGetYawPitchRoll(ypr, &q, &gravity);

// // last_x_angle = ypr[2] * 180 / M_PI;
// // last_y_angle = ypr[1] * 180 / M_PI;
// // last_z_angle = ypr[0] * 180 / M_PI;

// return rd;
// }

#define E(x) x

uint8_t MPU9250_CLASSNAME::mpuWriteSlaveReg(uint8_t addr, uint8_t reg, uint8_t val)
{
	setSlave4Address(0x00 | addr);
	setSlave4Register(reg);
	setSlave4OutputByte(val);
	// setSlave4MasterDelay(1);
	setSlave4Enabled(true);
	int i;
	mpuDelayMs(2);
	// for (i = 0; i < 100; i++)
	// {
	// if (getSlave4IsDone())
	// return 0;
	// DEBUG_PRINTLN("wait");
	// mpuDelayMs(10);
	// }
	// DEBUG_PRINTLN("timeout");
	return 1;
}

uint8_t MPU9250_CLASSNAME::mpuReadSlaveReg(uint8_t addr, uint8_t reg, uint8_t& val)
{
	setSlave4Address(0x80 | addr);
	setSlave4Register(reg);
	setSlave4OutputByte(val);
	setSlave4MasterDelay(1);
	setSlave4Enabled(true);
	mpuDelayMs(4);

	val = getSlate4InputByte();

	return 0;
	int i;
	for (i = 0; i < 100; i++)
	{
		if (getSlave4IsDone())
		{
			return 0;
		}
		DEBUG_PRINTLN("wait");
		mpuDelayMs(10);
	}
	DEBUG_PRINTLN("timeout");
	return 1;
}

// int mymain()
// {
// uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
// bool dmpReady = false; // set true if DMP init was successful

// initialize();
// printf("2\r\n");

// uint8_t count, b;
// count = readByte(MPU9250_RA_WHO_AM_I, &b);
// printf("RA_WHO_AM_I %i", b);


// // verify connection
// printf("DeviceId: %i\r\n", getDeviceID());
// if (testConnection())
// {
// printf("3\r\n");
// }

// // uint8_t mpuEnableMag()
// // {

// // for (;;)
// // {
// // for (int i = 0; i < 3; i++)
// // {
// // int8_t t1 = getExternalSensorByte(i * 2 + 0);
// // int8_t t2 = getExternalSensorByte(i * 2 + 1);
// // int16_t t = (t1 << 8) | t2;
// // DEBUG_PRINTLN("%d = %d", i, t);
// // }
// // mpuDelayMs(20);
// // }
// // }
// //

// // load and configure the DMP
// devStatus = dmpInitialize();
// printf("4: %i\r\n", devStatus);

// resetI2CMaster();
// setI2CMasterModeEnabled(true);

// mpuWriteSlaveReg(MPU9250_MAG_ADDRESS, MPU9250_MAG_RSV, 0x01);
// mpuWriteSlaveReg(MPU9250_MAG_ADDRESS, MPU9250_MAG_CNTL, 0b00010110);

// resetI2CMaster();
// setI2CMasterModeEnabled(true);

// // setup read
// setSlaveAddress(0, 0b10000000 | 0x0c);
// setSlaveRegister(0, 0x02);
// setSlaveWordGroupOffset(0, true);
// setSlaveWordByteSwap(0, true);
// setSlaveDataLength(0, 8);
// setSlaveEnabled(0, true);

// // for (;;)
// // {
// // uint8_t ta;
// // mpuReadSlaveReg(MPU9250_MAG_ADDRESS, MPU9250_MAG_ST1, ta);
// // for (int i = 0; i < 3; i++)
// // {
// // uint8_t t1 = getExternalSensorByte(1 + i * 2 + 0);
// // uint8_t t2 = getExternalSensorByte(1 + i * 2 + 1);
// // int16_t t = ((int16_t)t2 << 8) | (uint8_t)t1;

// // t1 = t2 = 0;
// // mpuReadSlaveReg(MPU9250_MAG_ADDRESS, MPU9250_MAG_XOUT_L + i * 2 + 0, t1);
// // mpuReadSlaveReg(MPU9250_MAG_ADDRESS, MPU9250_MAG_XOUT_L + i * 2 + 1, t2);
// // int16_t t2t = ((int16_t)t2 << 8) | (uint8_t)t1;
// // DEBUG_PRINTLN("%d = %6d %6d", i, t, t2t);
// // }
// // mpuReadSlaveReg(MPU9250_MAG_ADDRESS, MPU9250_MAG_ST2, ta);
// // mpuDelayMs(100);
// // printf("\r\n");
// // }

// // supply your own gyro offsets here, scaled for min sensitivity
// /*    setXAccelOffset(-7550); // -522 -382 -475 -7482 (if drifting to the right, subtract)
// setYAccelOffset(6212); // 1131 1175
// setZAccelOffset(8157); // 1273 1289
// setXGyroOffset(79); // 44 42
// setYGyroOffset(-26); // -28 -21
// setZGyroOffset(10); // 17 :30:*/

// // make sure it worked (returns 0 if so)

// if (devStatus == 0)
// {
// // turn on the DMP, now that it's ready
// //Serial.println("Enabling DMP...");
// setDMPEnabled(true);

// // enable Arduino interrupt detection
// //Serial.println("Enabling interrupt detection (Arduino external interrupt 0)...");
// //attachInterrupt(0, dmpDataReady, RISING);
// //mpuIntStatus = mpu.getIntStatus();

// // set our DMP Ready flag so the main loop() function knows it's okay to use it
// //Serial.println("DMP ready! Waiting for first interrupt...");
// dmpReady = true;

// // get expected DMP packet size for later comparison
// packetSize = dmpGetFIFOPacketSize();
// }
// else
// {
// // ERROR!
// // 1 = initial memory load failed
// // 2 = DMP configuration updates failed
// // (if it's going to break, usually the code will be 1)
// //Serial.print("DMP Initialization failed (code ");
// //Serial.print(devStatus);
// //Serial.println(")");
// }
// unsigned long long lastdisplayupdate = 0;
// while (true)
// {
// if (readmpu())
// // if ((ticks - lastdisplayupdate) > 100)
// {
// // lastdisplayupdate = ticks;

// // printf("X: %2f ", last_x_angle);
// // printf("Y: %2f ", last_y_angle);
// // printf("Z: %2f\r\n", last_z_angle);

// int16_t ac[3], gy[3], mg[3];
// dmpGetAccel(ac, fifoBuffer);
// dmpGetGyro(gy, fifoBuffer);
// // dmpGetMag(mg, fifoBuffer);

// // for (int i = 0; i < 48; i++)
// // {
// // printf("0x%02x ", fifoBuffer[i]);
// // if (i == 15)
// // printf("\r\n");
// // if (i == 27)
// // printf("\r\n");
// // if (i == 33)
// // printf("\r\n");
// // if (i == 45)
// // printf("\r\n");
// // }
// // printf("\r\n");

// // printf("ax %d ay %d az %d\r\n", ac[0], ac[1], ac[2]);
// // printf("gx %d gy %d gz %d\r\n", gy[0], gy[1], gy[2]);
// // printf("mx %d my %d mz %d\r\n", mg[0], mg[1], mg[2]);

// Quaternion q;
// dmpGetQuaternion(&q, fifoBuffer);
// // for (int i = 0; i < 3; i++)
// int16_t mx = getExternalSensorWord(1 + 0 * 2);
// int16_t my = getExternalSensorWord(1 + 1 * 2);
// int16_t mz = getExternalSensorWord(1 + 2 * 2);
// // DEBUG_PRINTLN("%d = %8d", i, t1);
// }
// // mpuDelayMs(10);
// }

// }
