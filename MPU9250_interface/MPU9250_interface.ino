/*
 MPU9250 IMU chip test

 created dec 10 2014
 by Jan Bolting
 */
#include "SPI.h"
#include "MPU9250.h"
#include "MPU9250_mbed.h"
#include "mavlink.h"
#include "Timer.h"
#include <math.h>

const bool textOutputOn = true;
const bool binaryOutputOn = false;
const int ledPin =  13;
Timer t;

void setup()
{
  Serial1.begin(460800);
  Serial1.println("MPU9250 test...");

  SPI.begin(4);
  // Set SPI clock speed to 20 MHz:
  SPI.setClockDivider(4, 21);
  // Configure chip:
  setClockSource(0);
  setFullScaleGyroRange(MPU9250_GYRO_FULL_SCALE_250DPS);
  setFullScaleAccelRange(MPU9250_FULL_SCALE_4G);
  setSleepEnabled(false);

  // Configure auxiliary I2C to read magnetometer data:
  const uint8_t MPU_InitRegNum = 17;

  uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
    {0x80, MPUREG_PWR_MGMT_1},     // Reset Device
    {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
    {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
    //{low_pass_filter, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
    {0x18, MPUREG_GYRO_CONFIG},    // +-2000dps
    {0x08, MPUREG_ACCEL_CONFIG},   // +-4G
    {0x09, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
    {0x30, MPUREG_INT_PIN_CFG},    //
    //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
    //{0x20, MPUREG_USER_CTRL},      // Enable AUX
    {0x20, MPUREG_USER_CTRL},       // I2C Master mode
    {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz

    {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
    //{0x09, MPUREG_I2C_SLV4_CTRL},
    //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

    {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
    {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
    {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

    {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
    {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
    {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

  };

  for (uint8_t i = 0; i < MPU_InitRegNum; i++) {
    writeRegister(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
    delay(1);  //I2C must slow down the write speed, otherwise it won't work
  }
  // Set up tasks:
  t.every(200, toggleLED);
  t.every(1, doStuff);
  // t.every(200, sendHeartbeat);
  
  // Configure LED pin:
  pinMode(ledPin, OUTPUT);
}

void loop() {
  t.update();
}

void doStuff() {
  float acc[3];
  readAcc(acc);
  //delayMicroseconds(100);
  float gyr[3];
  //readGyr(gyr);
  float temp = 0;
  //float temp = readTemp();
  //unsigned int magID = AK8963_whoami();
  float mag[3];
  //readMag(mag);

  if (textOutputOn) {
    /*
    Serial1.print("Reading accelerometer z... ");
    Serial1.println(acc[2]);
    Serial1.print("Reading gyrometer z... ");
    Serial1.println(gyr[2]);
    Serial1.print("Reading chip temperature... ");
    Serial1.println(temp);
    Serial1.print("Reading magnetometer ID... ");
    Serial1.println(magID);
    Serial1.print("Reading magnetometer z value... ");
    Serial1.println(mag[2]);
    */
    Serial1.print(micros());
    Serial1.print(",");
    Serial1.print(acc[0]);
    Serial1.print(",");
    Serial1.print(acc[1]);
    Serial1.print(",");
    Serial1.print(acc[2]);
    Serial1.print(",");
    Serial1.print(gyr[0]);
    Serial1.print(",");
    Serial1.print(gyr[1]);
    Serial1.print(",");
    Serial1.print(gyr[2]);
    Serial1.print(",");
    Serial1.println(temp);
  }
  if (binaryOutputOn) {
        // Define the system type, in this case an airplane
        int system_type = MAV_TYPE_FIXED_WING;
        int autopilot_type = MAV_AUTOPILOT_GENERIC;
    
        // Initialize the required buffers
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        // Pack the message
        mavlink_msg_scaled_imu_pack(100, 
                                             200, 
                                             &msg,
                                             millis(),
                                             acc[0] * (1000/9.81),
                                             acc[1] * (1000/9.81),
                                             acc[2] * (1000/9.81),
                                             gyr[0] * ((M_PI/180)*1000),
                                             gyr[1] * ((M_PI/180)*1000),
                                             gyr[2] * ((M_PI/180)*1000),
                                             mag[0] / 1000,
                                             mag[1] / 1000,
                                             mag[2] / 1000);
        
        // Copy the message to the send buffer
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        // Send the message
        Serial1.write(buf, len);
  }
}

void sendHeartbeat() {
        // Define the system type, in this case an airplane
        int system_type = MAV_TYPE_FIXED_WING;
        int autopilot_type = MAV_AUTOPILOT_GENERIC;
    
        // Initialize the required buffers
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        // Pack the message
        // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
        mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type, MAV_MODE_FLAG_TEST_ENABLED, 0, MAV_STATE_UNINIT);
        // Copy the message to the send buffer
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        // Send the message
        Serial1.write(buf, len);        
}


void toggleLED() {
  static int ledState = LOW;             // ledState used to set the LED
  // if the LED is off turn it on and vice-versa:
  if (ledState == LOW) {
    ledState = HIGH;
  }
  else {
    ledState = LOW;
  }
  digitalWrite(ledPin, ledState);
}

void setClockSource(const uint8_t source) {
  if (source <= 7) {
    writeMaskedRegister(MPU9250_PWR_MGMT_1, MPU9250_CLKSEL_MASK, source);
  }
}

void setFullScaleGyroRange(const uint8_t range) {
  writeMaskedRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_MASK, range); //MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, range);
}

void setFullScaleAccelRange(const uint8_t range) {
  writeMaskedRegister(MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_FS_SEL_MASK, range);
}

void setSleepEnabled(const bool enabled) {
  writeMaskedRegister(MPU9250_PWR_MGMT_1, MPU9250_SLEEP_MASK, enabled);
}

/*
  Returns the reading of the accelerometers in [m/^2].
*/
void readAcc(float * acc) {
  float invsensitivity = 32768 / (9.81 * 4);
  uint8_t msb = readRegVal(MPU9250_ACCEL_XOUT_H);
  uint8_t lsb = readRegVal(MPU9250_ACCEL_XOUT_L);
  acc[0] = lsbmsb2float(msb, lsb) / invsensitivity;
  msb = readRegVal(MPU9250_ACCEL_YOUT_H);
  lsb = readRegVal(MPU9250_ACCEL_YOUT_L);
  acc[1] = lsbmsb2float(msb, lsb) / invsensitivity;
  msb = readRegVal(MPU9250_ACCEL_ZOUT_H);
  lsb = readRegVal(MPU9250_ACCEL_ZOUT_L);
  acc[2] = lsbmsb2float(msb, lsb) / invsensitivity;
}

/*
  Returns the readings of the gyroscopes in [deg/s].
*/
void readGyr(float * gyr) {
  float invsensitivity = 32768 / 250;
  uint8_t msb = readRegVal(MPU9250_GYRO_XOUT_H);
  uint8_t lsb = readRegVal(MPU9250_GYRO_XOUT_L);
  gyr[0] =  lsbmsb2float(msb, lsb) / invsensitivity;
  msb = readRegVal(MPU9250_GYRO_YOUT_H);
  lsb = readRegVal(MPU9250_GYRO_YOUT_L);
  gyr[1] = lsbmsb2float(msb, lsb) / invsensitivity;
  msb = readRegVal(MPU9250_GYRO_ZOUT_H);
  lsb = readRegVal(MPU9250_GYRO_ZOUT_L);
  gyr[2] = lsbmsb2float(msb, lsb) / invsensitivity;
}

/*
  Returns the reading of the internal temperature sensor in [degrees Celsius].
*/
float readTemp() {
  uint8_t msb = readRegVal(MPU9250_TEMP_OUT_H);
  uint8_t lsb = readRegVal(MPU9250_TEMP_OUT_L);
  const float invsensitivity = 333.87;
  return (lsbmsb2float(msb, lsb) / invsensitivity) + 21.0;
}

unsigned int AK8963_whoami() {
  unsigned int response;
  writeRegister(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
  writeRegister(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
  writeRegister(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

  //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
  delay(1);
  response = readRegVal(MPUREG_EXT_SENS_DATA_00);    //Read I2C
  return response;
}

/*
  Returns the readings of the on-chip magnetometer in [mT].
*/
void readMag(float * mag) {
  float invsensitivity = 32768 / 4800;;
  uint8_t response[7];
  int16_t bit_data;
  float data;
  int i;

  writeRegister(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
  writeRegister(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
  writeRegister(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer

  delay(1);
  // Must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
  readRegs(MPUREG_EXT_SENS_DATA_00, response, 7);
  for (i = 0; i < 3; i++) {
    bit_data = ((int16_t)response[i * 2 + 1] << 8) | response[i * 2];
    data = (float)bit_data;
    mag[i] = data / invsensitivity;
  }

}

float lsbmsb2float(uint8_t msb, uint8_t lsb) {
  uint16_t utmp = ((uint16_t) msb) << 8;
  int16_t stmp = (int16_t)(utmp | ((uint16_t) lsb));
  return (float) stmp;
}

uint8_t readRegVal(uint8_t cmd)
{
  transmitSPIcontinue(cmd | 0x80);
  return transmitSPI(0);
}

void readRegs( uint8_t ReadAddr, uint8_t * ReadBuf, unsigned int nBytes )
{
  unsigned int  i = 0;
  transmitSPIcontinue(ReadAddr | 0x80);
  for (i = 0; i < nBytes - 1; i++) {
    ReadBuf[i] = transmitSPIcontinue(0x00);
  }
  ReadBuf[nBytes - 1] = transmitSPI(0x00);
}

void writeMaskedRegister(const uint8_t register_addr, const uint8_t mask, const uint8_t value) {
  uint8_t masked_value = (mask & value);
  writeRegister(register_addr, masked_value);
  // Better way to do this: read register, set relevant bits, write register
}

void writeRegister(const uint8_t register_addr, const uint8_t value) {
  uint8_t tmp;
  tmp = SPI.transfer(4, register_addr, SPI_CONTINUE);
  tmp = SPI.transfer(4, value, SPI_LAST);
}

uint8_t transmitSPIcontinue(uint8_t val)
{
  uint8_t  tmp;
  noInterrupts();
  tmp = SPI.transfer(4, val, SPI_CONTINUE);
  interrupts();
  return tmp;
}

uint8_t transmitSPI(uint8_t val)
{
  uint8_t  tmp;
  noInterrupts();
  tmp = SPI.transfer(4, val, SPI_LAST);
  interrupts();
  return tmp;
}
