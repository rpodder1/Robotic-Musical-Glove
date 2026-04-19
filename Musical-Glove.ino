#include <Wire.h>
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32.h>
#include <MIDI.h>


BLEMIDI_CREATE_INSTANCE("E-nstrument", MIDI);


// MPU6050
const uint8_t MPU6050SlaveAddress       = 0x68;
const uint8_t scl                       = 1;
const uint8_t sda                       = 2;
const uint16_t AccelScaleFactor         = 16384;


const uint8_t MPU6050_REGISTER_SMPLRT_DIV        = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL         = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1        = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2        = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG            = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG       = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG      = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN           = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE        = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H      = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;


int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;


// Touch
const int touchPins[4]    = {7, 9, 11, 12};
int baseline[4]           = {0, 0, 0, 0};
float smoothedTouch[4]    = {0, 0, 0, 0};
const float touchAlpha    = 0.5;
const int TOUCH_THRESHOLD = 30000;


// IMU filter
const float alpha = 0.1;
float roll = 0, pitch = 0;
float smoothedAY = 0;
const float ayAlpha = 0.2;


// Octave
int octaveOffset = 0;
const float FLICK_THRESHOLD = 3.0;
const int FLICK_COOLDOWN = 1000;
unsigned long lastFlickTime = 0;


// Notes
const int notesets[3][4] = {
 {57, 58, 59, 60},
 {61, 62, 63, 64},
 {65, 66, 67, 68}
};


bool wasPressed[4] = {false, false, false, false};
bool isConnected   = false;
int lastMidi[4]    = {-1, -1, -1, -1};


// IMU helpers
float calcRoll(float ax, float ay, float az) {
 return atan2(ay, az) * 180.0 / PI;
}
float calcPitch(float ax, float ay, float az) {
 return atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
}
int getRollSet(float r) {
 if (r > -90 && r < 60)  return 0;
 if (r < -90 || r > 150) return 2;
 return 1;
}
int classifyIndex(float delta) {
 return (delta > TOUCH_THRESHOLD) ? 0 : -1;
}


// I2C / MPU6050
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
 Wire.beginTransmission(deviceAddress);
 Wire.write(regAddress);
 Wire.write(data);
 Wire.endTransmission();
}


void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
 Wire.beginTransmission(deviceAddress);
 Wire.write(regAddress);
 Wire.endTransmission();
 Wire.requestFrom(deviceAddress, (uint8_t)14);
 AccelX      = (((int16_t)Wire.read() << 8) | Wire.read());
 AccelY      = (((int16_t)Wire.read() << 8) | Wire.read());
 AccelZ      = (((int16_t)Wire.read() << 8) | Wire.read());
 Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
 GyroX       = (((int16_t)Wire.read() << 8) | Wire.read());
 GyroY       = (((int16_t)Wire.read() << 8) | Wire.read());
 GyroZ       = (((int16_t)Wire.read() << 8) | Wire.read());
}


void MPU6050_Init() {
 delay(150);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}


void calibrate() {
 for (int i = 0; i < 4; i++) {
   long sum = 0;
   for (int j = 0; j < 50; j++) {
     sum += touchRead(touchPins[i]);
     delay(10);
   }
   baseline[i] = sum / 50;
   smoothedTouch[i] = baseline[i];
 }
}


void setup() {
 Serial.begin(115200);


 MIDI.begin();


 BLEMIDI.setHandleConnected([]() {
   isConnected = true;
   Serial.println("BLE MIDI Connected!");
 });


 BLEMIDI.setHandleDisconnected([]() {
   isConnected = false;
   Serial.println("BLE MIDI Disconnected.");
 });


 Wire.begin(sda, scl);
 MPU6050_Init();


 Serial.println("Take glove OFF. Calibrating in 5 seconds...");
 delay(5000);
 calibrate();
 Serial.println("Done. Connect via Bluetooth then put glove on.");
}


void loop() {
 MIDI.read();


 if (!isConnected) return;


 Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);


 float ax = (float)AccelX / AccelScaleFactor;
 float ay = (float)AccelY / AccelScaleFactor;
 float az = (float)AccelZ / AccelScaleFactor;


 smoothedAY = ayAlpha * ay + (1 - ayAlpha) * smoothedAY;


 float rawRoll = calcRoll(ax, ay, az);
 roll  = alpha * rawRoll + (1 - alpha) * roll;
 pitch = alpha * calcPitch(ax, ay, az) + (1 - alpha) * pitch;


 float deltas[4];
 for (int i = 0; i < 4; i++) {
   int raw = touchRead(touchPins[i]);
   smoothedTouch[i] = touchAlpha * raw + (1 - touchAlpha) * smoothedTouch[i];
   deltas[i] = smoothedTouch[i] - baseline[i];
 }


 int set = getRollSet(rawRoll);


 // Octave change — index finger held + strong flick
 bool indexPressed = classifyIndex(deltas[0]) >= 0;
 unsigned long now = millis();


 if (indexPressed && (now - lastFlickTime > FLICK_COOLDOWN)) {
   if (ay > FLICK_THRESHOLD) {
     octaveOffset = constrain(octaveOffset + 12, -24, 24);
     lastFlickTime = now;
     Serial.print("Octave up: "); Serial.println(octaveOffset / 12);
   } else if (ay < -FLICK_THRESHOLD) {
     octaveOffset = constrain(octaveOffset - 12, -24, 24);
     lastFlickTime = now;
     Serial.print("Octave down: "); Serial.println(octaveOffset / 12);
   }
 }


 // Note on/off
 for (int i = 0; i < 4; i++) {
   bool pressed = classifyIndex(deltas[i]) >= 0;


   if (pressed && !wasPressed[i]) {
     int midi = constrain(notesets[set][i] + octaveOffset, 0, 127);
     MIDI.sendNoteOn(midi, 100, 1);
     lastMidi[i] = midi;
   }


   if (!pressed && wasPressed[i]) {
     if (lastMidi[i] >= 0) {
       MIDI.sendNoteOff(lastMidi[i], 0, 1);
       lastMidi[i] = -1;
     }
   }


   wasPressed[i] = pressed;
 }


 delay(50);
}
