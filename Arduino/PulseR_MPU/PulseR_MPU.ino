// MPU
#include <Wire.h>
#include <SPI.h>
#include <SD_fix.h>
#include <LoRa.h>
#include <GyverBME280.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>

#define SD_CS 4
#define S_RAW A1
#define SERVO1_PIN 7
#define SERVO2_PIN A2
#define LED_R 8
#define LED_G 9
#define LED_B 10

#define PROC_CON_1 5
#define PROC_CON_2 6

#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D
#define BME280_ADDR 0x76
GyverBME280 bme;
MPU6050 mpu;

#define BOOST_THRESHOLD 4096    // 2.00 g
#define BURNOUT_THRESHOLD 2150  // 1.05 g
#define BOOST_CONFIRM_CNT 2
#define BURNOUT_CONFIRM_CNT 2

uint8_t TeamID = 0xFF;
uint8_t pktId = 0;
char init_code = 0;
char status_code = 0;
char servo_code = 0;

bool bmeOK, mpuOK, qmcOK, sdOK;
bool ready = false;
bool launched = false;
bool hit_apogee = false;
bool landed = false;
bool alt_threshold = false;
bool in_boost = false;
bool burnout_detected = false;

float max_vel = 0;
float start_alt;
float apogee = 0;
float voltage = 0;
float r1 = 29800;
float r2 = 7490;
float min_fire_alt = 20.0f;
float water_alt;
const float g = 9.815f;

File logFile;
unsigned long lastLog = 0;
const unsigned long LOG_INT = 50;  // 20 Hz

struct {
  float alt;
  float vel_v;
  float vel_v_imu;
  float vel_fwd;
  float last_baro;
  unsigned long lastT;
  bool initialized;
} fuse;

void fusionReset(float baro_rel) {
  fuse.alt = baro_rel;
  fuse.vel_v = 0.0f;
  fuse.vel_v_imu = 0.0f;
  fuse.vel_fwd = 0.0f;
  fuse.last_baro = baro_rel;
  fuse.lastT = millis();
  fuse.initialized = true;
}

void fusionUpdate(float baro_rel, int16_t alx, int16_t aly, int16_t alz) {
  if (!fuse.initialized) {
    fusionReset(baro_rel);
    return;
  }
  unsigned long now = millis();
  float dt = (now - fuse.lastT) * 0.001f;
  fuse.lastT = now;
  if (dt <= 0.0f || dt > 0.3f) dt = 0.02f;
  // ---------------------------------------------------
  // BARO vertical velocity
  // ---------------------------------------------------
  float vel_baro = (baro_rel - fuse.last_baro) / dt;
  fuse.last_baro = baro_rel;
  vel_baro = constrain(vel_baro, -400, 400);
  // ---------------------------------------------------
  // MPU scaling for ±16G
  // ---------------------------------------------------
  float ax = ((float)alx / 2048.0f) * g;
  float ay = ((float)aly / 2048.0f) * g;
  float az = ((float)alz / 2048.0f) * g;
  // Vertical axis compensation
  float acc_v = ay - g;
  // Horizontal magnitude
  float acc_fwd = sqrt(ax * ax + az * az);

  if (fabs(acc_v) < 0.25f) acc_v = 0;
  if (acc_fwd < 0.25f) acc_fwd = 0;
  // ---------------------------------------------------
  // BEFORE launch / landed = zero MPU drift
  // ---------------------------------------------------
  if (!launched || landed) {
    acc_v = 0;
    acc_fwd = 0;
    fuse.vel_v_imu = 0;
    fuse.vel_fwd = 0;
  }
  // ---------------------------------------------------
  // AFTER APOGEE = disable MPU contribution
  // ---------------------------------------------------
  if (hit_apogee) {
    acc_v = 0;
    acc_fwd = 0;
  }
  // ---------------------------------------------------
  // Integrate velocities
  // ---------------------------------------------------
  fuse.vel_v_imu += acc_v * dt;
  fuse.vel_fwd += acc_fwd * dt;
  fuse.vel_v_imu = constrain(fuse.vel_v_imu, -500, 500);
  fuse.vel_fwd = constrain(fuse.vel_fwd, 0, 500);
  // ---------------------------------------------------
  // Weighting
  // ---------------------------------------------------
  float imu_w;
  if (!launched || landed || hit_apogee) {
    imu_w = 0.0f;  // after apogee = baro only
  } else if (in_boost) {
    imu_w = 0.90f;
  } else {
    imu_w = 0.45f;
  }
  // ---------------------------------------------------
  // Vertical fused velocity
  // ---------------------------------------------------
  fuse.vel_v = imu_w * fuse.vel_v_imu + (1.0f - imu_w) * vel_baro;

  // drift correction
  if (!in_boost && !hit_apogee) {
    fuse.vel_v_imu = fuse.vel_v_imu * 0.97f + vel_baro * 0.03f;
  }
  // ---------------------------------------------------
  // Altitude integration
  // ---------------------------------------------------
  fuse.alt += fuse.vel_v * dt;
  float alpha = in_boost ? 0.01f : 0.05f;
  fuse.alt = (1.0f - alpha) * fuse.alt + alpha * baro_rel;
}

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, !r);
  digitalWrite(LED_G, !g);
  digitalWrite(LED_B, !b);
}

void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLED(1, 1, 0);

  Wire.begin();
  SPI.begin();

  bmeOK = bme.begin(BME280_ADDR);

  mpu.initialize();
  mpu.setFullScaleAccelRange(3);
  mpuOK = mpu.testConnection();
  mpu.setXAccelOffset(-80);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(141);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(-32);

  qmcOK = true;
  sdOK = SD.begin(SD_CS);

  if (bmeOK && mpuOK && qmcOK && sdOK) {
    init_code = 0;
    setLED(0, 1, 0);
    delay(500);
  } else {
    if (!bmeOK) init_code |= (1 << 0);
    if (!mpuOK) init_code |= (1 << 1);
    if (!qmcOK) init_code |= (1 << 2);
    if (!sdOK) init_code |= (1 << 3);
    setLED(1, 0, 0);
    delay(1000);
  }

  if (sdOK) {
    char name[] = "M00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      name[1] = '0' + (i / 10);
      name[2] = '0' + (i % 10);
      if (!SD.exists(name)) {
        logFile = SD.open(name, FILE_WRITE);
        break;
      }
    }
    if (logFile) {
      logFile.println(F("teamid,time,alt,alx,aly,alz,grx,gry,grz,heading,"
                        "ready,launched,hitapg,landed,inboost,burnout,"
                        "armed,state,fired,temp,prs,hum,voltage,init,"
                        "vel,maxvel,apogee,logid"));
      logFile.flush();
    }
  }

  setLED(1, 0, 1);
  delay(1000);

  float start_prs = bme.readPressure();
  start_alt = pressureToAltitude(start_prs);
  fuse.initialized = false;
}

void loop() {
  unsigned long now = millis();

  // === Read sensors ===
  float pressure = bme.readPressure();
  water_alt = pressureToAltitude(pressure);
  int8_t temperature = bme.readTemperature();
  uint8_t humidity = bme.readHumidity();

  int16_t alx, aly, alz, grx, gry, grz;
  mpu.getMotion6(&alx, &aly, &alz, &grx, &gry, &grz);

  int v_raw = analogRead(S_RAW);
  float tmp = (v_raw * 5.0f) / 1024.0f;
  voltage = (tmp / (r2 / (r1 + r2))) + 0.35f;

  // === Sensor fusion ===
  float baro_rel = water_alt - start_alt;
  fusionUpdate(baro_rel, alx, aly, alz);

  float altitude = fuse.alt;
  float vel = fuse.vel_v;
  float vel_fwd = fuse.vel_fwd;

  if (altitude > apogee) apogee = altitude;
  if (vel_fwd > max_vel) max_vel = vel_fwd;

  // === Status bytes ===
  status_code = 0;
  if (ready) status_code |= (1 << 0);
  if (launched) status_code |= (1 << 1);
  if (hit_apogee) status_code |= (1 << 2);
  if (landed) status_code |= (1 << 3);
  if (in_boost) status_code |= (1 << 4);
  if (burnout_detected) status_code |= (1 << 5);

  /*
  // change to servo
  pyro1_code = 0;
  if (pyro1_armed) pyro1_code |= (1 << 0);
  if (pyro1_state) pyro1_code |= (1 << 1);
  if (pyro1_fired) pyro1_code |= (1 << 2);
  */

  // === FLIGHT LOGIC ===
  // = Launch detect =
  static unsigned long launchTime = 0;
  if (ready && !launched && (aly > BOOST_THRESHOLD || altitude > 4.0f)) {
    launched = true;
    launchTime = now;
    setLED(1, 1, 0);
  }

  if (launched && altitude > min_fire_alt && !alt_threshold)
    alt_threshold = true;

  // = Apogee detect =
  if (launched && !hit_apogee && vel < -0.3f && apogee - altitude > 0.5f) {
    hit_apogee = true;
    setLED(1, 0, 0);
  }

  // = Landing detect =
  static unsigned long landTime = 0;
  if (hit_apogee && !landed && fabsf(vel) < 3.0f) {
    if (landTime == 0) landTime = now;
    if (now - landTime > 2000) {
      setLED(0, 1, 0);
      landed = true;
      // pyro1_armed = false;
    }
  } else {
    landTime = 0;
  }

  // = Servo drive =
  // static unsigned long pyro1_fire_time = 0;
  // if (pyro1_state) {
  //   if (pyro1_fire_time == 0) {
  //     pyro1_fire_time = millis();
  //     digitalWrite(PYRO_PIN, HIGH);
  //   }
  //   if (millis() - pyro1_fire_time >= pyroFireTime) {
  //     digitalWrite(PYRO_PIN, LOW);
  //     pyro1_state = false;
  //     pyro1_fired = true;
  //     pyro1_fire_time = 0;
  //   }
  // }

  // === SD log ===
  static uint8_t logId = 0;
  if (logFile && (now - lastLog >= LOG_INT)) {
    logFile.print(TeamID);
    logFile.print(',');
    logFile.print(now);
    logFile.print(',');
    logFile.print(altitude, 2);
    logFile.print(',');
    logFile.print(alx);
    logFile.print(',');
    logFile.print(aly);
    logFile.print(',');
    logFile.print(alz);
    logFile.print(',');
    logFile.print(grx);
    logFile.print(',');
    logFile.print(gry);
    logFile.print(',');
    logFile.print(grz);
    logFile.print(',');
    logFile.print(ready);
    logFile.print(',');
    logFile.print(launched);
    logFile.print(',');
    logFile.print(hit_apogee);
    logFile.print(',');
    logFile.print(landed);
    logFile.print(',');
    logFile.print(in_boost);
    logFile.print(',');
    logFile.print(burnout_detected);
    logFile.print(',');
    logFile.print(temperature);
    logFile.print(',');
    logFile.print(pressure, 1);
    logFile.print(',');
    logFile.print(humidity);
    logFile.print(',');
    logFile.print(voltage, 2);
    logFile.print(',');
    logFile.print((int)init_code);
    logFile.print(',');
    logFile.print(vel, 2);
    logFile.print(',');
    logFile.print(max_vel, 2);
    logFile.print(',');
    logFile.print(apogee, 2);
    logFile.print(',');
    logFile.println(logId++);
    logFile.flush();
    lastLog = now;
  }
}
