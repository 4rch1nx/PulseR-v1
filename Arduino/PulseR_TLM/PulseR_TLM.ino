// TLM
#include <Wire.h>
#include <SPI.h>
#include <SD_fix.h>
#include <LoRa.h>

#define SD_TLM_CS 4
#define LORA_DIO0 2
#define LORA_RST 9
#define LORA_NSS 10
#define LED_R 5
#define LED_G 6
#define LED_B 7

uint8_t TeamID = 0xFF;
uint8_t pktId = 0;
File logFileTLM;
unsigned long lastLog = 0;
const unsigned long LOG_INT = 200;   // 5 Hz
uint8_t checkCommandsMaxTime = 100;  // 10 Hz
bool sdOK, loraOK;
char init_code;

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, !r);
  digitalWrite(LED_G, !g);
  digitalWrite(LED_B, !b);
}

void sendAck(int cmd) {
  char ack[30];
  snprintf(ack, sizeof(ack), "ACK;%d", cmd);
  uint8_t cs = 0;
  for (char *p = ack; *p; p++) cs += *p;
  LoRa.beginPacket();
  LoRa.print(ack);
  LoRa.print(';');
  LoRa.print(cs);
  LoRa.endPacket();
  LoRa.receive();
}

void checkCommands() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  char buf[80];
  int i = 0;
  while (LoRa.available() && i < (int)sizeof(buf) - 1)
    buf[i++] = LoRa.read();
  buf[i] = '\0';

  char *lastSep = strrchr(buf, ';');
  if (!lastSep) return;
  uint8_t rxCS = (uint8_t)atoi(lastSep + 1);
  *lastSep = '\0';
  uint8_t calcCS = 0;
  for (char *p = buf; *p; p++) calcCS += *p;
  if (calcCS != rxCS) return;

  if (strncmp(buf, "CMD;", 4) != 0) return;

  char *token;
  token = strtok(buf, ";");   // "CMD"
  token = strtok(NULL, ";");  // team ID (ignored)
  token = strtok(NULL, ";");  // cmd number
  if (!token) return;
  int cmd = atoi(token);

  token = strtok(NULL, ";");
  if (!token) return;
  int val = atoi(token);

  if (cmd == 2) {
    if (val == 2)  //  && pyro1_armed
      ;            //pyro1_state = true
    else
      ;  // pyro1_armed = (bool)val
  }
  if (cmd == 3) {
    setLED(0, 0, 1);
    /*
    ready = (bool)val;
    pyro1_armed = (bool)val;
    start_alt = water_alt;
    apogee = 0;
    max_vel = 0;
    fusionReset(0.0f);
    checkCommandsMaxTime = ready ? 50 : 100;
    */
  }
  if (cmd == 5) {
    /*
    apogee = 0;
    max_vel = 0;
    start_alt = water_alt;
    fusionReset(0.0f);
    */
  }

  sendAck(cmd);
}

void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(SD_TLM_CS, OUTPUT);
  setLED(1, 1, 0);
  Wire.begin();
  SPI.begin();

  sdOK = SD.begin(SD_TLM_CS);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(50);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  loraOK = LoRa.begin(433E6);
  if (loraOK) LoRa.setTxPower(13);

  if (sdOK && loraOK) {
    init_code = 0;
    setLED(0, 1, 0);
    delay(500);
  } else {
    // if (!bmeOK) init_code |= (1 << 0);
    // if (!mpuOK) init_code |= (1 << 1);
    // if (!qmcOK) init_code |= (1 << 2);
    if (!sdOK) init_code |= (1 << 3);
    if (!loraOK) init_code |= (1 << 4);
    setLED(1, 0, 0);
    delay(1000);
  }
  if (sdOK) {
    char name[] = "T00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      name[1] = '0' + (i / 10);
      name[2] = '0' + (i % 10);
      if (!SD.exists(name)) {
        logFileTLM = SD.open(name, FILE_WRITE);
        break;
      }
    }
    if (logFileTLM) {
      logFileTLM.println(F("teamid,time,alt,alx,aly,alz,grx,gry,grz,heading,"
                           "ready,launched,hitapg,landed,inboost,burnout,"
                           "armed,state,fired,temp,prs,hum,voltage,init,"
                           "vel,maxvel,apogee,logid"));
      logFileTLM.flush();
    }
  }
}

void loop() {
  unsigned long now = millis();

  static uint8_t logId = 0;
  if (logFileTLM && (now - lastLog >= LOG_INT)) {
    logFileTLM.print(TeamID);
    logFileTLM.print(',');
    logFileTLM.println(logId++);
    logFileTLM.flush();
    lastLog = now;
  }

  // ── Telemetry packet ──────────────────────────────────────────────
  int32_t alt_i = (int32_t)(altitude * 100.0f);
  int16_t volt_i = (int16_t)(voltage * 100.0f);
  int32_t vel_i = (int32_t)(vel * 100.0f);
  int32_t apo_i = (int32_t)(apogee * 100.0f);
  uint32_t pressure_i = (uint32_t)pressure;

  char packet[180];
  snprintf(packet, sizeof(packet),
           "%02X;%lu;%ld;%d;%d;%d;%u;%u;%u;%d;%lu;%u;%d;%u;%ld;%ld;%u",
           TeamID, now,
           alt_i, alx, aly, alz,
           heading, status_code, pyro1_code,
           temperature, pressure_i, humidity,
           volt_i, init_code,
           vel_i, apo_i,
           pktId++);

  uint8_t checksum = 0;
  for (char *p = packet; *p; p++) checksum += *p;

  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.print(';');
  LoRa.print(checksum);
  LoRa.endPacket();
  LoRa.receive();

  unsigned long t = millis();
  while (millis() - t < checkCommandsMaxTime)
    checkCommands();
}
