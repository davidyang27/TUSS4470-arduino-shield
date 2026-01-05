#include "settings.h"
#include <SPI.h>
#include <FspTimer.h>

// ---------------------- CONFIG ----------------------
#define MAX_SAMPLES 18000 // 靜態分配最大記憶體 (32KB SRAM 足夠)
volatile uint16_t currentNumSamples = 2000; // 預設啟動時的採樣數

// ---------------------- PIN CONFIGURATION ----------------------
const int SPI_CS = 10;
const int IO1 = 8;
const int IO2 = 9; 
const int O3 = 3;
const int O4 = 2;  
const int analogIn = A0;

// ---------------------- REGISTERS (R4) ----------------------
#define MSTP_BASE   0x40040000u
#define MSTPCRD    (*(volatile uint32_t *)(MSTP_BASE + 0x7008u))
#define ADC_BASE    0x40050000u
#define ADCSR      (*(volatile uint16_t *)(ADC_BASE + 0xC000u))
#define ADANSA0    (*(volatile uint16_t *)(ADC_BASE + 0xC004u))
#define ADCER      (*(volatile uint16_t *)(ADC_BASE + 0xC00Eu))
#define ADDR09     (*(volatile uint16_t *)(ADC_BASE + 0xC020u + 18))

// ---------------------- DATA STRUCTURE ----------------------
// [修改] Header 加入 num_samples
struct __attribute__((packed)) Header {
  uint8_t  start = 0xAA;
  uint16_t depth_index;            
  int16_t  temp_scaled;     
  uint16_t vDrv_scaled;
  uint16_t num_samples; // 告訴 Python 這次後面跟著多少數據
};

Header header;
uint8_t sampleBuffer[MAX_SAMPLES]; // 固定分配最大空間

// ---------------------- GLOBALS ----------------------
byte misoBuf[2];  
byte inByteArr[2];  

volatile int pulseCount = 0;
volatile int sampleIndex = 0;

float temperature = 0.0f;
int vDrv = 0;

volatile bool detectedDepth = false;  
volatile uint16_t depthDetectSample = 0;

FspTimer burstTimer;

void burstCallback(timer_callback_args_t *) {
  digitalWrite(IO2, !digitalRead(IO2)); 
  pulseCount++;
  if (pulseCount >= 32) { 
    burstTimer.stop();
    pulseCount = 0;  
  }
}

void handleInterrupt() {
  if (!detectedDepth) {
    depthDetectSample = sampleIndex;
    detectedDepth = true;
  }
}

void stopTransducer() {
  burstTimer.stop();
  pulseCount = 0;
  digitalWrite(IO2, LOW); 
}

unsigned int BitShiftCombine(unsigned char x_high, unsigned char x_low) {
  return (x_high << 8) | x_low;  
}

byte parity16(unsigned int val) {
  byte ones = 0;
  for (int i = 0; i < 16; i++) {
    if ((val >> i) & 1) { ones++; }
  }
  return (ones + 1) % 2;  
}

void spiTransfer(byte* mosi, byte sizeOfArr) {
  memset(misoBuf, 0x00, sizeof(misoBuf));
  digitalWrite(SPI_CS, LOW);
  for (int i = 0; i < sizeOfArr; i++) {
    misoBuf[i] = SPI.transfer(mosi[i]);
  }
  digitalWrite(SPI_CS, HIGH);
}

void tuss4470Write(byte addr, byte data) {
  inByteArr[0] = (addr & 0x3F) << 1;  
  inByteArr[1] = data;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
}

byte tuss4470Read(byte addr) {
  inByteArr[0] = 0x80 + ((addr & 0x3F) << 1);  
  inByteArr[1] = 0x00;  
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
  return misoBuf[1];
}

byte tuss4470Parity(byte* spi16Val) {
  return parity16(BitShiftCombine(spi16Val[0], spi16Val[1]));
}

void setup()
{
  Serial.begin(2000000); // 2M Baud

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  pinMode(IO2, OUTPUT);
  pinMode(O4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(O4), handleInterrupt, RISING);

  tuss4470Write(0x10, FILTER_FREQUENCY_REGISTER);  
  tuss4470Write(0x11, 0x10);                       
  tuss4470Write(0x16, 0xF); 
  tuss4470Write(0x1A, 0x0F); 
  tuss4470Write(0x17, THRESHOLD_VALUE); 
  tuss4470Write(0x13, 0x06); 

  uint8_t timerType = GPT_TIMER;
  uint8_t channel = FspTimer::get_available_timer(timerType);
  burstTimer.begin(TIMER_MODE_PERIODIC, timerType, channel, DRIVE_FREQUENCY * 2.0f, 0.0f, burstCallback);
  burstTimer.setup_overflow_irq();
  burstTimer.open();

  MSTPCRD &= ~(1u << 16); 
  ADANSA0 = (1u << 9);    
  ADCER = 0x0000;         
  ADCSR &= ~(1u << 5);    
}

void loop()
{
  detectedDepth = false; 
  depthDetectSample = 0;

  tuss4470Write(0x1B, 0x01);
  burstTimer.start();

  // [關鍵] 使用 currentNumSamples 進行迴圈
  for (sampleIndex = 0; sampleIndex < currentNumSamples; sampleIndex++) {
    ADCSR |= (1u << 15);        
    while (ADCSR & (1u << 15));  
    
    sampleBuffer[sampleIndex] = ADDR09 >> 4; 

    delayMicroseconds(11.5); 

    if (sampleIndex == BLINDZONE_SAMPLE_END) {
      detectedDepth = false;
      depthDetectSample = 0;
    }
  }

  tuss4470Write(0x1B, 0x00);
  
  #if USE_DEPTH_OVERRIDE
  int overrideSample = 0;
  uint8_t max = 0;
  for (int i = BLINDZONE_SAMPLE_END; i < currentNumSamples; i++) {
    if (sampleBuffer[i] > max) {
      max = sampleBuffer[i];
      overrideSample = i;
    }
  }
  if (overrideSample > 0) {
    header.vDrv_scaled = (uint16_t)overrideSample;
  } else {
    header.vDrv_scaled = 0;
  }
  #else
    header.vDrv_scaled = (uint16_t)(vDrv); 
  #endif
  
  sendData();

  // --- Serial Command Parser ---
  if (Serial.available() >= 3) {
    byte cmd = Serial.read(); 
    
    if (cmd == 'W') { 
      byte addr = Serial.read();
      byte data = Serial.read();
      stopTransducer(); 
      tuss4470Write(addr, data);
    }
    else if (cmd == 'N') { // [新增] 設定 Num Samples 指令
      byte high = Serial.read();
      byte low = Serial.read();
      uint16_t newSamples = (high << 8) | low;
      
      if (newSamples > MAX_SAMPLES) newSamples = MAX_SAMPLES;
      if (newSamples < 100) newSamples = 100;
      
      currentNumSamples = newSamples;
    }
  }

  delay(10);
}

void sendData() {
  header.depth_index = depthDetectSample;
  header.temp_scaled = (int16_t)(DRIVE_FREQUENCY / 1000); 
  header.num_samples = currentNumSamples; // 告訴 Python 這次的長度
  
  uint8_t cs = 0;
  cs ^= (uint8_t)(header.depth_index & 0xFF);
  cs ^= (uint8_t)(header.depth_index >> 8);
  cs ^= (uint8_t)(header.temp_scaled & 0xFF);
  cs ^= (uint8_t)(header.temp_scaled >> 8);
  cs ^= (uint8_t)(header.vDrv_scaled & 0xFF);
  cs ^= (uint8_t)(header.vDrv_scaled >> 8);
  cs ^= (uint8_t)(header.num_samples & 0xFF); // 加入 num_samples 到校驗
  cs ^= (uint8_t)(header.num_samples >> 8);
  
  for (int i = 0; i < currentNumSamples; i++) {
    cs ^= sampleBuffer[i];
  }
  
  // Header: 1(AA) + 2(depth) + 2(temp) + 2(vdrv) + 2(num) = 9 bytes
  Serial.write(header.start);
  Serial.write((uint8_t*)&header.depth_index, 2);
  Serial.write((uint8_t*)&header.temp_scaled, 2);
  Serial.write((uint8_t*)&header.vDrv_scaled, 2);
  Serial.write((uint8_t*)&header.num_samples, 2);
  
  // Data
  Serial.write(sampleBuffer, currentNumSamples);
  
  // Checksum
  Serial.write(cs);
}
