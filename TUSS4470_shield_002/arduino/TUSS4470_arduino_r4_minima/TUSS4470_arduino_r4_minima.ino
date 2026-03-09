#include "settings.h"
#include <SPI.h>
#include <FspTimer.h>

// ======================================================================
// --- 雙頻系統配置 ---
// ======================================================================
#define MAX_SAMPLES 18000 
volatile uint16_t currentNumSamples = 2000; 

// --- 雙頻狀態機變數 ---
bool isHighFreqPing = false; 

// [新增] 工作模式：0 = 40kHz, 1 = 200kHz, 2 = 雙頻交錯
volatile uint8_t currentOpMode = 2; 

// --- 接收 Python 傳來的參數 ---
volatile uint8_t currentSampleDelay = 11;
volatile int currentBlindZone = 60;
volatile int baseCycles = 16; // 基準波數 (以 40kHz 為基準)

// ======================================================================
// --- PIN CONFIGURATION ---
// ======================================================================
const int SPI_CS = 10;
const int IO1 = 8;
const int IO2 = 9; 
const int O3 = 3;
const int O4 = 2;  
const int analogIn = A0;

// ======================================================================
// --- REGISTERS (R4) ---
// ======================================================================
#define MSTP_BASE   0x40040000u
#define MSTPCRD    (*(volatile uint32_t *)(MSTP_BASE + 0x7008u))
#define ADC_BASE    0x40050000u
#define ADCSR      (*(volatile uint16_t *)(ADC_BASE + 0xC000u))
#define ADANSA0    (*(volatile uint16_t *)(ADC_BASE + 0xC004u))
#define ADCER      (*(volatile uint16_t *)(ADC_BASE + 0xC00Eu))
#define ADDR09     (*(volatile uint16_t *)(ADC_BASE + 0xC020u + 18))

// ======================================================================
// --- DATA STRUCTURE ---
// ======================================================================
struct __attribute__((packed)) Header {
  uint8_t  start = 0xAA;
  uint16_t depth_index;            
  int16_t  drive_frequency; 
  uint16_t vDrv_scaled;
  uint16_t num_samples; 
};

Header header;
uint8_t sampleBuffer[MAX_SAMPLES]; 

// ======================================================================
// --- GLOBALS ---
// ======================================================================
byte misoBuf[2];  
byte inByteArr[2];  

volatile int pulseCount = 0;
volatile int sampleIndex = 0;

volatile bool detectedDepth = false;  
volatile uint16_t depthDetectSample = 0;

// 宣告單一 Timer 與 通道
FspTimer burstTimer;
uint8_t timer_channel;

// 當下這回合目標要打幾次 Toggle (動態變化)
volatile int currentToggleCount = 32;

// ======================================================================
// --- TIMER CALLBACK ---
// ======================================================================
void burstCallback(timer_callback_args_t *) {
  digitalWrite(IO2, !digitalRead(IO2)); 
  pulseCount++;
  
  if (pulseCount >= currentToggleCount) { 
    burstTimer.stop();
    pulseCount = 0;  
    digitalWrite(IO2, LOW); // 確保最後停在 LOW
  }
}

// 深度偵測中斷
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

// ======================================================================
// --- SPI & UTILS ---
// ======================================================================
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

// ======================================================================
// --- SETUP ---
// ======================================================================
void setup()
{
  Serial.begin(2000000); 

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  
  pinMode(IO2, OUTPUT);
  digitalWrite(IO2, LOW);
  
  pinMode(O4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(O4), handleInterrupt, RISING);

  // 初始化 TUSS4470 暫存器
  tuss4470Write(0x11, 0x10);                        
  tuss4470Write(0x1A, 0x00); // Continuous Mode
  tuss4470Write(0x17, THRESHOLD_VALUE); 
  tuss4470Write(0x13, 0x06); 

  // ===========================================================
  // [關鍵修正] 在 setup 中只把 Timer 建立並開啟一次，避免 loop 中資源崩潰
  // ===========================================================
  uint8_t timerType = GPT_TIMER;
  timer_channel = FspTimer::get_available_timer(timerType);
  
  // 先預設為 40kHz 的頻率 (80000.0f toggles)
  burstTimer.begin(TIMER_MODE_PERIODIC, timerType, timer_channel, 80000.0f, 0.0f, burstCallback);
  burstTimer.setup_overflow_irq();
  burstTimer.open();
  // 注意：這裡不呼叫 start()，保留在 loop 中觸發

  // ADC 初始化
  MSTPCRD &= ~(1u << 16); 
  ADANSA0 = (1u << 9);    
  ADCER = 0x0000;         
  ADCSR &= ~(1u << 5);    
}

// ======================================================================
// --- MAIN LOOP ---
// ======================================================================
void loop()
{
  detectedDepth = false; 
  depthDetectSample = 0;

  // ---------------------------------------------------------
  // 1. 動態配置這回合的硬體參數 (使用安全的 set_frequency)
  // ---------------------------------------------------------
  
  if (isHighFreqPing) {
    // 【高頻回合：200kHz】
    tuss4470Write(0x10, 0x1E); // BPF 設為 ~206kHz
    burstTimer.set_frequency(400000.0f); // 動態變更 Timer 頻率為 400k (達成 200kHz 方波)
    currentToggleCount = baseCycles * 2; 
  } else {
    // 【低頻回合：40kHz】
    tuss4470Write(0x10, 0x00); // BPF 設為 ~40.6kHz
    burstTimer.set_frequency(80000.0f); // 動態變更 Timer 頻率為 80k (達成 40kHz 方波)
    currentToggleCount = baseCycles * 2; 
  }

  // ---------------------------------------------------------
  // 2. 喚醒晶片與發射
  // ---------------------------------------------------------
  tuss4470Write(0x1B, 0x01);
  burstTimer.start(); 

  // ---------------------------------------------------------
  // 3. 高速 ADC 採樣迴圈
  // ---------------------------------------------------------
  for (sampleIndex = 0; sampleIndex < currentNumSamples; sampleIndex++) {
    ADCSR |= (1u << 15);        
    while (ADCSR & (1u << 15));  
    
    uint8_t rawVal = ADDR09 >> 4;
    sampleBuffer[sampleIndex] = rawVal;

    delayMicroseconds(currentSampleDelay); 

    if (sampleIndex < currentBlindZone) {
      detectedDepth = false;
      depthDetectSample = 0;
    }
  }

  tuss4470Write(0x1B, 0x00);
  
  // ---------------------------------------------------------
  // 4. 深度覆蓋邏輯
  // ---------------------------------------------------------
  #if USE_DEPTH_OVERRIDE
  int overrideSample = 0;
  uint8_t max = 0;
  for (int i = currentBlindZone; i < currentNumSamples; i++) {
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
    header.vDrv_scaled = 0; 
  #endif
  
  // ---------------------------------------------------------
  // 5. 標記與傳送資料
  // ---------------------------------------------------------
  header.drive_frequency = isHighFreqPing ? 200 : 40; 
  sendData();

  // ---------------------------------------------------------
  // 6. 處理 Python 指令 (使用 peek 來確保封包完整性)
  // ---------------------------------------------------------
  if (Serial.available() > 0) { 
    byte cmd = Serial.peek(); // 先偷看指令是什麼，不消耗
    
    if (cmd == 'W') { 
      if (Serial.available() >= 3) {
        Serial.read(); // 吃掉 'W'
        byte addr = Serial.read();
        byte data = Serial.read();
        stopTransducer(); 
        tuss4470Write(addr, data);
      }
    }
    else if (cmd == 'N') { 
      if (Serial.available() >= 3) {
        Serial.read(); // 吃掉 'N'
        byte high = Serial.read();
        byte low = Serial.read();
        uint16_t newSamples = (high << 8) | low;
        
        if (newSamples > MAX_SAMPLES) newSamples = MAX_SAMPLES;
        if (newSamples < 100) newSamples = 100;
        currentNumSamples = newSamples;
      }
    }
    else if (cmd == 'D') { 
      // 現在 D 指令有 5 個 bytes (D + val + cycles + blind + mode)
      if (Serial.available() >= 5) {
        Serial.read(); // 吃掉 'D'
        byte val = Serial.read();    
        byte cycles = Serial.read(); 
        byte blind = Serial.read();  
        byte mode = Serial.read();   // 0=40k, 1=200k, 2=Dual
        
        if (val < 5) val = 5;
        currentSampleDelay = val;
        
        if (cycles > 0) {
            if (cycles > 128) cycles = 128; 
            baseCycles = cycles; 
        }
        
        currentBlindZone = (int)blind;
        
        // 更新工作模式
        if (mode <= 2) currentOpMode = mode;
      }
    }
    else {
      // 遇到不認識的垃圾字元，清掉它
      Serial.read();
    }
  }

  // ---------------------------------------------------------
  // 7. 根據工作模式決定狀態反轉
  // ---------------------------------------------------------
  if (currentOpMode == 0) {
    isHighFreqPing = false;      // 強制鎖定 40kHz
  } else if (currentOpMode == 1) {
    isHighFreqPing = true;       // 強制鎖定 200kHz
  } else {
    isHighFreqPing = !isHighFreqPing; // 雙頻交錯
  }

  delay(10);
}

void sendData() {
  header.depth_index = depthDetectSample;
  header.num_samples = currentNumSamples; 
  
  uint8_t cs = 0;
  cs ^= (uint8_t)(header.depth_index & 0xFF);
  cs ^= (uint8_t)(header.depth_index >> 8);
  cs ^= (uint8_t)(header.drive_frequency & 0xFF);
  cs ^= (uint8_t)(header.drive_frequency >> 8);
  cs ^= (uint8_t)(header.vDrv_scaled & 0xFF);
  cs ^= (uint8_t)(header.vDrv_scaled >> 8);
  cs ^= (uint8_t)(header.num_samples & 0xFF); 
  cs ^= (uint8_t)(header.num_samples >> 8);
  
  for (int i = 0; i < currentNumSamples; i++) {
    cs ^= sampleBuffer[i];
  }
  
  Serial.write(header.start);
  Serial.write((uint8_t*)&header.depth_index, 2);
  Serial.write((uint8_t*)&header.drive_frequency, 2);
  Serial.write((uint8_t*)&header.vDrv_scaled, 2);
  Serial.write((uint8_t*)&header.num_samples, 2);
  
  Serial.write(sampleBuffer, currentNumSamples);
  
  Serial.write(cs);
}
