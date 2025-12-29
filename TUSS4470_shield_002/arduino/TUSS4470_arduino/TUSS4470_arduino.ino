#include "settings.h"
#include <SPI.h>

// Pin configuration
const int SPI_CS = 10;
const int IO1 = 8;
const int IO2 = 9;
const int O3 = 3;
const int O4 = 2;
const int analogIn = A0;

struct __attribute__((packed)) Frame {
  uint8_t start = 0xAA;
  uint16_t depth_index;
  int16_t temp_scaled;
  uint16_t vDrv_scaled;
  uint8_t samples[NUM_SAMPLES];
  uint8_t checksum;
};

static Frame frame;             

byte misoBuf[2];    
byte inByteArr[2];  
float temperature = 0.0f;
int vDrv = 0;

volatile uint8_t pulseCount = 0;
volatile int sampleIndex = 0;

volatile bool detectedDepth = false;  
volatile uint16_t depthDetectSample = 0;

ISR(TIMER1_COMPA_vect) {
  pulseCount++;
  if (pulseCount >= 32) {
    stopTransducer();
    pulseCount = 0;  
  }
}

void startTransducerBurst() {
  TCCR1A = _BV(COM1A0);             
  TCCR1B = _BV(WGM12) | _BV(CS10);  

  OCR1A = DRIVE_FREQUENCY_TIMER_DIVIDER;

  TIMSK1 = _BV(OCIE1A);  
}

void stopTransducer() {
  TCCR1A = 0;
  TCCR1B = 0;  
  TIMSK1 = 0;  
}

byte tuss4470Read(byte addr) {
  inByteArr[0] = 0x80 + ((addr & 0x3F) << 1);  
  inByteArr[1] = 0x00;                         
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));

  return misoBuf[1];
}

void tuss4470Write(byte addr, byte data) {
  inByteArr[0] = (addr & 0x3F) << 1;  
  inByteArr[1] = data;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
}

byte tuss4470Parity(byte* spi16Val) {
  return parity16(BitShiftCombine(spi16Val[0], spi16Val[1]));
}

void spiTransfer(byte* mosi, byte sizeOfArr) {
  memset(misoBuf, 0x00, sizeof(misoBuf));

  digitalWrite(SPI_CS, LOW);
  for (int i = 0; i < sizeOfArr; i++) {
    misoBuf[i] = SPI.transfer(mosi[i]);
  }
  digitalWrite(SPI_CS, HIGH);
}

unsigned int BitShiftCombine(unsigned char x_high, unsigned char x_low) {
  return (x_high << 8) | x_low;  
}

byte parity16(unsigned int val) {
  byte ones = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if ((val >> i) & 1) {
      ones++;
    }
  }
  return (ones + 1) % 2;  
}

void handleInterrupt() {
  // 硬體中斷：只要偵測到 OUT4 上升緣就記錄
  // 過濾工作由 Loop 中的 BLINDZONE_SAMPLE_END 重置來完成
  if (!detectedDepth) {
    depthDetectSample = sampleIndex;
    detectedDepth = true;
  }
}

void setup() {
  Serial.begin(250000);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE1); 

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  pinMode(IO2, OUTPUT);
  pinMode(O4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(O4), handleInterrupt, RISING);

  // Init TUSS4470
  tuss4470Write(0x10, FILTER_FREQUENCY_REGISTER); 
  tuss4470Write(0x11, 0x10);                       
  tuss4470Write(0x16, 0xF);                        
  tuss4470Write(0x1A, 0x0F);                       
  tuss4470Write(0x17, THRESHOLD_VALUE);            
  tuss4470Write(0x13, 0x06);                       

  // ADC Setup
  ADCSRA = (1 << ADEN) | (1 << ADPS2);   
  ADMUX = (1 << REFS0);    
  ADCSRB = 0;              
  ADCSRA |= (1 << ADATE);  
  ADCSRA |= (1 << ADSC);   
}

void loop() {
  // Loop 開始前不重置，而是在 BlindZone 結束點重置
  
  tuss4470Write(0x1B, 0x01); // Start TOF

  startTransducerBurst();

  for (sampleIndex = 0; sampleIndex < NUM_SAMPLES; sampleIndex++) {
    while (!(ADCSRA & (1 << ADIF)));            
    ADCSRA |= (1 << ADIF);                      
    uint8_t v = ADC >> 2;                       
    frame.samples[sampleIndex] = v;

    // [關鍵恢復] 硬體 Blind Zone 過濾
    // 當採樣點到達 BlindZone 結束點時，強制清除前面偵測到的任何訊號(餘震)
    // 這樣之後偵測到的才是真正的回波
    if (sampleIndex == BLINDZONE_SAMPLE_END) {
       detectedDepth = false;
       depthDetectSample = 0; 
    }
  }

  tuss4470Write(0x1B, 0x00); // Stop TOF

#if USE_DEPTH_OVERRIDE
  int overrideSample = 0;
  uint8_t maxVal = 0; 
  for (int i = BLINDZONE_SAMPLE_END; i < NUM_SAMPLES; i++) {
    if (frame.samples[i] > maxVal) {
      maxVal = frame.samples[i];
      overrideSample = i;
    }
  }
  
  if (overrideSample > 0) {
    frame.vDrv_scaled = (uint16_t)overrideSample; 
  } else {
    frame.vDrv_scaled = 0;
  }
#else
  frame.vDrv_scaled = (uint16_t)(vDrv); 
#endif

  sendData(); 

  // --- Serial Command Parser (只保留 'W' 寫入指令) ---
  if (Serial.available() >= 3) {
    if (Serial.read() == 'W') { 
      byte addr = Serial.read();
      byte data = Serial.read();
      stopTransducer(); 
      tuss4470Write(addr, data);
    }
  }

  delay(10);
}

void sendData() {
  frame.depth_index = depthDetectSample;
  frame.temp_scaled = (int16_t)(DRIVE_FREQUENCY / 1000); 
  
  frame.checksum = 0;
  frame.checksum ^= (uint8_t)(frame.depth_index & 0xFF);
  frame.checksum ^= (uint8_t)(frame.depth_index >> 8);
  frame.checksum ^= (uint8_t)(frame.temp_scaled & 0xFF);
  frame.checksum ^= (uint8_t)(frame.temp_scaled >> 8);
  frame.checksum ^= (uint8_t)(frame.vDrv_scaled & 0xFF);
  frame.checksum ^= (uint8_t)(frame.vDrv_scaled >> 8);

  for (int i = 0; i < NUM_SAMPLES; i++) {
    frame.checksum ^= frame.samples[i];
  }

  const size_t len = 1 + 2 + 2 + 2 + NUM_SAMPLES + 1;
  Serial.write(reinterpret_cast<uint8_t*>(&frame), len);
}
