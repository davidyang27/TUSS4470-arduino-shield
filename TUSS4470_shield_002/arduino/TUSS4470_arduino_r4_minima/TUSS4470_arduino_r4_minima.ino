#include "settings.h"
#include <SPI.h>
#include <FspTimer.h> // Arduino R4 內建計時器庫

// ---------------------- PIN CONFIGURATION ----------------------
const int SPI_CS = 10;
const int IO1 = 8;
const int IO2 = 9; // Transducer drive pin
const int O3 = 3;
const int O4 = 2;  // Threshold interrupt pin
const int analogIn = A0;

// ---------------------- ADC REGISTERS FOR R4 ----------------------
// Direct register access for high-speed ADC on RA4M1
#define MSTP_BASE   0x40040000u
#define MSTPCRD    (*(volatile uint32_t *)(MSTP_BASE + 0x7008u))
#define ADC_BASE    0x40050000u
#define ADCSR      (*(volatile uint16_t *)(ADC_BASE + 0xC000u))
#define ADANSA0    (*(volatile uint16_t *)(ADC_BASE + 0xC004u))
#define ADCER      (*(volatile uint16_t *)(ADC_BASE + 0xC00Eu))
#define ADDR09     (*(volatile uint16_t *)(ADC_BASE + 0xC020u + 18)) // Channel AN09 (A0 pin)

// ---------------------- DATA STRUCTURE ----------------------
struct __attribute__((packed)) Frame {
  uint8_t  start = 0xAA;
  uint16_t  depth_index;            
  int16_t  temp_scaled;     
  uint16_t vDrv_scaled;     
  uint8_t  samples[NUM_SAMPLES];
  uint8_t  checksum;         
};

static Frame frame;  // Data frame to send

// ---------------------- GLOBALS ----------------------
byte misoBuf[2];  // SPI receive buffer
byte inByteArr[2];  // SPI transmit buffer

volatile int pulseCount = 0;
volatile int sampleIndex = 0;

float temperature = 0.0f;
int vDrv = 0;

volatile bool detectedDepth = false;  // Condition flag
volatile uint16_t depthDetectSample = 0;

// Timer for 40kHz Burst
FspTimer burstTimer;

// ---------------------- INTERRUPT HANDLERS ----------------------

// Burst Generation Callback (Replaces Timer1 ISR on R3)
void burstCallback(timer_callback_args_t *) {
  digitalWrite(IO2, !digitalRead(IO2)); // Toggle pin 9
  pulseCount++;
  if (pulseCount >= 32) { // 16 pulses * 2 toggles = 32
    burstTimer.stop();
    pulseCount = 0;  
  }
}

// Echo Detection Interrupt (External Interrupt on Pin 2)
void handleInterrupt() {
  // 硬體中斷：只要偵測到 OUT4 上升緣就記錄
  // 過濾工作由 Loop 中的 BLINDZONE_SAMPLE_END 重置來完成 (與 R3 邏輯一致)
  if (!detectedDepth) {
    depthDetectSample = sampleIndex;
    detectedDepth = true;
  }
}

// ---------------------- HELPER FUNCTIONS ----------------------

// Wrapper to stop burst (compatible with R3 function name)
void stopTransducer() {
  burstTimer.stop();
  pulseCount = 0;
  digitalWrite(IO2, LOW); // Ensure pin is low
}

unsigned int BitShiftCombine(unsigned char x_high, unsigned char x_low) {
  return (x_high << 8) | x_low;  
}

byte parity16(unsigned int val) {
  byte ones = 0;
  for (int i = 0; i < 16; i++) {
    if ((val >> i) & 1) {
      ones++;
    }
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

void sendData() {
  frame.depth_index = depthDetectSample;
  frame.temp_scaled = (int16_t)(DRIVE_FREQUENCY / 1000); 
  // vDrv_scaled is set in loop based on override or vDrv value
  
  // Calculate Checksum
  uint8_t cs = 0;
  cs ^= (uint8_t)(frame.depth_index & 0xFF);
  cs ^= (uint8_t)(frame.depth_index >> 8);
  cs ^= (uint8_t)(frame.temp_scaled & 0xFF);
  cs ^= (uint8_t)(frame.temp_scaled >> 8);
  cs ^= (uint8_t)(frame.vDrv_scaled & 0xFF);
  cs ^= (uint8_t)(frame.vDrv_scaled >> 8);
  for (int i = 0; i < NUM_SAMPLES; i++) {
    cs ^= frame.samples[i];
  }
  frame.checksum = cs;

  const size_t len = 1 + 2 + 2 + 2 + NUM_SAMPLES + 1;
  Serial.write(reinterpret_cast<uint8_t*>(&frame), len);
}

// ---------------------- SETUP & LOOP ----------------------

void setup()
{
  Serial.begin(250000);

  // SPI Setup
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  // Configure GPIOs
  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  pinMode(IO2, OUTPUT);
  pinMode(O4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(O4), handleInterrupt, RISING);

  // Initialize TUSS4470
  tuss4470Write(0x10, FILTER_FREQUENCY_REGISTER);  
  tuss4470Write(0x11, 0x10);                       
  tuss4470Write(0x16, 0xF);  // Enable VDRV (not Hi-Z)
  tuss4470Write(0x1A, 0x0F);  // Set burst pulses to 16
  tuss4470Write(0x17, THRESHOLD_VALUE); // enable threshold detection
  tuss4470Write(0x13, 0x06); // Initial LNA Gain

  // --- R4 Timer Setup (FspTimer) ---
  uint8_t timerType = GPT_TIMER;
  uint8_t channel = FspTimer::get_available_timer(timerType);
  // Frequency * 2 because we toggle the pin (High/Low)
  burstTimer.begin(TIMER_MODE_PERIODIC, timerType, channel, DRIVE_FREQUENCY * 2.0f, 0.0f, burstCallback);
  burstTimer.setup_overflow_irq();
  burstTimer.open();

  // --- R4 ADC Setup (Direct Register Access) ---
  MSTPCRD &= ~(1u << 16); // Enable ADC module
  ADANSA0 = (1u << 9);    // Select channel AN09 (A0) only
  ADCER = 0x0000;         // 12-bit, right align
  ADCSR &= ~(1u << 5);    // Software trigger mode
}

void loop()
{
  // Reset flags for new cycle
  detectedDepth = false; 
  depthDetectSample = 0;

  // Trigger time-of-flight measurement
  tuss4470Write(0x1B, 0x01);

  burstTimer.start();

  // Read analog values from A0
  for (sampleIndex = 0; sampleIndex < NUM_SAMPLES; sampleIndex++) {
    // Start ADC conversion
    ADCSR |= (1u << 15);        
    // Wait for conversion
    while (ADCSR & (1u << 15));  
    
    // Read ADC value (12-bit to 8-bit)
    uint8_t v = ADDR09 >> 4; 
    frame.samples[sampleIndex] = v;

    // [R4 Speed Matching]
    // R4 is much faster than R3. We delay to approximate the ~13.2us sample time 
    // of the R3 so the Python display aspect ratio remains correct.
    delayMicroseconds(11.5); 

    // [Blind Zone Logic]
    // Matches R3 logic: Force reset if we are at the blind zone boundary
    if (sampleIndex == BLINDZONE_SAMPLE_END) {
      detectedDepth = false;
      depthDetectSample = 0;
    }
  }

  // Stop time-of-flight measurement
  tuss4470Write(0x1B, 0x00);
  
  // Software depth override logic (Matches R3)
  #if USE_DEPTH_OVERRIDE
  int overrideSample = 0;
  uint8_t max = 0;
  for (int i = BLINDZONE_SAMPLE_END; i < NUM_SAMPLES; i++) {
    if (frame.samples[i] > max) {
      max = frame.samples[i];
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

  // --- Serial Command Parser (Matches R3) ---
  if (Serial.available() >= 3) {
    if (Serial.read() == 'W') { 
      byte addr = Serial.read();
      byte data = Serial.read();
      stopTransducer(); 
      tuss4470Write(addr, data);
    }
  }

  // Small loop delay
  delay(10);
}