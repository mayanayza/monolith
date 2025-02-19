#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_NeoPixel.h>

//Motor and LED settings
#define LED_COUNT 2
#define MOTOR_COUNT 9
#define MAX_MOTORS_MOVING 2
#define PATTERN_DURATION 30000
#define INITIAL_POSITION 4000 //where to assume the motor will be retracting from during init
#define MAX_POSITION 2500
#define MIN_POSITION 0

//Audio playback and buffer settings
#define SAMPLE_RATE 16000
#define DAC_RESOLUTION 4096
#define DAC_MIDPOINT (DAC_RESOLUTION / 2)
#define AUDIO_MAX_VOLUME 255
#define AUDIO_MIN_VOLUME 0
#define BUFFER_SIZE 2048
#define UPDATE_INTERVAL 1  //ms between updates

// Audio buffer and timer settings
volatile uint32_t isrCount = 0;
volatile uint16_t nextDacValue = 2048;
volatile int audioBufferPos = 0;
volatile bool needsDacUpdate = false;
volatile bool bufferNeedsFill = true;
volatile bool isPlaying = false;
volatile float currentVolume = AUDIO_MAX_VOLUME;
volatile uint8_t* currentBuffer = nullptr;

Adafruit_MCP4725 dac;

void setupTimer5() {
  // Clear timer registers
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;

  // Set CTC mode
  TCCR5B |= (1 << WGM52);

  // Set prescaler to 64 instead of 8 to reduce interrupt frequency
  TCCR5B |= (1 << CS51) | (1 << CS50);

  // Adjust compare value for new prescaler while maintaining ~16kHz
  OCR5A = 250;

  // Enable timer compare interrupt
  TIMSK5 |= (1 << OCIE5A);
}

void setupTimer4() {
  // Reset timer registers
  TCCR4A = 0;
  TCCR4B = 0;
  TIMSK4 = 0;
  TCNT4 = 0;

  // Use prescaler of 1 for maximum precision
  TCCR4B |= (1 << WGM42) | (1 << CS40);

  // Calculate compare value for exact 16kHz
  // 16MHz/16kHz = 1000
  OCR4A = (F_CPU / SAMPLE_RATE) - 1;

  // Enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);
}

ISR(TIMER4_COMPA_vect) {
  if (!isPlaying || currentBuffer == nullptr) {
    return;
  }

  // Simple direct conversion
  uint8_t sample = currentBuffer[audioBufferPos++];
  nextDacValue = ((uint32_t)sample * 4095) / 255;
  needsDacUpdate = true;
}

// Keep Timer5 for motor control only
ISR(TIMER5_COMPA_vect) {
  static unsigned long samples = 0;
  static unsigned long lastPrint = 0;

  samples++;
  if (millis() - lastPrint >= 1000) {
    samples = 0;
    lastPrint = millis();
  }
}

struct Motor {
  const uint8_t pin1;
  const uint8_t pin2;
  const uint8_t sameBoardMotorID;
  const uint8_t ID;
  Adafruit_NeoPixel* led;
  int position = INITIAL_POSITION;
  int targetPosition = 0;
  unsigned long lastStateChange = 0;
  uint8_t ledBrightness = 0;
  bool isStopped = true;
  bool canMove = false;
  bool isMoving = false;
  bool reachedDestination = false;
  bool printedMoveLogForThisTarget = false;

  void init() {
    this->print("Init");
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    led->begin();
    led->show();
  }
  bool reachedTarget() {
    return position == targetPosition;
  }
  void setTargetPosition(unsigned int target) {
    if (target != targetPosition) {
      unsigned long now = millis();
      lastStateChange = now;
      targetPosition = constrain(target, MIN_POSITION, MAX_POSITION);
      printedMoveLogForThisTarget = false;
      this->print("New Target");
    }
  }
  void stop() {
    isStopped = true;
    canMove = false;
    isMoving = false;
    lastStateChange = millis();

    this->setLEDBrightness(0);

    this->print("Stopping");
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
  void allowMove() {
    canMove = true;
    lastStateChange = millis();
  }
  void move() {
    isMoving = true;
    isStopped = false;

    if (!printedMoveLogForThisTarget) {
      printedMoveLogForThisTarget = true;
      this->print("Moving");
    }

    unsigned long now = millis();
    int direction = targetPosition > position ? 1 : -1;

    int incrementalMotorPosition = now - lastStateChange;
    int nextPosition = position + direction * incrementalMotorPosition;
    position = constrain(nextPosition, MIN_POSITION, MAX_POSITION);

    lastStateChange = now;

    if (direction == 1) {
      this->setLEDBrightness(255);
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
    } else if (direction == -1) {
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
    }
  }
  void setLEDBrightness(uint8_t newBrightness) {
    if (ledBrightness != newBrightness) {
      ledBrightness = newBrightness;
      led->setPixelColor(0, led->Color(newBrightness, newBrightness, newBrightness));
      led->setPixelColor(1, led->Color(newBrightness, newBrightness, newBrightness));
      led->show();
    }
  }
  void print(const char* eventName) {
    Serial.print("[");
    Serial.print(ID);
    Serial.print("] ");
    Serial.print(eventName);
    Serial.print("; ");
    Serial.print(position);
    Serial.print("->");
    Serial.print(targetPosition);
    Serial.print(" ");
    Serial.println();
  }

  Motor(uint8_t p1, uint8_t p2, uint8_t ID, uint8_t sameBoardMotorID, Adafruit_NeoPixel* ledObj)
    : pin1(p1), pin2(p2), sameBoardMotorID(sameBoardMotorID), ID(ID), led(ledObj) {}
};
struct MotorMovementPattern {
  int destination;  // Duration in milliseconds for forward movement
  const int minDestination = 150;
  int origin = MIN_POSITION;
  
  int repetitions;
  size_t index = 0;
  
  uint8_t order[MOTOR_COUNT] = { 0, 5, 2, 8, 1, 7, 4, 6, 3 };
  //radial[MOTOR_COUNT] = { 0, 1, 2, 4, 3, 5, 6, 8, 7 };

  struct Orders {
    static const uint8_t radial[MOTOR_COUNT];
    static const uint8_t opposite[MOTOR_COUNT];
  };

  bool isLastMotor(){
    return index == MOTOR_COUNT-1;
  }

  bool completedLastRepetition(){
    return repetitions == 0;
  }

  uint8_t getMotor(){
    return order[index];
  }

  int getDestination(){
    return destination;
  }
  
  void initializePattern() {
    destination = 0;
    repetitions = 1;
    index = 0;
  }

  void generateNewPattern() {
    float r = random(0, 10000) / 10000.0;
    r = r * r;  // Square it to skew distribution
    destination = minDestination + (int)(r * (MAX_POSITION - minDestination));
    repetitions = max(1, (int)(PATTERN_DURATION / (destination * MOTOR_COUNT)));
    index = 0;
  }

  void next() {
    if (isLastMotor()) {
      repetitions--;
      Serial.print(repetitions);
      Serial.println(" repetitions remaining, index back to 0");
      index = 0;
    } else {
      Serial.print("index ");
      Serial.print(index);
      Serial.print(" incrementing to ");
      index++;
      Serial.println(index);
    }
    if (completedLastRepetition()) generateNewPattern();
  }

  MotorMovementPattern() {
    initializePattern();
  }
};

struct AudioTrack {
  enum State {
    IDLE,
    READY_TO_PLAY,
    PLAYING,
    PAUSED,
    FAILED
  };

  static Adafruit_MCP4725* dac;
  char const* fileName;
  unsigned long duration;
  unsigned long playbackStartedTimestamp = 0;
  unsigned long playbackEndedTimestamp = 1;
  unsigned long lastVolumeUpdate = 0;
  File fileHandle;
  State state = IDLE;
  uint8_t buffer[BUFFER_SIZE];

  AudioTrack(const char* fn, unsigned long dur)
    : fileName(fn), duration(dur) {}

  bool shouldPause() {
    return state != PAUSED && currentVolume == 0 && playbackStartedTimestamp > playbackEndedTimestamp;
  }
  bool shouldPlay() {
    return state != PLAYING && playbackStartedTimestamp < playbackEndedTimestamp;
  }

  void preloadFile() {
    fileHandle = SD.open(fileName);
    if (fileHandle) {
      int bytesRead = fileHandle.read(buffer, BUFFER_SIZE);
      if (bytesRead > 0) {
        fileHandle.seek(0);
        currentBuffer = buffer;
        audioBufferPos = 0;
        bufferNeedsFill = false;
        state = READY_TO_PLAY;
      } else {
        Serial.println("Failed to read file data");
        fileHandle.close();
        state = FAILED;
      }
    } else {
      Serial.println("Failed to open file");
      state = FAILED;
    }
  }

  void play() {
    if (state != FAILED) {
      Serial.print("Playing ");
      Serial.println(fileName);
      isPlaying = true;
      state = PLAYING;
      currentVolume = AUDIO_MAX_VOLUME;
    }
  }

  void pause() {
    if (state != FAILED) {
      Serial.print("Pausing ");
      Serial.println(fileName);
      state = PAUSED;
      isPlaying = false;
      playbackEndedTimestamp = millis();
      dac->setVoltage(2048, false);  // Center/silence
    }
  }

  void update() {
    if (state == PLAYING && audioBufferPos >= (BUFFER_SIZE * 3/4)) {
      fillBuffer();
    }
  }

  void fillBuffer() {
    cli();
    int bytesRead = fileHandle.read(buffer, BUFFER_SIZE);
    if (bytesRead > 0) {
      currentBuffer = buffer;
      audioBufferPos = 0;
      bufferNeedsFill = false;
    } else {
      fileHandle.seek(0);
      fillBuffer();
    }
    sei();
  }
};

Adafruit_NeoPixel led_0(LED_COUNT, 47, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_1(LED_COUNT, 41, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_2(LED_COUNT, 30, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_3(LED_COUNT, 49, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_4(LED_COUNT, 35, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_5(LED_COUNT, 39, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_6(LED_COUNT, 32, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_7(LED_COUNT, 33, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_8(LED_COUNT, 37, NEO_GRB + NEO_KHZ800);

Motor motor0(36, 34, 0, 255, &led_0);
Motor motor2(45, 43, 2, 3, &led_2);
Motor motor3(48, 46, 3, 2, &led_3);
Motor motor5(44, 42, 5, 8, &led_5);
Motor motor8(40, 38, 8, 5, &led_8);
Motor motor1(4, 3, 1, 4, &led_1);
Motor motor4(14, 15, 4, 1, &led_4);
Motor motor6(7, 6, 6, 7, &led_6);
Motor motor7(9, 8, 7, 6, &led_7);

Motor m[MOTOR_COUNT] = { motor0, motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8 };

AudioTrack track("onset.raw", 54059);
Adafruit_MCP4725* AudioTrack::dac = nullptr;

MotorMovementPattern pattern;

bool isSameBoardMotorMoving(uint8_t i) {
  uint8_t j = m[i].sameBoardMotorID;
  if (j == 255) return false;  //motor 0 is alone
  else return m[j].isMoving;
}

uint8_t countMotorsMoving() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    if (m[i].isMoving) count = count + 1;
  return count;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  SPI.begin();
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));

  while (!SD.begin()) {
    Serial.println("SD initialization failed. Retrying...");
    delay(1000);
  }
  Serial.println("SD initialization done.");

  while (!dac.begin(0x60)) {
    Serial.println("DAC initialization failed. Retrying...");
  }
  Serial.println("DAC initialized");
  AudioTrack::dac = &dac;

  Serial.println("Resetting motors...");
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    m[i].init();
  }

  Serial.println("Pre-loading audio files...");
  track.preloadFile();
  track.play();

  Serial.println("Initializing timers...");
  setupTimer5();
  setupTimer4();

  sei();

  Serial.println("Setup complete");
}

void loop() {

  if (needsDacUpdate) {
    dac.setVoltage(nextDacValue, false);
    needsDacUpdate = false;
  }
  track.update();

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {

    if (!isSameBoardMotorMoving(i) && countMotorsMoving() < MAX_MOTORS_MOVING && !m[i].canMove) m[i].allowMove();

    if (!m[i].reachedTarget() && m[i].canMove) m[i].move();
    else if (m[i].reachedTarget() && !m[i].isStopped) {
      m[i].stop();

      if (!m[i].reachedDestination) {
        m[i].setTargetPosition(pattern.origin);
        m[i].reachedDestination = true;
        pattern.next();
        uint8_t j = pattern.getMotor();
        m[j].setTargetPosition(pattern.getDestination());
        m[j].reachedDestination = false;
      }
    }
  }
}