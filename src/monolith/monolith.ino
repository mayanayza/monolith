#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_NeoPixel.h>

//Motor and LED settings
#define LED_COUNT 2
#define MOTOR_COUNT 9
#define MAX_MOTORS_MOVING 2
#define INITIAL_POSITION 4000 //where to assume the motor will be retracting from during init
#define MAX_POSITION 2500
#define MIN_POSITION 0

//Audio playback and buffer settings
#define AUDIO_MAX_VOLUME 255
#define AUDIO_MIN_VOLUME 0
#define FADE_DURATION 500  //total duration of volume fade
#define BUFFER_SIZE 2600
#define UPDATE_INTERVAL 1  //ms between updates

const float volumeStepSize = AUDIO_MAX_VOLUME / (FADE_DURATION / UPDATE_INTERVAL);

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
  TCCR4A = 0;
  TCCR4B = 0;
  TIMSK4 = 0;
  TCNT4 = 0;

  // Use prescaler of 1 for maximum precision
  TCCR4B |= (1 << WGM42) | (1 << CS40);

  // 16MHz/16kHz = 1000
  OCR4A = 999;

  TIMSK4 |= (1 << OCIE4A);
}

ISR(TIMER4_COMPA_vect) {

  if (!isPlaying) {
    return;
  }

  uint8_t sample = currentBuffer[audioBufferPos++];
  nextDacValue = ((uint32_t)sample * 4095) / 255;

  needsDacUpdate = true;
  isrCount++;
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

struct Pattern;

template<typename T>
class EntityCycler {
private:
  T* objects;
  size_t count;
  size_t index = 0;
public:
  EntityCycler(T* objArray, size_t objCount)
    : objects(objArray), count(objCount) {}

  T& getCurrent() const {
    return objects[index];
  }
  void next() {
    index = (index + 1) % count;
  }
  bool reachedEnd() {
    return index == count - 1;
  }
  size_t getIndex() {
    return index;
  }
  void resetIndex() {
    index = 0;
  }
};

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
  bool completedPattern = false;
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

struct MotorMovementPattern : public EntityCycler<uint8_t> {
  int destination;
  int origin;  //will return here after reaching destination

  uint8_t getMotorID() {
    return EntityCycler::getCurrent();
  }

  MotorMovementPattern(uint8_t* arr, int destination = MAX_POSITION, int origin = MIN_POSITION)
    : EntityCycler<uint8_t>(arr, MOTOR_COUNT), destination(destination), origin(origin) {}
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
  int8_t fade = 0;
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
      playbackStartedTimestamp = millis();
      startFadeIn();
    }
  }

  void startFadeOut() {
    fade = state != FAILED ? -1 : 0;
  }

  void startFadeIn() {
    fade = state != FAILED ? 1 : 0;
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
    if (state != FAILED) {
      static unsigned long lastUpdate = 0;

      unsigned long now = millis();
      if (now - lastUpdate < UPDATE_INTERVAL) {
        return;
      }
      lastUpdate = now;

      if (fade == 1 && currentVolume < AUDIO_MAX_VOLUME) {
        currentVolume = min(currentVolume + volumeStepSize, (float)AUDIO_MAX_VOLUME);
        if (currentVolume >= AUDIO_MAX_VOLUME) { fade = 0; }
      } else if (fade == -1 && currentVolume > AUDIO_MIN_VOLUME) {
        currentVolume = min(currentVolume - volumeStepSize, (float)AUDIO_MIN_VOLUME);
        if (currentVolume >= AUDIO_MIN_VOLUME) { fade = 0; }
      }

      if (state == PLAYING && audioBufferPos >= (BUFFER_SIZE * 3 / 4)) {
        fillBuffer();
      }
    }
  }

  void fillBuffer() {
    cli();

    SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
    int bytesRead = fileHandle.read(buffer, BUFFER_SIZE);
    SPI.endTransaction();

    if (bytesRead > 0) {
      currentBuffer = buffer;
      audioBufferPos = 0;
      bufferNeedsFill = false;
    } else {
      // Loop track
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

static const uint8_t PATTERN_COUNT = 32;
uint8_t radial[MOTOR_COUNT] = { 0, 1, 2, 4, 3, 5, 6, 8, 7 };
uint8_t opposites[MOTOR_COUNT] = { 0, 5, 2, 8, 1, 7, 4, 6, 3 };

MotorMovementPattern& getNextMotorMovementPattern();

MotorMovementPattern retractAll(radial, MIN_POSITION, MIN_POSITION);
MotorMovementPattern full(radial);
MotorMovementPattern half(radial, MAX_POSITION/2);
MotorMovementPattern quarter(opposites, MAX_POSITION/4);
MotorMovementPattern eighth(opposites, MAX_POSITION/8);
MotorMovementPattern sixteenth(opposites, MAX_POSITION/16);
MotorMovementPattern p[PATTERN_COUNT] = { 
  retractAll, 
  full, 
  half,
  half,
  quarter,
  quarter, 
  quarter, 
  quarter,
  eighth, 
  eighth, 
  eighth,
  eighth, 
  eighth, 
  eighth, 
  eighth,
  eighth,
  sixteenth, 
  sixteenth, 
  sixteenth,
  sixteenth, 
  sixteenth, 
  sixteenth, 
  sixteenth,
  sixteenth, 
  sixteenth, 
  sixteenth, 
  sixteenth,
  sixteenth, 
  sixteenth, 
  sixteenth, 
  sixteenth,
  sixteenth
  };
EntityCycler<MotorMovementPattern> patterns(p, PATTERN_COUNT);

Adafruit_MCP4725* AudioTrack::dac = nullptr;
static const uint8_t AUDIO_COUNT = 2;
AudioTrack au[AUDIO_COUNT] = {
  AudioTrack("onset.raw", 54059),
  AudioTrack("upset.raw", 77089)
};
EntityCycler<AudioTrack> tracks(au, AUDIO_COUNT);

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

MotorMovementPattern& getNextMotorMovementPattern() {

  if (patterns.getCurrent().reachedEnd()) {
    patterns.next();
    if (patterns.getIndex() == 0) {
      patterns.next();
    }
    // Reset the motor index when switching patterns
    patterns.getCurrent().resetIndex();  // Add this line
  } else {
    patterns.getCurrent().next();
  }
  
  Serial.print("Pattern ");
  Serial.print(patterns.getIndex());
  Serial.print(" on step ");
  Serial.println(patterns.getCurrent().getIndex());

  return patterns.getCurrent();
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
  for (int i = 0; i < AUDIO_COUNT; i++) {
    au[i].preloadFile();
  }

  Serial.println("Initializing timers...");
  setupTimer5();
  setupTimer4();

  sei();

  Serial.println("Setup complete");
}

void loop() {

  AudioTrack& track = tracks.getCurrent();
  MotorMovementPattern& mp = patterns.getCurrent();

  if (needsDacUpdate) {
    dac.setVoltage(nextDacValue, false);
    needsDacUpdate = false;
  }
  if (track.shouldPlay()) {
    track.play();
  } else if (track.shouldPause()) {
    track.pause();
    tracks.next();
  }
  track.update();

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {

    if (!isSameBoardMotorMoving(i) && countMotorsMoving() < MAX_MOTORS_MOVING && !m[i].canMove) m[i].allowMove();

    if (!m[i].reachedTarget() && m[i].canMove) m[i].move();
    else if (m[i].reachedTarget() && !m[i].isStopped) {
      m[i].stop();

      if (!m[i].completedPattern) {
        m[i].setTargetPosition(mp.origin);
        m[i].completedPattern = true;
        mp = getNextMotorMovementPattern();
        uint8_t j = mp.getMotorID();
        m[j].setTargetPosition(mp.destination);
        m[j].completedPattern = false;
      }
    }
  }
}