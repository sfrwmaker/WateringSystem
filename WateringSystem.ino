// The WATERER watering system. Using LCD1602@nano
#include <LiquidCrystal.h>
#include <Time.h>
#include <EEPROM.h>

const float IN_STRAY_CAP_TO_GND = 24.48;
const float IN_CAP_TO_GND       = IN_STRAY_CAP_TO_GND;
const float R_PULLUP            = 34.8;  
const int MAX_ADC_VALUE         = 1023;

// The LCD 1602 parallel interface
const byte LCD_RS_PIN     = 12;
const byte LCD_E_PIN      = 11;
const byte LCD_DB4_PIN    = 5;
const byte LCD_DB5_PIN    = 4;
const byte LCD_DB6_PIN    = 3;
const byte LCD_DB7_PIN    = 13;
const byte LCD_BLGHT_PIN  = 6;
const byte LIGHT_SENSOR   = A7;

// The sensors PINs
const byte S1A_PIN = A0;
const byte S1B_PIN = A1;
const byte S2A_PIN = A2;
const byte S2B_PIN = A3;

// Motor drives
const byte M_FORWARD  = A4;
const byte M_BACKWARD = A5;
const byte M_A_PWM    = 9;
const byte M_B_PWM    = 10;

// The Rotary encoder
const byte R_MAIN_PIN = 2;                          // Rotary Encoder main pin (right)
const byte R_SECD_PIN = 8;                          // Rotary Encoder second pin (left)
const byte R_BUTN_PIN = 7;                          // Rotary Encoder push button pin

// Global function declaration
void applyConfig(void);

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
  uint32_t ID                           each time increment by 1
  struct cfg                            config data, 18 bytes
  byte CRC                              the checksum
*/
struct cfg {
  bool     pump_enabled[2];                         // Weither the pump is enabled
  bool     pump_smooth[2];                          // no longer used
  uint16_t dry[2];                                  // The dry threshold for pumps
  uint16_t fwd_time[2];                             // The time to run pump forward, tenth of seconds
  uint16_t bcw_time[2];                             // The time to run pump backward, tenth of seconds
  byte     backlight_nightly;                       // The nightly display brightness
  byte     backlight_manual;                        // Manually set up brightness
  bool     backlight_auto;                          // Weither automatically adjust display brightness
  byte     backlight_morning;                       // The hour when morning begins
  byte     backlight_evening;                       // The hour when night begin
};

class CONFIG {
  public:
    CONFIG() {
      can_write = is_valid = false;
      buffRecords = 0;
      rAddr = wAddr = 0;
      eLength = 0;
      nextRecID = 0;
    }
    void init();
    bool load(void);
    bool isValid(void)            { return is_valid; }
    void getConfig(struct cfg &Cfg);                // Copy config structure from this class
    void updateConfig(struct cfg &Cfg);             // Copy updated config into this class
    bool saveConfig(struct cfg &Cfg);               // write updated config into the EEPROM
    
  private:
    void defaultConfig(void);
    struct cfg Config;
    bool readRecord(uint16_t addr, uint32_t &recID);
    bool save(void);
    bool can_write;                                 // The flag indicates that data can be saved
    bool is_valid;                                  // Weither tha data was loaded
    byte buffRecords;                               // Number of the records in the outpt buffer
    uint16_t rAddr;                                 // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                                 // Address in the EEPROM to start write new record
    uint16_t eLength;                               // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                             // next record ID
    const byte record_size = 32;                    // The size of one record in bytes
};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
  defaultConfig();
  eLength = EEPROM.length();
  byte t, p ,h;
  uint32_t recID;
  uint32_t minRecID = 0xffffffff;
  uint16_t minRecAddr = 0;
  uint32_t maxRecID = 0;
  uint16_t maxRecAddr = 0;
  byte records = 0;

  nextRecID = 0;

  // read all the records in the EEPROM find min and max record ID
  for (uint16_t addr = 0; addr < eLength; addr += record_size) {
    if (readRecord(addr, recID)) {
      ++records;
      if (minRecID > recID) {
        minRecID = recID;
        minRecAddr = addr;
      }
      if (maxRecID < recID) {
        maxRecID = recID;
        maxRecAddr = addr;
      }
    } else {
      break;
    }
  }

  if (records == 0) {
    wAddr = rAddr = 0;
    can_write = true;
    return;
  }

  rAddr = maxRecAddr;
  if (records < (eLength / record_size)) {          // The EEPROM is not full
    wAddr = rAddr + record_size;
    if (wAddr > eLength) wAddr = 0;
  } else {
    wAddr = minRecAddr;
  }
  can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
  memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
  memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
  updateConfig(Cfg);
  return save();                                     // Save new data into the EEPROM
}

bool CONFIG::save(void) {
  if (!can_write) return can_write;
  if (nextRecID == 0) nextRecID = 1;

  uint16_t startWrite = wAddr;
  uint32_t nxt = nextRecID;
  byte summ = 0;
  for (byte i = 0; i < 4; ++i) {
    EEPROM.write(startWrite++, nxt & 0xff);
    summ <<=2; summ += nxt;
    nxt >>= 8;
  }
  byte* p = (byte *)&Config;
  for (byte i = 0; i < sizeof(struct cfg); ++i) {
    summ <<= 2; summ += p[i];
    EEPROM.write(startWrite++, p[i]);
  }
  summ ++;                                            // To avoid empty records
  EEPROM.write(wAddr+record_size-1, summ);

  rAddr = wAddr;
  wAddr += record_size;
  if (wAddr > EEPROM.length()) wAddr = 0;
  return true;
}

bool CONFIG::load(void) {

  is_valid = readRecord(rAddr, nextRecID);
  nextRecID ++;
  if (!is_valid) defaultConfig();
  return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
  byte Buff[record_size];

  for (byte i = 0; i < record_size; ++i) 
    Buff[i] = EEPROM.read(addr+i);
  
  byte summ = 0;
  for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {

    summ <<= 2; summ += Buff[i];
  }
  summ ++;                                              // To avoid empty fields
  if (summ == Buff[record_size-1]) {                    // Checksumm is correct
    uint32_t ts = 0;
    for (char i = 3; i >= 0; --i) {
      ts <<= 8;
      ts |= Buff[i];
    }
    recID = ts;
    byte i = 4;
    memcpy(&Config, &Buff[4], sizeof(struct cfg));
    return true;
  }
  return false;
}

void CONFIG::defaultConfig(void) {
  Config.pump_enabled[0]   = true;
  Config.pump_enabled[1]   = false;
  Config.dry[0]            = 600;
  Config.dry[1]            = 600;
  Config.fwd_time[0]       = 70;
  Config.fwd_time[1]       = 40;
  Config.bcw_time[0]       = 20;
  Config.bcw_time[1]       = 20;
  Config.backlight_nightly = 30;
  Config.backlight_auto    = true;
  Config.backlight_manual  = 128;
  Config.backlight_morning = 8;
  Config.backlight_evening = 23; 
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 8
class HISTORY {
  public:
    HISTORY(void) { len = 0; }
    void init(void) { len = 0; }
    void put(int item) {
      if (len < H_LENGTH) {
        queue[len++] = item;
      } else {
        for (byte i = 0; i < len-1; ++i) queue[i] = queue[i+1];
        queue[H_LENGTH-1] = item;
      }
    }
    bool isFull(void) { return len == H_LENGTH; }
    int top(void) { return queue[0]; }
    int average(void);
    float dispersion(void);
    float gradient(void);

  private:
    int queue[H_LENGTH];
    byte len;
};

int HISTORY::average(void) {
  long sum = 0;
  if (len == 0) return 0;
  if (len == 1) return queue[0];
  for (byte i = 0; i < len; ++i) sum += queue[i];
  sum += len >> 1;                                  // round the average
  sum /= len;
  return (int)sum;
}

float HISTORY::dispersion(void) {
  if (len < 3) return 1000;
  long sum = 0;
  long avg = average();
  for (byte i = 0; i < len; ++i) {
    long q = queue[i];
    q -= avg;
    q *= q;
    sum += q;
  }
  sum += len << 1;
  float d = (float)sum / (float)len;
  return d;
}

// approximating the history with the line (y = ax+b) using method of minimum square. Gradient is parameter 'a'
float HISTORY::gradient(void) {
  if (len < 2) return 0;
  long sx, sx_sq, sxy, sy;
  sx = sx_sq = sxy = sy = 0;
  for (byte i = 1; i <= len; ++i) {
    sx    += i;
  sx_sq += i*i;
  sxy   += i*queue[i-1];
  sy    += queue[i-1];
  }
  long numerator   = len * sxy - sx * sy;
  long denominator = len * sx_sq - sx * sx;
  float a = (float)numerator / (float)denominator;
  return a;
}

//------------------------------------------ backlight of the LCD display --------------------------------------
class BACKLIGHT {
  public:
    BACKLIGHT(byte sensorPIN, byte lightPIN) {
      sensor_pin = sensorPIN;
      led_pin    = lightPIN;
    }
    void init(void);                                // Initialize the data
    void adjust(void);                              // Automatically adjust the brightness
    void setBrightness(byte b);                     // Manually set brightness, turn off automatic adjustment
    void turnAuto(bool a);                          // Turn on/off automatic adjustment
    void setNightPeriod(byte hEvening, byte hMorning, byte br = 0, bool lt = true);
    bool isDark(void);                              // Weither it is night time
    int  getSensorValue(void)       { return analogRead(sensor_pin); }
  
  private:
    byte sensor_pin;                                // Light sensor pin
    byte led_pin;                                   // Led PWM pin
    uint32_t checkMS;                               // Time in ms when the sensor was checked
    bool automatic;                                 // Weither the backlight should be adjusted automatically
    bool use_local_time;                            // Weither to use local time to switch off the light nightly
    byte brightness;                                // The backlight brightness
    byte new_brightness;                            // The baclight brightness to set up
    byte evening, morning;                          // The hour of eveniung (to switch off backlight) and morning
    byte nightly_brightness;                        // The brightness to use nightly
    HISTORY hLight;                                 // Statistics of the sensor values
    const byte default_brightness = 128;            // Default brightness of backlight
    const byte dayly_brightness = 150;              // Dayly brightness of backlight
    const uint16_t b_night = 400;                   // light sensor value of the night
    const uint16_t b_day = 800;                     // light sensor value of the day light
    const uint16_t period = 200;                    // The period in ms to check the photeregister
    const byte max_dispersion = 15;                 // The maximum dispersion of the sensor to change the brightness 
};

void BACKLIGHT::init(void) {

  pinMode(led_pin, OUTPUT);
  pinMode(sensor_pin, INPUT);
  brightness = new_brightness = default_brightness;
  analogWrite(led_pin, brightness);
  checkMS = 0;
  automatic = true;
  nightly_brightness = 50;
  use_local_time = false;
  evening = morning = 0;                            // This value will be overwritten by config
  adjust();
  hLight.init();
}

void BACKLIGHT::adjust(void) {

  if (!automatic) return;

  if (new_brightness != brightness) {
    if (new_brightness > brightness) ++brightness; else --brightness;
    analogWrite(led_pin, brightness);
    delay(5);
  }

  if (millis() < checkMS) return;
  checkMS = millis() + period;
 
  // Turn off the backlight at night
  if (use_local_time) {
    byte h = hour();
    if ((h <= morning) || (h >= evening)) {
      new_brightness = nightly_brightness;
      return;
    }
  }
 
  int light = analogRead(sensor_pin);
  hLight.put(light);
  light = hLight.average();
  if (light < b_night) {
    new_brightness = nightly_brightness;
    return;
  }

  if (light > b_day) {
    new_brightness = 0;
    return;
  }

  new_brightness = map(light, b_night, b_day, nightly_brightness, dayly_brightness);
}

void BACKLIGHT::setBrightness(byte b) {
  brightness = b;
  automatic = false;
  analogWrite(led_pin, brightness);
}

void BACKLIGHT::turnAuto(bool a) {
  automatic = a;
  checkMS = 0;
  if (a)
    adjust();    
}

void BACKLIGHT::setNightPeriod(byte hEvening, byte hMorning, byte br, bool lt) {
  if (hEvening <= hMorning) return;
  if (hEvening > 23) return;
  morning = hMorning;
  evening = hEvening;
  nightly_brightness = br;
  use_local_time = lt;
}

bool BACKLIGHT::isDark(void) {

  if (use_local_time) {
    byte h = hour();
    return ((h <= morning) || (h >= evening));
  }

  long light = 0;
  for (byte i = 0; i < 4; ++i) {
    light += analogRead(sensor_pin);
    delay(200);
  }
  light >>= 2;
  return (light < b_night);
}

//------------------------------------------ class BUTTON ------------------------------------------------------
class BUTTON {
  public:
    BUTTON(byte ButtonPIN, unsigned int timeout_ms = 3000) {
      pt = tickTime = 0;
      buttonPIN = ButtonPIN;
      overPress = timeout_ms;
    }
    void init(void) { pinMode(buttonPIN, INPUT_PULLUP); }
    void setTimeout(uint16_t timeout_ms = 3000) { overPress = timeout_ms; }
    byte intButtonStatus(void) { byte m = mode; mode = 0; return m; }
    void cnangeINTR(void);
    byte buttonCheck(void);
    bool buttonTick(void);
  private:
    volatile byte mode;                             // The button mode: 0 - not pressed, 1 - pressed, 2 - long pressed
    const uint16_t tickTimeout = 200;               // Period of button tick, while tha button is pressed 
    const uint16_t shortPress = 900;                // If the button was pressed less that this timeout, we assume the short button press
    uint16_t overPress;                             // Maxumum time in ms the button can be pressed
    volatile uint32_t pt;                           // Time in ms when the button was pressed (press time)
    uint32_t tickTime;                              // The time in ms when the button Tick was set
    byte buttonPIN;                                 // The pin number connected to the button
};

void BUTTON::cnangeINTR(void) {                     // Interrupt function, called when the button status changed
  
  bool keyUp = digitalRead(buttonPIN);
  unsigned long now_t = millis();
  if (!keyUp) {                                     // The button has been pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t; 
  } else {
    if (pt > 0) {
      if ((now_t - pt) < shortPress) mode = 1;      // short press
        else mode = 2;                              // long press
      pt = 0;
    }
  }
}

byte BUTTON::buttonCheck(void) {                    // Check the button state, called each time in the main loop

  mode = 0;
  bool keyUp = digitalRead(buttonPIN);              // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp) {                                     // The button is pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t;
  } else {
    if (pt == 0) return 0;
    if ((now_t - pt) > shortPress)                  // Long press
      mode = 2;
    else
      mode = 1;
    pt = 0;
  } 
  return mode;
}

bool BUTTON::buttonTick(void) {                     // When the button pressed for a while, generate periodical ticks

  bool keyUp = digitalRead(buttonPIN);              // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp && (now_t - pt > shortPress)) {        // The button have been pressed for a while
    if (now_t - tickTime > tickTimeout) {
       tickTime = now_t;
       return (pt != 0);
    }
  } else {
    if (pt == 0) return false;
    tickTime = 0;
  } 
  return false;
}

//------------------------------------------ class ENCODER ------------------------------------------------------
class ENCODER {
  public:
    ENCODER(byte aPIN, byte bPIN, int16_t initPos = 0) {
      pt = 0; mPIN = aPIN; sPIN = bPIN; pos = initPos;
      min_pos = -32767; max_pos = 32766; channelB = false; increment = 1;
      changed = 0;
      is_looped = false;
    }
    void init(void) {
      pinMode(mPIN, INPUT_PULLUP);
      pinMode(sPIN, INPUT_PULLUP);
    }
    void reset(int16_t initPos, int16_t low, int16_t upp, byte inc = 1, byte fast_inc = 0, bool looped = false) {
      min_pos = low; max_pos = upp;
      if (!write(initPos)) initPos = min_pos;
      increment = fast_increment = inc;
      if (fast_inc > increment) fast_increment = fast_inc;
      is_looped = looped;
    }
    void set_increment(byte inc) { increment = inc; }
    byte get_increment(void) { return increment; }
    bool write(int16_t initPos) {
      if ((initPos >= min_pos) && (initPos <= max_pos)) {
        pos = initPos;
        return true;
      }
      return false;
    }
    int16_t read(void) { return pos; }
    void cnangeINTR(void);
  private:
    const uint16_t overPress = 1000;
    int32_t min_pos, max_pos;
    volatile uint32_t pt;                           // Time in ms when the encoder was rotaded
    volatile uint32_t changed;                      // Time in ms when the value was changed
    volatile bool channelB;
    volatile int32_t pos;                           // Encoder current position
    byte mPIN, sPIN;                                // The pin numbers connected to the main channel and to the socondary channel
    bool is_looped;                                 // Weither the encoder is looped
    byte increment;                                 // The value to add or substract for each encoder tick
    byte fast_increment;                            // The value to change encoder when in runs quickly
    const uint16_t fast_timeout = 300;              // Time in ms to change encodeq quickly
};

void ENCODER::cnangeINTR(void) {                    // Interrupt function, called when the channel A of encoder changed
  
  bool rUp = digitalRead(mPIN);
  unsigned long now_t = millis();
  if (!rUp) {                                       // The channel A has been "pressed"
    if ((pt == 0) || (now_t - pt > overPress)) {
      pt = now_t;
      channelB = digitalRead(sPIN);
    }
  } else {
    if (pt > 0) {
      byte inc = increment;
      if ((now_t - pt) < overPress) {
        if ((now_t - changed) < fast_timeout) inc = fast_increment;
        changed = now_t;
        if (channelB) pos -= inc; else pos += inc;
        if (pos > max_pos) { 
          if (is_looped)
            pos = min_pos;
          else 
            pos = max_pos;
        }
        if (pos < min_pos) {
          if (is_looped)
            pos = max_pos;
          else
            pos = min_pos;
        }
      }
      pt = 0; 
    }
  }
}

//------------------------------------------ Humidity sensor ---------------------------------------------------
class SENSOR {
  public:
    SENSOR(byte aPIN, byte bPIN) {
      sA = aPIN;
      sB = bPIN;
      checked = 0;
  }
  void init(void);
  uint16_t getHumidity(void);                       // ln(capacity) * 100

  private:
    byte sA, sB;                                    // The pins used by the sensor
    uint32_t checked;                               // The time in ms when the sensor was read
};

void SENSOR::init(void) {
  pinMode(sA, OUTPUT);
  pinMode(sB, OUTPUT);
  digitalWrite(sA, LOW);
  digitalWrite(sB, LOW);
  checked = 0;
}

uint16_t SENSOR::getHumidity(void) {

  pinMode(sA, INPUT);
  digitalWrite(sB, HIGH);
  int val = analogRead(sA);
  digitalWrite(sB, LOW);

  float capacitance = 0.0;

  if (val < 1000) {
    pinMode(sA, OUTPUT);
    capacitance = (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);
  } else {
    pinMode(sA, OUTPUT);
    delay(1);
    pinMode(sB, INPUT_PULLUP);
    unsigned long u1 = micros();
    unsigned long t;
    int digVal;

    do {
      digVal = digitalRead(sB);
      unsigned long u2 = micros();
      t = u2 > u1 ? u2 - u1 : u1 - u2;
    } while ((digVal < 1) && (t < 400000L));

    pinMode(sB, INPUT); 
    val = analogRead(sB);
    digitalWrite(sA, HIGH);
    int dischargeTime = (int)(t / 1000L) * 5;
    delay(dischargeTime);   
    pinMode(sB, OUTPUT); 
    digitalWrite(sB, LOW);
    digitalWrite(sB, LOW);
    capacitance = -(float)t / R_PULLUP
                              / log(1.0 - (float)val / (float)MAX_ADC_VALUE);
    capacitance *= 1000.0;
  }
  while (millis() % 1000 != 0);

  checked = millis();
  uint32_t cap = round(capacitance);
  cap |= 0x1f;                                        // The error of capacitance should be rounded, fill up last 5 bits
  capacitance = (float)cap;
  float humidity = log(capacitance) * 100.0;
  uint16_t h = round(humidity);
  return h;
}

//------------------------------------------ Water Pump driver -------------------------------------------------
class PUMP {
  public:
    PUMP(byte fwdPIN, byte bcwPIN, byte pwmPIN) {
    forward_pin  = fwdPIN;
    backward_pin = bcwPIN;
    pwm_pin      = pwmPIN;
  }
  void init(void);
  void stop(void);
  void forward(uint16_t ds,  bool smooth = false);  // Run the motor forward for ms tenth of second
  void backward(uint16_t ds, bool smooth = false);  // Run the motor backward for ms tenth of second
    
  private:
    byte backward_pin;                              // To run the motor clockwise
    byte forward_pin;                               // To run the motor contrclockwise
    byte pwm_pin;                                   // Motor enable pin, PWM
    void runSmoothly(void);                         // Start the motor smoothly
    const byte start_power = 200;                   // Minimum PWM power to the motor
};

void PUMP::init(void) {
  pinMode(forward_pin,  OUTPUT);
  pinMode(backward_pin, OUTPUT);
  pinMode(pwm_pin,      OUTPUT);
  stop();
}

void PUMP::stop(void) {
  digitalWrite(pwm_pin,      LOW);
  digitalWrite(forward_pin,  LOW);
  digitalWrite(backward_pin, LOW);
}

void PUMP::runSmoothly(void) {
  for (uint16_t i = start_power; i < 255; i += 10) {
    analogWrite(pwm_pin, i);
    delay(100);
  }
}

void PUMP::forward(uint16_t ds,  bool smooth) {
  char buff[6];

  digitalWrite(pwm_pin, LOW);                       // First, stop the motor
  digitalWrite(forward_pin,  HIGH);
  digitalWrite(backward_pin, LOW);
  if (smooth) runSmoothly();
  digitalWrite(pwm_pin, HIGH);
  if (ds > 0) {
    delay(ds * 100);
    stop();
  }
}

void PUMP::backward(uint16_t ds,  bool smooth) {

  digitalWrite(pwm_pin, LOW);                       // First, stop the motor
  digitalWrite(forward_pin,  LOW);
  digitalWrite(backward_pin, HIGH);
  if (smooth) runSmoothly();
  digitalWrite(pwm_pin, HIGH);
  delay(ds * 100);
  stop();
}

//------------------------------------------ Waterer (Sensor + motor pump) --------------------------------------
class WATERER: protected SENSOR, PUMP {
  public:
    WATERER(byte s_aPIN, byte s_bPIN, byte m_fwdPIN, byte m_bcwPIN, byte m_pwmPIN):
        SENSOR(s_aPIN, s_bPIN), PUMP(m_fwdPIN, m_bcwPIN, m_pwmPIN) {
    enabled = false;
    }
    void init(void);                                // Initialize
    bool needRain(uint16_t h = 0);                  // Check the humidity
    bool rain(void);                                // Start the motor, do not check the humidity
    void setParams(uint16_t  fwdTime, uint16_t  bcwTime);
    bool startManual(void);                         // Manual watering
    bool finishManual(uint16_t bcwTime = 0);
    uint16_t getHumidity(void)            { return SENSOR::getHumidity(); }
    void     activate(bool e)             { enabled = e; }
    bool     isActive(void)               { return enabled; }
    void     setDryThreshold(uint16_t l)  { dry_threshold = l; }
    uint16_t getDryThreshold(void)        { return dry_threshold; }
    uint16_t getFwdTime(void)             { return fwd_time; }
    uint16_t getBcwTime(void)             { return bcw_time; }

  private:
    LiquidCrystal* pD;                              // The pointer to the screen to animate the process
    bool enabled;                                   // Weither this WATERER exists
    uint16_t dry_threshold;                         // Humidity limit to start watering the flower
    uint16_t fwd_time;                              // Time in tenth of the second to run motor forward
    uint16_t bcw_time;                              // Time in tenth of the second to run motor backward
    const uint16_t max_time = 600;                  // Maximum value for run motor time in tenth of the second
    const uint16_t min_time = 10;                   // Minimum value for run motor time in tenth of the second
};

void WATERER::init(void) {
  SENSOR::init();
  PUMP::init();
}

bool WATERER::needRain(uint16_t h) {

  if (!enabled) return false;

  if (h == 0)
    h = getHumidity();
  return (h <= dry_threshold);
}

bool WATERER::rain(void) {

  if (enabled) {
    forward(fwd_time);
    delay(500);
    backward(bcw_time);
  }
  return enabled;
}

bool WATERER::startManual(void) {

  if (enabled)
    forward(0);
  return enabled;
}

bool WATERER::finishManual(uint16_t bcwTime) {

  if (enabled) {
    if (bcwTime == 0) bcwTime = bcw_time;
    backward(bcwTime);
  }
  return enabled;
}

void WATERER::setParams(uint16_t fwdTime, uint16_t bcwTime) {

  fwd_time = min_time;
  bcw_time = min_time << 1;
  
  if (fwdTime > max_time)
    fwd_time = max_time;
  else
    if (fwdTime > min_time)
      fwd_time = fwdTime;

  if (bcwTime > max_time)
    bcw_time = max_time;
  else
    if (bcwTime > min_time)
      bcw_time = bcwTime;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
  public:
    SCREEN* next;                                   // Pointer to the next screen
    SCREEN* nextL;                                  // Pointer to the next Level screen, usually, setup
    SCREEN* main;                                   // Pointer to the main screen

    SCREEN() {
      next = nextL = main = 0;
      force_redraw = true;
      scr_timeout = 0;
      time_to_return = 0;
    }
    virtual void init(byte param = 255) { }
    virtual void show(void) { }
    virtual SCREEN* menu(void) {if (this->next != 0) return this->next; else return this; }
    virtual SCREEN* menu_long(void) { if (this->nextL != 0) return this->nextL; else return this; }
    virtual void rotaryValue(int16_t value) { }
    bool isSetup(void){ return (scr_timeout != 0); }
    void forceRedraw(void) { force_redraw = true; }
    SCREEN* returnToMain(void) {
      if (main && (scr_timeout != 0) && (millis() >= time_to_return)) {
        scr_timeout = 0;
        return main;
      }
      return this;
    }
    void resetTimeout(void) {
      if (scr_timeout > 0)
        time_to_return = millis() + scr_timeout*1000;
    }
    void setSCRtimeout(uint16_t t) {
      scr_timeout = t;
      resetTimeout(); 
    }
  protected:
    bool force_redraw;
    uint16_t scr_timeout;                             // Timeout is sec. to return to the main screen, canceling all changes
    uint32_t time_to_return;                          // Time in ms to return to main screen
};

//---------------------------------------- class main creen ----------------------------------------------------
class mainSCREEN : public SCREEN {
  public:
    mainSCREEN(LiquidCrystal* LCD, ENCODER* ENC, BACKLIGHT* BCKL, WATERER* pF[2]) {
      update_screen = 0;
      pD = LCD;
      pEnc = ENC;
	    pBcl = BCKL;
      pFlower[0] = pF[0];
      pFlower[1] = pF[1];
    }
    virtual void init(byte param);
    virtual SCREEN* menu(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    
  private:
	void autoRain(void);                              // automatically rain the flowers 
    LiquidCrystal* pD;                                // Pointer to the display instance
    ENCODER*       pEnc;                              // Pointer to the rotary encoder instance
	  BACKLIGHT*     pBcl;
    WATERER*       pFlower[2];
    uint32_t       update_screen;                     // Time in ms to update the display
    uint32_t       update_humidity;                   // Time in ms to update humidity
    uint16_t       humidity[2];                       // The sensor humidity;
    uint16_t       h_before[2];                       // The sensor humidity before waterer was running
    uint32_t       last_watered[2];                   // The time in ms when the waterer was activated
    bool           need_to_be_checked[2];             // Weither the waterer need to be checked after run
	  bool           is_dark;                           // Weither is night nime
    const uint32_t waterer_timeout = 300000;          // waterer check period after it was run
    const uint32_t chill_timeout   = 100000;          // The timeout to cool the L298n CI
    const uint16_t period          = 10000;           // Period to refresh the screen
    const uint32_t humidity_period = 600000;          // The Period to check the humidity
};

void mainSCREEN::init(byte param) {
  pD->clear();
  update_screen = update_humidity = 0;
  for (byte i = 0; i < 2; ++i) {
    humidity[i] = h_before[i] = 0;
	  last_watered[i] = 0;
	  need_to_be_checked[i] = false;
  }
  is_dark = false;
  pEnc->reset(0, 0, 1, 1, 0, true);
  forceRedraw();
}

SCREEN* mainSCREEN::menu(void) {
  // Check is there some flower is enabled
  bool ok = false;
  for (byte i = 0; i < 2; ++i) {
    if (pFlower[i]->isActive()) {
      ok = true;
      break;
    }
  }
  if (!ok) {
    pD->clear();
    pD->print(F("No active waterer"));
    update_screen = millis() + 5000;
  }
  if (ok && this->next != 0) return this->next; else return this;
}

void mainSCREEN::rotaryValue(int16_t value) {
  forceRedraw();
  pD->clear();
}

void mainSCREEN::show(void) {
  char buff[17];
  byte cur_pos[2] = {12, 9};
  
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + period;
  
  autoRain();

  int16_t mode = pEnc->read();
  for (byte i = 0; i < 2; ++i) {
    byte curr = 0;
    if (i == 1) curr = cur_pos[mode];
    if (pFlower[i]->isActive()) {
      if (mode) {
        uint16_t fwdTime = pFlower[i]->getFwdTime();
        uint16_t bcwTime = pFlower[i]->getBcwTime();
        sprintf(buff, "f: %2d.%d", fwdTime/10, fwdTime%10);
        pD->setCursor(curr, 0);
        pD->print(buff);
        sprintf(buff, "b: %2d.%d", bcwTime/10, bcwTime%10);
        pD->setCursor(curr, 1);
        pD->print(buff);
      } else {
        sprintf(buff, "%4d", humidity[i]);
        pD->setCursor(curr, 0);
        pD->print(buff);
        uint16_t dry = pFlower[i]->getDryThreshold();
        sprintf(buff, "%4d", dry);
        pD->setCursor(curr, 1);
        pD->print(buff);
        pD->setCursor(5, 1);
        pD->print(F("<limt>"));
      }
    } else {
      pD->setCursor(curr, 0);
      pD->print(F("xxxx"));
      pD->setCursor(curr, 1);
      pD->print(F("xxxx"));
    }
  }
  
  if (!mode) {                                        // Print minutes to next probe
    if (is_dark) {
	    pD->setCursor(6, 0);
	    pD->print(F("    "));
	  } else {
	    uint32_t next_probe = millis();
	    if (next_probe < update_humidity) {
	      next_probe = update_humidity - next_probe + 999;
	      next_probe /= 1000;                           // seconds
	    } else {
	      next_probe = 0;
	    }
	    sprintf(buff, "%3ds", next_probe);
	    pD->setCursor(6, 0);
	    pD->print(buff);
	  }
  }
  
  is_dark = pBcl->isDark();
}

void mainSCREEN::autoRain(void) {

  uint32_t now_t = millis();
  for (byte i = 0; i < 2; ++i) {                      // First, check the waterer
    if ((last_watered[i]) && (pFlower[i]->isActive() )) {  // If the flower was watered, check that the humidity increased
      if ((need_to_be_checked[i]) && (now_t > last_watered[i] + waterer_timeout)) {
        need_to_be_checked[i] = false;
        uint16_t h = pFlower[i]->getHumidity();
        if (h <= h_before[i]) {                        // The waterer failed
          pFlower[i]->activate(false);                 // Disable the waterer
        }
		    humidity[i] = h;
      }
    }
  }

  if (now_t > update_humidity) {                      // It is time to check the sensor humidity
    if (!is_dark) {
      for (byte i = 0; i < 2; ++i) {
        if (now_t > (last_watered[i] + humidity_period)) {
          humidity[i] = pFlower[i]->getHumidity();
          if (pFlower[i]->needRain(humidity[i])) {
	          h_before[i] = humidity[i];
            pFlower[i]->rain();
            last_watered[i] = now_t;
            need_to_be_checked[i] = true;
            update_humidity = now_t + chill_timeout;
            return;
          }
        }

      }
	  }
	  humidity[0] = pFlower[0]->getHumidity();
    humidity[1] = pFlower[1]->getHumidity();
    update_humidity = now_t + humidity_period;
  }

}

//---------------------------------------- class manual screen, manual watering --------------------------------
class manualSCREEN : public SCREEN {
  public:
    manualSCREEN(LiquidCrystal* LCD, ENCODER* ENC, WATERER* pF[2]) {
      update_screen = 0;
      pD = LCD;
      pEnc = ENC;
      pFlower[0] = pF[0];
      pFlower[1] = pF[1];
    }
    virtual void init(byte param);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    
  private:
    LiquidCrystal* pD;                                // Pointer to the display instance
    ENCODER* pEnc;                                    // Pointer to the rotary encoder instance
    WATERER* pFlower[2];
    uint32_t update_screen;                           // Time in ms to update the display
    const uint16_t period = 30000;                    // Period to refresh the screen
};

void manualSCREEN::init(byte param) {
  pD->clear();
  update_screen = 0;
  this->setSCRtimeout(30);
  pEnc->reset(0, 0, 2, 1, 0, true);
  forceRedraw();
}

void manualSCREEN::rotaryValue(int16_t value) {
  forceRedraw();
}

void manualSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + period;

  int index = pEnc->read();
  pD->clear();
  pD->print(F("Manual watering"));
  pD->setCursor(0, 1);
  if (index <= 1) {                                    // The flower chosen
    if (index == 0)
      pD->print(F("Left,"));
    else
      pD->print(F("Right,"));
    if (pFlower[index]->isActive())
      pD->print(F(" push"));
    else
      pD->print(F(" disabled"));
  } else {
    pD->print(F("Return to menu"));
  }
}

SCREEN* manualSCREEN::menu(void) {
  char buff[6];
  
  int index = pEnc->read();
  if (index < 2) {                                    // Run watering the flower
    if (pFlower[index]->isActive()) {
      pD->clear();
      pD->print(F("Watering "));
      if (index == 0)
        pD->print(F("left"));
      else
        pD->print(F("right"));
      uint32_t fwdTime = pFlower[index]->getFwdTime();
      fwdTime *= 100;                                 // The run forward time in milliseconds
      uint32_t stop_t = millis() + fwdTime;
      pD->setCursor(0, 1);
      pD->print(F("Rainning "));
      pFlower[index]->startManual();
      while (millis() < stop_t) {
        uint32_t r = (stop_t - millis()) / 1000;
        sprintf(buff, "%3ds", r);
        pD->setCursor(9, 1);
        pD->print(buff);
        delay(500);
      }
      pFlower[index]->finishManual();
    }
  }
  if (this->next != 0) return this->next; else return this;
}

//---------------------------------------- Main menu, global options configure ---------------------------------
class configSCREEN : public SCREEN {
  public:
    configSCREEN(LiquidCrystal* LCD, ENCODER* ENC, CONFIG* CFG, BACKLIGHT* BCKL, WATERER* pF[2]) {
      update_screen = 0;
      pD = LCD;
      pEnc = ENC;
      pCfg = CFG;
      pBcl = BCKL;
      pFlower[0] = pF[0];
      pFlower[1] = pF[1];
    }
    virtual void init(byte param);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    
  private:
    void restoreLCD(void);                            // Restore LCD brightness
    LiquidCrystal* pD;                                // Pointer to the display instance
    ENCODER* pEnc;                                    // Pointer to the rotary encoder instance
    CONFIG*  pCfg;                                    // Pointer to config class
    BACKLIGHT* pBcl;                                  // Pointer to backlight
    WATERER* pFlower[2];                              // Pointers to waterers
    byte go_index;                                    // Index of the global option
    bool modified;                                    // Weither the config was modifed
    struct cfg Cfg;                                   // The loaded config
	  bool lcd_auto;                                    // Current lcd parameters;
	  byte lcd_brightness;
    uint32_t update_screen;                           // Time in ms to update the display
    const uint16_t period = 30000;                    // Period to refresh the screen
    /*
     * 0, 1 - setup waterer (flower)
     * 2 - backlight
     * 3 - nightly brightness
	   * 4 - cancel changes
     * 5 - save changes
     */
};

void configSCREEN::init(byte param) {
  pD->clear();
  update_screen = 0;
  go_index = 0;
  modified = false;
  pEnc->reset(0, 0, 5, 1, 0, true);
  pCfg->getConfig(Cfg);
  lcd_auto = Cfg.backlight_auto;                       // Save lcd parameters to restore later on
  lcd_brightness = Cfg.backlight_manual;
  forceRedraw();
}

void configSCREEN::rotaryValue(int16_t value) {
  switch (go_index) {
    case 2:                                           // Backlight (0 is for auto)
	  if (value == 0) {
		  Cfg.backlight_auto = true;
		  pBcl->turnAuto(true);
    } else {
		  Cfg.backlight_auto = false;
		  Cfg.backlight_manual = value & 0xff;
		  pBcl->setBrightness(value & 0xff);
	  }
    break;
    case 3:                                           // Nightly brightness
	    Cfg.backlight_nightly = value & 0xff;
      pBcl->setBrightness(value & 0xff);
      break;
    default:
      break;
  }
  forceRedraw();
}

void configSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + period;

  pD->clear();
  pD->print(F("Settings"));
  uint16_t main_index = pEnc->read();
  if (go_index > 1) {                                 // Modifying global option
    pD->setCursor(13, 0);
	  pD->print(F("[*]"));
	  main_index = go_index;
  }
  pD->setCursor(1, 1);
  switch(main_index) {                                // The main setup menu
    case 0:                                           // Choose left waterer
      pD->print(F("Left waterer"));
      break;
    case 1:                                           // Choose right waterer
      pD->print(F("Right waterer"));
      break;
    case 2:                                           // backlight setup
      pD->print(F("Backlight: "));
      if (Cfg.backlight_auto)
        pD->print(F("auto"));
      else
        pD->print(Cfg.backlight_manual);
      break;
    case 3:                                           // Nightly brightness setup
      pD->print(F("Nightly: "));
      pD->print(Cfg.backlight_nightly);
      break;
	  case 4:                                           // cancel changes
	    pD->print(F("Cancel changes"));
	  break;
    case 5:
      pD->print(F("Save changes"));
    default:
      break;
  }
}

SCREEN* configSCREEN::menu(void) {
  if (go_index > 1) {                                 // The global option has been modified
    modified = true;
    pEnc->reset(go_index, 0, 5, 1, 0, true);
    go_index = 0;
    restoreLCD();
    return this;
  }
  go_index = pEnc->read();                            // Menu option is selected
  if (go_index <= 1) {                                // Waterer configuration
    if (this->next != 0) {
      next->init(go_index);                           // Select the waterer to be configured
      go_index = 0;
      if (modified) pCfg->updateConfig(Cfg);          // Save modified options into global config
      return next;
    } else {
      return this;
    }
  } else {                                            // global option is selected
    byte p = 0;
    switch (go_index) {
	    case 2:                                         // Get ready to setup the backlight (0 - auto)
	      p = Cfg.backlight_manual;
	      if (Cfg.backlight_auto) p = 0;
		    break;
	    case 3:                                         // Get ready to setup the nightly brightness
	      p = Cfg.backlight_nightly;
		    break;
	    case 4:                                         // Cancel all changes, restore screen settings
		    if (nextL) {
          pCfg->init();                               // Reread the config from EEPROM
          pCfg->load();
		      return nextL;
		    }
		    return this;
      case 5:                                         // Save changes
        return menu_long();
	    default:
	      return this;
	  }
    pEnc->reset(p, 0, 255, 1, 5, false);
  }
  return this;
}

SCREEN* configSCREEN::menu_long(void) {
  pCfg->saveConfig(Cfg);
  applyConfig();
  if (nextL) return nextL; else return this;
}

void configSCREEN::restoreLCD(void) {
  if (lcd_auto) {
    pBcl->turnAuto(true);
  } else {
    pBcl->setBrightness(lcd_brightness);
  }
}

//---------------------------------------- Waterer configuration menu ------------------------------------------
class watererCfgSCREEN : public SCREEN {
  public:
    watererCfgSCREEN(LiquidCrystal* LCD, ENCODER* ENC, CONFIG* CFG, WATERER* pF[2]) {
      update_screen = 0;
      pD = LCD;
      pEnc = ENC;
      pCfg = CFG;
      cfg_index = 255;
      pFlower[0] = pF[0];
      pFlower[1] = pF[1];
    }
    virtual void init(byte param);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    
  private:
    void printTime(uint16_t param);                   // Print fwd and bcw times in tens of seconds
    void testWaterer(void);                           // Test the watere with new parameters
    LiquidCrystal* pD;                                // Pointer to the display instance
    ENCODER* pEnc;                                    // Pointer to the rotary encoder instance
    CONFIG*  pCfg;                                    // Pointer to config class
    byte cfg_index;                                   // Waterer to be modified
    byte opt_index;                                   // Option index to be modified
  /*
   * 0 - enabled
   * 1 - dry threshold
   * 2 - forward time
   * 3 - backward time
   * 4 - test the waterer times
   * 5 - return to main menu
   */
    WATERER* pFlower[2];                              // Pointers to waterers
    bool modified;                                    // Weither the config was modifed
    struct cfg Cfg;                                   // The loaded config
    uint32_t update_screen;                           // Time in ms to update the display
    const uint16_t period = 30000;                    // Period to refresh the screen
};

void watererCfgSCREEN::init(byte param) {
  pD->clear();
  update_screen = 0;
  this->setSCRtimeout(30);
  if (param < 2) cfg_index = param;
  opt_index = 255;
  pEnc->reset(0, 0, 5, 1, 0, true);
  pCfg->getConfig(Cfg);
  forceRedraw();
}

void watererCfgSCREEN::rotaryValue(int16_t value) {
  if (opt_index != 255) {                             // The menu item has been selected, modify the options
    switch (opt_index) {
      case 1:                                         // Dry threshold
        Cfg.dry[cfg_index] = value;
      break;
      case 2:                                         // Forward time
        Cfg.fwd_time[cfg_index] = value;
        break;
      case 3:                                         // Backward time
        Cfg.bcw_time[cfg_index] = value;
      break;
    default:
      break;
    }
  }
  forceRedraw();
}

void watererCfgSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + period;
  if (cfg_index > 1) cfg_index = 0;

  pD->clear();
  if (cfg_index == 0)
    pD->print(F("Left waterer"));
  else
    pD->print(F("Right waterer"));
  if (opt_index != 255) {                              // Modifying the parameter
    pD->setCursor(13, 0);
    pD->print(F("[*]"));
  }
  pD->setCursor(1, 1);
  int index = opt_index;
  if (opt_index == 255) index = pEnc->read();         // No element selected, list the menu
  switch(index) {
    case 0:                                           // Weither the waterer is enabled
      if (Cfg.pump_enabled[cfg_index])
        pD->print(F("enabled"));
      else
        pD->print(F("disabled"));
      break;
    case 1:                                           // Dry threshold
      pD->print(F("Dry thr. "));
      pD->print(Cfg.dry[cfg_index]);
      break;
    case 2:                                           // Run forward time
      pD->print(F("run fwd "));
      printTime(Cfg.fwd_time[cfg_index]);
      break;
    case 3:                                           // Run backward time
      pD->print(F("run bck "));
      printTime(Cfg.bcw_time[cfg_index]);
      break;
    case 4:                                           // test forward and backward parameters
      pD->print(F("test waterer"));
      break;
    case 5:                                           // return to the main menu
      pD->print(F("return to setup"));
      break;
    default:
    break;
  }
}

SCREEN* watererCfgSCREEN::menu(void) {
  if (opt_index != 255) {                             // The option has been modified
    modified = true;
    pEnc->reset(opt_index, 0, 5, 1, 0, true);
    opt_index = 255;
    return this;
  }
  if (cfg_index > 1) cfg_index = 0;
  opt_index = pEnc->read();
  switch (opt_index) {
    case 0:                                           // Enable|Disable the waterer
      Cfg.pump_enabled[cfg_index] = !Cfg.pump_enabled[cfg_index];
      forceRedraw();
      modified = true;
      opt_index = 255;
      break;
    case 1:                                           // Get ready to setup the dry limit
      pEnc->reset(Cfg.dry[cfg_index], 100, 20000, 1, 5, false);
      break;
    case 2:                                           // Get ready tosetup the forward time
      pEnc->reset(Cfg.fwd_time[cfg_index], 0, 600, 1, 5, false);
      break;
    case 3:                                           // Get ready to setup the backward time
      pEnc->reset(Cfg.bcw_time[cfg_index], 0, 100, 1, 5, false);
      break;
    case 4:                                           // test current parameters
      testWaterer();
      opt_index = 255;
      forceRedraw();
      break;       
    case 5:                                           // return to main setup menu
      if (next) {
        if (modified) pCfg->updateConfig(Cfg);        // Save modified options into global config
        return next;
      }
      break;
    default:
      break;
  }
  return this;
}

void watererCfgSCREEN::printTime(uint16_t param) {
  char buff[6];

  sprintf(buff, "%2d.%ds", param / 10, param % 10);
  pD->print(buff);
}

void watererCfgSCREEN::testWaterer(void) {
  char buff[4];
  
  pD->clear();
 if (cfg_index == 0)
   pD->print(F("Left waterer"));
  else
   pD->print(F("Right waterer"));
  uint32_t fwdTime = Cfg.fwd_time[cfg_index];
  fwdTime *= 100;                                     // The run forward time in milliseconds
  uint32_t stop_t = millis() + fwdTime;
  pD->setCursor(0, 1);
  pD->print(F("Rainning "));
  pFlower[cfg_index]->startManual();
  while (millis() < stop_t) {
    uint32_t r = (stop_t - millis()) / 1000;
    sprintf(buff, "%2ds", r);
    pD->setCursor(9, 1);
    pD->print(buff);
    delay(500);
  }
  pFlower[cfg_index]->finishManual(Cfg.bcw_time[cfg_index]);
}

//=================================== End of class declarations ================================================
LiquidCrystal lcd(LCD_RS_PIN, LCD_E_PIN, LCD_DB4_PIN, LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN);
WATERER   flowerA(S1A_PIN, S1B_PIN, M_FORWARD, M_BACKWARD, M_A_PWM);
WATERER   flowerB(S2A_PIN, S2B_PIN, M_FORWARD, M_BACKWARD, M_B_PWM);
WATERER*  pFlower[2] = {&flowerA, &flowerB};
ENCODER   rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON    rotButton(R_BUTN_PIN);
BACKLIGHT bckLight(LIGHT_SENSOR, LCD_BLGHT_PIN);
CONFIG    watererCfg;

mainSCREEN       mainScr(&lcd, &rotEncoder, &bckLight, pFlower);
manualSCREEN     manualScr(&lcd, &rotEncoder, pFlower);
configSCREEN     cfgScr(&lcd, &rotEncoder, &watererCfg, &bckLight, pFlower);
watererCfgSCREEN wtrCfgScr(&lcd, &rotEncoder, &watererCfg, pFlower);

SCREEN* pCurrentScreen = &mainScr;

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);

  for (byte i = 0; i < 2; ++i) {
    pFlower[i]->init();
  }

  bckLight.init();

  watererCfg.init();
  watererCfg.load();
  applyConfig();
  
  rotEncoder.init();
  rotButton.init();
  delay(500);
  attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
  
  mainScr.next    = &manualScr;
  mainScr.nextL   = &cfgScr;
  manualScr.next  = &mainScr;
  manualScr.main  = &mainScr;
  cfgScr.next     = &wtrCfgScr;
  cfgScr.nextL    = &mainScr;
  wtrCfgScr.next  = &cfgScr;

  pCurrentScreen->init();
}

void rotEncChange(void) {
  rotEncoder.cnangeINTR();
}

void loop() {
  static int16_t old_pos = rotEncoder.read();

  SCREEN* nxt = pCurrentScreen->returnToMain();
  if (nxt != pCurrentScreen) {                // return to the main screen by timeout
    pCurrentScreen = nxt;
    pCurrentScreen->init();
	struct cfg Cfg;							  // Restore main screen paramaters
    watererCfg.getConfig(Cfg);
    if (Cfg.backlight_auto) {
      bckLight.turnAuto(true);
    } else {
      bckLight.setBrightness(Cfg.backlight_manual);
      bckLight.turnAuto(false);
    }
  }

  byte bStatus = rotButton.buttonCheck();
  switch (bStatus) {
    case 2:                                   // long press;
      nxt = pCurrentScreen->menu_long();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 1:                                   // short press
      nxt = pCurrentScreen->menu();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 0:                                   // Not pressed
    default:
      break;
  }

  int16_t pos = rotEncoder.read();
  if (old_pos != pos) {
    pCurrentScreen->rotaryValue(pos);
    old_pos = pos;
    if (pCurrentScreen->isSetup())
     pCurrentScreen->resetTimeout();
  }

  pCurrentScreen->show();
  bckLight.adjust();                          // Automatically adjust backlight of the screen
}

void applyConfig(void) {

  struct cfg Cfg;
  watererCfg.getConfig(Cfg);
  for (byte i = 0; i < 2; ++i) {
    pFlower[i]->activate(Cfg.pump_enabled[i]);
    pFlower[i]->setDryThreshold(Cfg.dry[i]);
    pFlower[i]->setParams(Cfg.fwd_time[i], Cfg.bcw_time[i]);
  }
  bckLight.setNightPeriod(Cfg.backlight_evening, Cfg.backlight_morning, Cfg.backlight_nightly, false);
  if (Cfg.backlight_auto) {
    bckLight.turnAuto(true);
  } else {
    bckLight.setBrightness(Cfg.backlight_manual);
    bckLight.turnAuto(false);
  }
}
