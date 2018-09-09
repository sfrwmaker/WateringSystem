/* 
 * The Home plant watering system.
 * Using L298n motor driver, soli moisture sensors and LCD1602@nano
 * v.2.0 Realeased 08/19/2017
 * New class hierarchy implemented, new configuration parameters added (check humidity period, test watering period, reactivate period)
*/
#include <LiquidCrystal.h>
#include <Time.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <CommonControls.h>

const byte NWP = 2;                                 // Number of the waterers

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
const byte S_PIN[NWP][2] = {
  {A0, A1},
  {A2, A3}
};

// Motor drives
const byte M_FORWARD  = A4;
const byte M_BACKWARD = A5;
const byte M_PWM[NWP] = {9, 10};

// The Rotary encoder
const byte R_MAIN_PIN = 2;                          // Rotary Encoder main pin (right)
const byte R_SECD_PIN = 8;                          // Rotary Encoder second pin (left)
const byte R_BUTN_PIN = 7;                          // Rotary Encoder push button pin

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
  uint32_t ID                           each time increment by 1
  struct cfg                            config data, 53 bytes
  byte CRC                              the checksum
*/
struct cfg {
  bool     pump_enabled[NWP];                       // Whether the pump is enabled
  bool     pump_smooth[NWP];                        // Whether the pump is starting smoothly
  uint16_t dry[NWP];                                // The dry threshold for pumps
  uint16_t fwd_time[NWP];                           // The time to run pump forward, tenth of seconds
  uint16_t bcw_time[NWP];                           // The time to run pump backward, tenth of seconds
  byte     backlight_nightly;                       // The nightly display brightness
  byte     backlight_manual;                        // Manually set up brightness
  bool     backlight_auto;                          // Whether automatically adjust display brightness
  byte     humidity_period;                         // The period in 10-minutes intervals of the humidity checking
  byte     watering_check;							            // The time in 10-seconds intervals when the flower humidity should be checked after it was watered
  byte     reactivate;                              // The time in 30-minutes intervals when the disabled flower should be activated  
};

class CONFIG {
  public:
    CONFIG() {
      can_write     = false;
      buffRecords   = 0;
      rAddr = wAddr = 0;
      eLength       = 0;
      nextRecID     = 0;
      byte rs = sizeof(struct cfg) + 5;               // The total config record size
      // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
      for (record_size = 8; record_size < rs; record_size <<= 1);
    }
    void init();
    bool load(void);
    void getConfig(struct cfg &Cfg);                  // Copy config structure from this class
    void updateConfig(struct cfg &Cfg);               // Copy updated config into this class
    bool save(void);                                  // Save current config copy to the EEPROM
    bool saveConfig(struct cfg &Cfg);                 // write updated config into the EEPROM

  protected:
    struct   cfg Config;

  private:
    bool     readRecord(uint16_t addr, uint32_t &recID);
    bool     can_write;                               // The flag indicates that data can be saved
    byte     buffRecords;                             // Number of the records in the outpt buffer
    uint16_t rAddr;                                   // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                                   // Address in the EEPROM to start write new record
    uint16_t eLength;                                 // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                               // next record ID
    byte     record_size;                             // The size of one record in bytes
};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
  eLength = EEPROM.length();
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
  if (records < (eLength / record_size)) {            // The EEPROM is not full
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
  return save();                                      // Save new data into the EEPROM
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
  nextRecID ++;                                       // Get ready to write next record
  return true;
}

bool CONFIG::load(void) {
  bool is_valid = readRecord(rAddr, nextRecID);
  nextRecID ++;
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
  summ ++;                                            // To avoid empty fields
  if (summ == Buff[record_size-1]) {                  // Checksumm is correct
    uint32_t ts = 0;
    for (char i = 3; i >= 0; --i) {
      ts <<= 8;
      ts |= Buff[byte(i)];
    }
    recID = ts;
    memcpy(&Config, &Buff[4], sizeof(struct cfg));
    return true;
  }
  return false;
}

//------------------------------------------ class WATERER CONFIG ----------------------------------------------
class WTR_CFG : public CONFIG {
  public:
    WTR_CFG() : CONFIG()                              { }
	  void     init(void);
	  uint32_t checkHumidityPeriod(void)                { return uint32_t(Config.humidity_period) * 600; }
	  uint32_t wateringCheckTime(void)                  { return uint32_t(Config.watering_check)  * 10; }
	  bool     isBcklightAuto(void)                     { return Config.backlight_auto; }
	  byte     backlight(void)                          { return Config.backlight_manual; }
    bool     isActive(byte index)                     { if (index < NWP) return Config.pump_enabled[index]; else return false; }
	  bool     isSmooth(byte index)                     { if (index < NWP) return Config.pump_smooth[index];  else return false; }
	  uint16_t dryLimit(byte index)                     { if (index < NWP) return Config.dry[index];          else return 0; }
	  uint16_t fwdTime(byte index)                      { if (index < NWP) return Config.fwd_time[index];     else return 0; }
	  uint16_t bcwTime(byte index)                      { if (index < NWP) return Config.bcw_time[index];     else return 0; }
    uint32_t reactivatePeriod(void)                   { return (uint32_t(Config.reactivate) * 30*60); }
  private:
    void setDefaults(bool Write);
};

void WTR_CFG::init(void) {
  CONFIG::init();
  if (!CONFIG::load()) setDefaults(false);            // If failed to load the data from EEPROM, initialize the config data with the default values
}

void WTR_CFG::setDefaults(bool Write) {
  for (byte i = 0; i < NWP; ++i) {
    Config.pump_enabled[i]   = false;
    Config.pump_smooth[i]    = false;
    Config.dry[i]            = 600;
    Config.fwd_time[i]       = 70;
    Config.bcw_time[i]       = 20;
  }
  Config.backlight_nightly = 30;
  Config.backlight_auto    = true;
  Config.backlight_manual  = 128;
  Config.humidity_period   = 1;                     // check the humidity every 10 minutes
  Config.watering_check    = 30;                    // Check the humidity in 300 seconds afrer watering
  if (Write) {
    CONFIG::save();
  }
}

//------------------------------------------ Humidity sensor: uses two analog pins to measure the humidity -----
class SENSOR {
  public:
    SENSOR(byte aPIN, byte bPIN) {
      sA = aPIN;
      sB = bPIN;
    }
    void init(void);
    uint16_t measureHumidity(void);                 // ln(capacity) * 100
  private:
    byte sA, sB;                                    // The pins used by the sensor
    const float IN_STRAY_CAP_TO_GND = 24.48;
    const float IN_CAP_TO_GND       = IN_STRAY_CAP_TO_GND;
    const float R_PULLUP            = 34.8;  
    const int   MAX_ADC_VALUE       = 1023;
};

void SENSOR::init(void) {
  pinMode(sA, OUTPUT);
  pinMode(sB, OUTPUT);
  digitalWrite(sA, LOW);
  digitalWrite(sB, LOW);
}

uint16_t SENSOR::measureHumidity(void) {
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

  uint32_t cap = round(capacitance);
  cap |= 0x1f;                                      // The error of capacitance should be rounded, fill up last 5 bits
  capacitance = (float)cap;
  float humidity = log(capacitance) * 100.0;
  uint16_t h = round(humidity);
  init();                                           // Turn off the power from the sensor
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
  void forward(bool smooth = false);                // Run the motor forward
  void backward(bool smooth = false);               // Run the motor backward
    
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

void PUMP::forward(bool smooth) {
  digitalWrite(pwm_pin, LOW);                       // First, stop the motor
  digitalWrite(forward_pin,  HIGH);
  digitalWrite(backward_pin, LOW);
  if (smooth) runSmoothly();
  digitalWrite(pwm_pin, HIGH);
}

void PUMP::backward(bool smooth) {
  digitalWrite(pwm_pin, LOW);                       // First, stop the motor
  digitalWrite(forward_pin,  LOW);
  digitalWrite(backward_pin, HIGH);
  if (smooth) runSmoothly();
  digitalWrite(pwm_pin, HIGH);
}

//------------------------------------------ Waterer (Sensor + motor pump) --------------------------------------
class WATERER: public SENSOR, public PUMP {
  public:
    WATERER(byte s_aPIN, byte s_bPIN, byte m_fwdPIN, byte m_bcwPIN, byte m_pwmPIN):
      SENSOR(s_aPIN, s_bPIN), PUMP(m_fwdPIN, m_bcwPIN, m_pwmPIN)
                                                    { }
    void     init(void);                            // Initialize
};

void WATERER::init(void) {
  SENSOR::init();
  PUMP::init();
}

//------------------------------------------ Hardware: all the waterers together, chill timeout -----------------
class HARDWARE {
  public:
    HARDWARE(WATERER* W[NWP]) {
      for (byte i = 0; i < NWP; ++i)
        pW[i] = W[i]; 
    }
    void       init(void);                          // initialize the hardware
    void       stop(byte index)                     { pW[index]->stop(); }
    void       stopAll(void);                       // Stop all the waterers
    void       startFWD(byte index, bool smooth);
    void       startBCW(byte index, bool smooth);
    void       forward(byte  index, uint16_t ds, bool smooth);
    void       backward(byte index, uint16_t ds, bool smooth);
    uint32_t   chillTimeout(void);                  // Time in seconds to wait before the hardware become ready
    uint16_t   measureHumidity(byte index);         // Measure the current humidity of the sensor index
  private:
    WATERER*   pW[NWP];                             // The pointers to all the waterers
    uint32_t   run_ts;                              // The timestamp of last run operation, to prevent overheating
    const byte chill_timeout   = 30;                // The minimal time between consequent run the pumps in seconds
};

void HARDWARE::init(void) {
  for (byte i = 0; i < NWP; ++i)
    pW[i]->init();
  stopAll();
  run_ts = 0;
}

void HARDWARE::stopAll(void) {
  for (byte i = 0; i < NWP; ++i)
    pW[i]->stop();
}

void HARDWARE::startFWD(byte index, bool smooth) {
  if (index < NWP) {
    pW[index]->forward(smooth);
    run_ts = now();
  }
}

void HARDWARE::startBCW(byte index, bool smooth) {
  if (index < NWP) {
    pW[index]->backward(smooth);
    run_ts = now();
  }
}

void HARDWARE::forward(byte index, uint16_t ds, bool smooth) {
  if (index < NWP) {
    startFWD(index, smooth);
    delay(ds*100);
  }
  pW[index]->stop();
}

void HARDWARE::backward(byte index, uint16_t ds, bool smooth) {
  if (index < NWP) {
    startBCW(index, smooth);
    delay(ds*100);
  }
  pW[index]->stop();
}

uint32_t HARDWARE::chillTimeout(void) {
  if (run_ts == 0) return 0;
  uint32_t elapsed = now() - run_ts;
  if (elapsed >= chill_timeout) return 0;
  return chill_timeout - elapsed;
}

uint16_t HARDWARE::measureHumidity(byte index) {
  if (index >= NWP) return 0;
  return pW[index]->measureHumidity();
}

//---------------------- Flower: all parameters of one flower ---------------------------------------------------
class FLOWER {
  public:
    FLOWER()                                        { }
    bool     enabled;                               // whether the flower is enabled
    uint16_t humidity;                              // humidity of the flower
    time_t   h_ts;                                  // The time stamp when the humidity was measured
    time_t   w_ts;                                  // The time stamp the flower was watered
    time_t   reactivate;                            // The time stamp when the flower should be reactivated
    byte     check_times;                           // The number of check attepmps after watering remains
};

//----------------------WateringPlant: All the waterers together (sure pause between watering) ------------------
class WTR_PLANT : protected HARDWARE {
  public:
    WTR_PLANT(WATERER* W[NWP], WTR_CFG* CFG) :
      HARDWARE(W)                                   { pCfg = CFG; }
    void      init(void);                           // Initialize the parameters
    void      stop(byte index)                      { HARDWARE::stop(index); }
    void      autoWatering(void);                   // automaticaly run the waterers
    byte      numberActive(void);                   // Returns the number of the active waterers
    bool      startForward(byte  index, bool smooth);
    bool      startBackward(byte index, bool smooth);
    void      activate(byte index, bool e);         // Activate the waterer
    bool      isActive(byte index)                  { return flower[index].enabled; }
    bool      isSmooth(byte index)                  { return pCfg->isSmooth(index); }
    uint16_t  getDryThreshold(byte index)           { return pCfg->dryLimit(index); }
    uint16_t  getFwdTime(byte index)                { return pCfg->fwdTime(index);  }
    uint16_t  getBcwTime(byte index)                { return pCfg->bcwTime(index);  }
    uint16_t  getHumidity(byte index)               { return flower[index].humidity; }
    uint32_t  secondsToWait(void);                  // The remaining seconds to wait next operation
  private:
    uint32_t   secs2Run(void);                      // The seconds to wait for next operation
    void       checkFlowers(void);                  // Check all the flower after warering, the humidity should uncrease
    void       rain(byte index);
    WTR_CFG*   pCfg;                                // The pointer to the config instance
    FLOWER     flower[NWP];                         // All the flowers
    uint32_t   next_run;                            // The time in secs when autoWatering can run the loop
    uint16_t   check_time;                          // The humidity check time after watering (seconds)
    uint32_t   humidity_period;                     // The period to recheck the flower humidity
    const byte flower_check_times = 3;              // Now many times the flower should be checked after watering
};

void WTR_PLANT::init(void) {
  HARDWARE::init();
  humidity_period = pCfg->checkHumidityPeriod();
  check_time      = pCfg->wateringCheckTime();
  time_t ts = now();
  for (byte i = 0; i < NWP; ++i) {
    flower[i].enabled     = pCfg->isActive(i);
    flower[i].humidity    = HARDWARE::measureHumidity(i);;
    flower[i].h_ts        = ts;
    flower[i].w_ts        = ts;
    flower[i].reactivate  = 0;
    flower[i].check_times = 0;
  }
  next_run = secs2Run() + now();
}

void WTR_PLANT::autoWatering(void) {
  if (now() < next_run) return;

  checkFlowers();                                   // Check the watered flower
  time_t ts = now();
  for (byte i = 0; i < NWP; ++i) {
    if (flower[i].enabled && (flower[i].check_times == 0) && (HARDWARE::chillTimeout() == 0)) {
      if (ts - flower[i].h_ts >= humidity_period) {
        uint16_t h = HARDWARE::measureHumidity(i);
        flower[i].humidity = h;
        flower[i].h_ts = ts;
        if (h < pCfg->dryLimit(i)) {
          rain(i);
        }
      }
    }
  }
  next_run = secs2Run() + now();
}

byte WTR_PLANT::numberActive(void) {
  byte n = 0;
  for (byte i = 0; i < NWP; ++i) {
    if (isActive(i)) ++n;
  }
  return n;
}

bool WTR_PLANT::startForward(byte index, bool smooth) {
  if (index < NWP) {
    HARDWARE::startFWD(index, smooth);
    return true;
  }
  return false;
}

bool WTR_PLANT::startBackward(byte index, bool smooth) {
  if (index < NWP) {
    HARDWARE::startBCW(index, smooth);
    return true;
  }
  return false;
}

void WTR_PLANT::activate(byte index, bool e) {      // Change the activation parameter in the memory, not in the config
  if (index < NWP)
    flower[index].enabled = e;
}

uint32_t WTR_PLANT::secondsToWait(void) {
  long s = next_run - now();
  if (s < 0) s = 0;
  return s;
}

void WTR_PLANT::checkFlowers(void) {
  for (byte i = 0; i < NWP; ++i) {
    if (!pCfg->isActive(i)) continue;               // Skip non-active waterers
    // Check the flower after watered, the humidity should increase. Otherwise, disable the waterer
    if (flower[i].check_times && ((now() - flower[i].w_ts) >= check_time)) {
      uint16_t h = HARDWARE::measureHumidity(i);
      if (h <= flower[i].humidity) {
        if (flower[i].check_times <= 1) {
          activate(i, false);                       // Disable the waterer because the humidity was not increased after watering
          flower[i].check_times = 0;
          uint32_t r_period = pCfg->reactivatePeriod();
          flower[i].reactivate = r_period;
          if (r_period) flower[i].reactivate += now();
        } else {
          --flower[i].check_times;
        }
      } else {                                      // The waterer works fine
        flower[i].humidity    = h;
        flower[i].h_ts        = now();
        flower[i].check_times = 0;
      }
    }
    // Reenable disabled waterers
    if (!flower[i].enabled && flower[i].reactivate && (flower[i].reactivate <= now())) {
      activate(i, true); 
    }
  }
}

void WTR_PLANT::rain(byte index) {
  uint16_t ds = pCfg->fwdTime(index);
  bool smooth = pCfg->isSmooth(index);
  HARDWARE::forward(index, ds, smooth);
  delay(200);
  ds = pCfg->bcwTime(index);
  HARDWARE::backward(index, ds, smooth);
  flower[index].w_ts = now();
  flower[index].check_times = flower_check_times;
}

uint32_t WTR_PLANT::secs2Run(void) {                                 
  time_t ts = now();
  long secs = 86400;                                // 24 Hours
  for (byte i = 0; i < NWP; ++i) {
    if (!pCfg->isActive(i)) continue;               // Do not check non-active flowers
    long remain = 0;
    if (flower[i].enabled) {
      if (flower[i].check_times) {
        long ta = ts - flower[i].w_ts;
        remain = check_time - ta;
      } else {
        long ta = ts - flower[i].h_ts;
        remain = humidity_period - ta;
      }
    } else {                                        // The waterer was disabled toe to error while watering (the humidity was not increased)
      if (flower[i].reactivate == 0)
        continue;
      remain = flower[i].reactivate - now();
    }
    if (remain < secs) secs = remain; 
  }
  if (secs < 0) secs = 0;
  long h_secs = HARDWARE::chillTimeout();           // The seconds for the hardware to become ready
  if (secs < h_secs) secs = h_secs;
  return secs;
}

//--------------------- class liquid crystal lcd with russian messages ----------------------------------------
class RUS_LCD : public LiquidCrystal {
  public:
    RUS_LCD(byte RS_PIN, byte E_PIN, byte DB4_PIN, byte DB5_PIN, byte DB6_PIN, byte DB7_PIN)
	  : LiquidCrystal(RS_PIN,E_PIN,DB4_PIN, DB5_PIN, DB6_PIN, DB7_PIN) { }
    void   init(void)                               { LiquidCrystal::begin(16, 2); }
    void   watererName(byte index);                 // Print the waterer name
	  void   timeLeft(uint32_t time_left);            // Print the time in seconds
	  void   printSeconds(uint32_t s);                // Print the time period in seconds as hh:mm:ss
    void   msgNoActiveWaterer(void);                // Print 'no waterer' message
    void   msgManual(void);                         // Print 'manual watering' message
    void   msgRun(void);                            // Print 'run' message
    void   msgDisabled(void);                       // Print 'disabled' message
    void   msgBack(void);                           // Print 'return back to menu' message
    void   msgRaining(void);                        // Print 'Raining' message
    void   msgSettings(void);                       // Print 'Settings' message
    void   msgBacklight(void);                      // Print 'Backlight' message
    void   msgAuto(void);                           // Print 'auto' message
    void   msgNightly(void);                        // Print 'Nightly' message
    void   msgCancel(void);                         // Print 'Cancel changes' message
    void   msgSave(void);                           // Print 'Save changes' message
    void   msgEnabled(void);                        // Print 'Enabled' message
    void   msgSmooth(void);                         // Print 'Smoothly' message
    void   msgNormal(void);                         // Print 'Normal' message
    void   msgDryThreshold(void);                   // Print 'Dry threshold' message
    void   msgRunFwd(void);                         // Print 'Run fwd time' message
    void   msgRunBcw(void);                         // Print 'Run bcw time' message
    void   msgTestWaterer(void);                    // Print 'test waterer' message
    void   msgReturn(void);                         // Print 'return to the menu' message
    void   msgDark(void);                           // Print 'it is dark' message
    void   msgStarting(void);                       // Print 'starting' message (at the beginning)
    void   msgReenable(bool Short);                 // Print 'reactivate' message
  private:
    const byte s_left[5]   = {'L', 'e', 'f', 't', ' '};
    const byte s_right[6]  = {'R', 'i', 'g', 'h', 't', ' ' };    
};

void RUS_LCD::watererName(byte index) {
  char buff[17];
  byte i = 0;
  switch (index) {
    case 0:                                         // Left
	  i = sizeof(s_left);
	  memcpy(buff, &s_left, i);
	  break;
	case 1:                                           // Right
	  i = sizeof(s_right);
	  memcpy(buff, &s_right, i);
	  break;
	default:
	  break;
  }
  for ( ; i < 16; ++i)
    buff[i] = ' ';
  buff[16] = '\0';
  LiquidCrystal::print(buff);
}

void RUS_LCD::timeLeft(uint32_t time_left) {
  char buff[6];
  if (time_left < 60) {
    sprintf(buff, "%3ds", time_left);
  } else if (time_left < 3600) {
    time_left += 30;
    time_left /= 60;
    sprintf(buff, "%3dm", time_left);
  } else {
    time_left += 1800;
    time_left /= 3600;
    sprintf(buff, "%3dh", time_left);
  }
  LiquidCrystal::print(buff);
}

void RUS_LCD::printSeconds(uint32_t s) {
  char buff[9];
  byte secs = s % 60; s /= 60;
  byte mins = s % 60; s /= 60;
  byte hous = s % 24;
  sprintf(buff, "%2d:%02d:%02d", hous, mins, secs);
  LiquidCrystal::print(buff);
}

void RUS_LCD::msgNoActiveWaterer(void) {
  LiquidCrystal::print(F("All off"));
}

void RUS_LCD::msgManual(void) {
   LiquidCrystal::print(F("Manual watering"));
}

void RUS_LCD::msgRun(void) {
   LiquidCrystal::print(F("run"));
}

void RUS_LCD::msgDisabled(void) {
  LiquidCrystal::print(F("disabled"));
}

void RUS_LCD::msgBack(void) {
    LiquidCrystal::print(F("Back"));
}

void RUS_LCD::msgRaining(void) {
  LiquidCrystal::print(F("Watering"));
}

void RUS_LCD::msgSettings(void) {
  LiquidCrystal::print(F("Setup"));
}

void RUS_LCD::msgBacklight(void) {
  LiquidCrystal::print(F("Brightness "));
}

void RUS_LCD::msgAuto() {
  LiquidCrystal::print(F("Auto"));
}

void RUS_LCD::msgNightly() {
  LiquidCrystal::print(F("at night "));
}

void RUS_LCD::msgCancel() {
  LiquidCrystal::print(F("Cancel"));
}

void RUS_LCD::msgSave() {
  LiquidCrystal::print(F("Save"));
}

void RUS_LCD::msgEnabled(void) {
  LiquidCrystal::print(F("enabled"));
}

void RUS_LCD::msgSmooth(void) {
  LiquidCrystal::print(F("start smoothly"));
}

void RUS_LCD::msgNormal(void) {
  LiquidCrystal::print(F("start normally"));
}

void RUS_LCD::msgDryThreshold(void) {
  LiquidCrystal::print(F("dry "));
}

void RUS_LCD::msgRunFwd(void) {
  LiquidCrystal::print(F("run fwd "));
}

void RUS_LCD::msgRunBcw(void) {
  LiquidCrystal::print(F("Run bck "));
}

void RUS_LCD::msgTestWaterer(void) {
  LiquidCrystal::print(F("Test"));
}

void RUS_LCD::msgReturn(void) {
  LiquidCrystal::print(F("Back to menu"));
}

void RUS_LCD::msgDark(void) {
  LiquidCrystal::print(F("dark"));
}

void RUS_LCD::msgStarting(void) {
  LiquidCrystal::print(F("Initializing"));
}

void RUS_LCD::msgReenable(bool Short) {
  if (Short)
    LiquidCrystal::print(F("reac. "));
  else
    LiquidCrystal::print(F("reactivate"));
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
  public:
    SCREEN*  next;                                  // Pointer to the next screen
    SCREEN*  nextL;                                 // Pointer to the next Level screen, usually, setup
    SCREEN*  main;                                  // Pointer to the main screen

    SCREEN(RUS_LCD* LCD) {
	    pD = LCD;
      next = nextL = main = 0;
      scr_timeout = 0;
      time_to_return = 0;
	    w_index = NWP;
    }
    virtual void init(void)                         { }
    virtual void show(void)                         { }
    virtual SCREEN* menu(void)                      { if (this->next != 0)  return this->next;  else return this; }
    virtual SCREEN* menu_long(void)                 { if (this->nextL != 0) return this->nextL; else return this; }
    virtual void rotaryValue(int16_t value)         { }
    void    selectWaterer(byte w)                   { w_index = w; }
    bool    isSetup(void)                           { return (scr_timeout != 0); }
	  void    remainTime(uint32_t t);
    SCREEN* returnToMain(void);
    void    resetTimeout(void);
    void    setSCRtimeout(uint16_t t);
  protected:
	  RUS_LCD* pD;                                      // Pointer to the display instance
    uint16_t scr_timeout;                             // Timeout is sec. to return to the main screen, canceling all changes
    uint32_t time_to_return;                          // Time in ms to return to main screen
	  byte     w_index;                                 // Waterer index (used to select waterer to be configured)
};

SCREEN* SCREEN::returnToMain(void) {
  if (main && (scr_timeout != 0) && (millis() >= time_to_return)) {
    scr_timeout = 0;
    return main;
  }
  return this;
}

void SCREEN::resetTimeout(void) {
  if (scr_timeout > 0)
    time_to_return = millis() + scr_timeout*1000;
}

void SCREEN::setSCRtimeout(uint16_t t) {
  scr_timeout = t;
  resetTimeout(); 
}

void SCREEN::remainTime(uint32_t t) {
  char buff[6];
  uint32_t stop_t = millis() + t * 100;
  while (millis() < stop_t) {
    pD->setCursor(10, 1);
    uint32_t r = (stop_t - millis()) / 100;
    sprintf(buff, "%3d.%1ds", r/10, r%10);
    pD->print(buff);
    delay(100);
  }
}

//---------------------------------------- class main creen ----------------------------------------------------
class mainSCREEN : public SCREEN {
  public:
    mainSCREEN(RUS_LCD* LCD, ENCODER* ENC, BL* BCKL, WTR_PLANT* W) : SCREEN(LCD) {
      update_screen = 0;
      pEnc = ENC;
	    pBcl = BCKL;
      pW   = W;
    }
    virtual void init(void);
    virtual SCREEN* menu(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    
  private:
    void            refreshDisplay(void);
	void		    checkDarkStatus(void);
    ENCODER*        pEnc;                           // Pointer to the rotary encoder instance
	BL*             pBcl;
    WTR_PLANT*      pW;
    uint32_t        update_screen;                  // Time in ms to update the display
    uint32_t        next_dark;                      // Time in ms to chech the dark status
    time_t          change_dark_time;               // The time to change the status from dark to day and back
	time_t		    light_on_time;					// The time to check the light is on (during night)
    time_t          morning;                        // The time to start check the morning light
    bool            clear_scr;                      // Whether the LCD should be cleared
    bool            is_dark_displayed;              // Whether 'is dark' message  is displayed on the screen
    bool            light_at_night;                 // Whether light was detected at night time
    const uint32_t change_period = 1800;            // 30 minutes period to change the dark status
	const uint32_t light_period  = 900;             // 15 minutes period the light should be on to shift the night time
    const uint32_t night_length  = 36000;           // 10 hours
    const uint32_t period        = 1000;            // The default update screen period in ms
};

void mainSCREEN::init(void) {
  pD->clear();
  pD->noCursor();                                   // The cursor may bo on after the clock setup
  pW->init();                                       // Reload configuration
  clear_scr = true;
  update_screen  = 0;
  change_dark_time = now() + change_period;
  morning = 0;                                      // morning does not checked because it is day time by default
  w_index = 0;
  pEnc->reset(0, 0, 1, 1, 0, true);       			    // Display modes: humidity / parameters
  update_screen = 0;
  next_dark = 0;
  light_on_time = 0;
  is_dark_displayed = false;
  light_at_night = false;
}

void mainSCREEN::show(void) {
  if (millis() < next_dark) return;
  next_dark = millis() + period;

  byte stat = byte(is_dark_displayed) ^ byte(!pBcl->isDark());
  if (stat) {
    is_dark_displayed = !is_dark_displayed;
    update_screen = 0;
  }

  checkDarkStatus();

  if (millis() < update_screen) return;

  refreshDisplay();

  uint32_t s2w = pW->secondsToWait();               // seconds to wait for the next operation
  time_t n = now();
  if (morning > 0) {
    if (n < morning) {
      s2w = morning - n;
    } else {
      s2w = change_dark_time - n;
    }
  }
  if (s2w > 60) {
    update_screen = millis() + (s2w % 60) * 1000;
  } else {
    update_screen = millis() + 1000;
  }

  int16_t mode = pEnc->read();                      // mode: 0 - normal, 1 - paramaters
  if (!mode) {                                      // Print minutes to next probe
    if (pBcl->isDark()) {
	    pD->setCursor(6, 0);
	    pD->print(F("dark"));
      if (morning > 0) {
        pD->setCursor(5, 1);
        pD->print(F(" night"));
      }
	  } else {
      pD->setCursor(6, 0);
      pD->timeLeft(s2w);
	  }
  }

  if ((morning == 0) && !pBcl->isDark())            // Run waterer only at day time and when it is not dark!
    pW->autoWatering();
}

SCREEN* mainSCREEN::menu(void) {
  // Check is there some waterer is enabled
  byte active = pW->numberActive();
  if (active == 0) {
    pD->clear();
    pD->msgNoActiveWaterer();
    update_screen = millis() + period;
  }
  if ((active > 0) && SCREEN::next != 0) {
    w_index = 0;
    return SCREEN::next;
  } else return this;
}

void mainSCREEN::rotaryValue(int16_t value) {
  pD->clear();
  update_screen = 0;
}

void mainSCREEN::refreshDisplay(void) {
  char buff[17];
  byte cur_pos[2] = {12, 9};
  
  int16_t mode = pEnc->read();                      // mode: 0 - normal, 1 - paramaters
  for (byte i = 0; i < NWP; ++i) {
    byte curr = 0;
    if (i == 1) curr = cur_pos[mode];
    if (mode) {                                     // parameters: forward and backward times
      if (pW->isActive(i)) {
        uint16_t fwdTime = pW->getFwdTime(i);
        uint16_t bcwTime = pW->getBcwTime(i);
        sprintf(buff, "f: %2d.%d", fwdTime/10, fwdTime%10);
        pD->setCursor(curr, 0);
        pD->print(buff);
        sprintf(buff, "b: %2d.%d", bcwTime/10, bcwTime%10);
        pD->setCursor(curr, 1);
        pD->print(buff);
      } else {
        pD->setCursor(curr, 0);
        pD->print(F("f: xx.x"));
        pD->setCursor(curr, 1);
        pD->print(F("b: xx.x"));
      }
    } else {
      if (pW->isActive(i)) {
        sprintf(buff, "%4d", pW->getHumidity(i));
        pD->setCursor(curr, 0);
        pD->print(buff);
        uint16_t dry = pW->getDryThreshold(i);
        sprintf(buff, "%4d", dry);
        pD->setCursor(curr, 1);
        pD->print(buff);
        pD->setCursor(5, 1);
        pD->print(F("<limt>"));
      } else {
        pD->setCursor(curr, 0);
        pD->print(F("xxxx"));
        pD->setCursor(curr, 1);
        pD->print(F(" off"));
      }
    }
  }
}

void mainSCREEN::checkDarkStatus(void) {
  time_t system_time = now();
  bool is_dark = pBcl->isDark();
  if (morning > 0) {                                // Night mode
    // First, check the light was on int the chamber, to shift the night period in winter
	if (!is_dark) {						            // Light is on
	  if (light_on_time == 0 || ((system_time >= light_on_time) && (system_time - light_on_time > night_length))) {
	    light_on_time = system_time + light_period; // Recheck the light at this time
	  } else if (system_time >= light_on_time) {
        light_on_time = 0;
		if ((morning > system_time) && (morning - system_time > night_length / 2)) {
		  light_at_night = true;
		}
	  }
	} else if (light_at_night) {                    // Dark Again!
        morning = system_time + night_length;       // Shift night period
        change_dark_time = morning + change_period;
        light_at_night = false;
        update_screen = 0;
	}
	// Second, check the night time is over
    if (system_time >= morning) {                   // The morning time has come, check the light
      if (is_dark) {
        change_dark_time = system_time + change_period;// Reset the timeout
      } else if (system_time >= change_dark_time) {
        morning = 0;                                // Change to the day time
        change_dark_time = system_time + change_period;
        update_screen = 0;
      }
    }
  } else {                                          // Day mode
    if (!is_dark) {
      change_dark_time = system_time + change_period; // Reset the timeout
    } else {
      if (system_time >= change_dark_time) {        // Change to night time
        morning = system_time + night_length;
        change_dark_time = morning + change_period;
        light_at_night = false;
        update_screen = 0;
      }
    }
  }
}

//---------------------------------------- class manual screen, manual watering --------------------------------
class manualSCREEN : public SCREEN {
  public:
    manualSCREEN(RUS_LCD* LCD, ENCODER* ENC, WTR_PLANT* W) : SCREEN(LCD){
      update_screen = 0;
      pEnc = ENC;
      pW   = W;
    };
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    
  private:
    ENCODER*       pEnc;                            // Pointer to the rotary encoder instance
    WTR_PLANT*     pW;
    uint32_t       update_screen;                   // Time in ms to update the display
    const uint16_t period = 30000;                  // Period to refresh the screen in ms
};

void manualSCREEN::init(void) {
  if (w_index >= NWP) w_index = 0;
  pD->clear();
  SCREEN::setSCRtimeout(30);
  pEnc->reset(w_index, 0, NWP, 1, 0, true);
  update_screen = 0;
}

void manualSCREEN::rotaryValue(int16_t value) {
  update_screen = 0;
}

void manualSCREEN::show(void) {
  if (millis() < update_screen) return;

  update_screen = millis() + period;

  int index = pEnc->read();
  pD->clear();
  pD->msgManual();
  pD->setCursor(1, 1);
  if (index < NWP) {
    pD->watererName(index);
    pD->setCursor(10, 1);
    if (pW->isActive(index))
      pD->msgRun();
    else
      pD->msgDisabled();
  } else {
    pD->msgBack();
  }
}

SCREEN* manualSCREEN::menu(void) {
  int index = pEnc->read();
  if (index < NWP) {                                // Run watering the flower
    if (pW->isActive(index)) {
      pD->clear();
      pD->watererName(index);

      uint32_t fwdTime = pW->getFwdTime(index);
      uint32_t bcwTime = pW->getBcwTime(index);
      bool     smooth  = pW->isSmooth(index);
      pD->setCursor(1, 1);
      pD->msgRaining();
      pW->startForward(index, smooth);
      remainTime(fwdTime);
      pW->stop(index);
      delay(200);
      pW->startBackward(index, smooth);
      SCREEN::remainTime(bcwTime);
      pW->stop(index);
    }
  }
  if (next != 0) return next; else return this;
}

//---------------------------------------- Main menu, global options configure ---------------------------------
class configSCREEN : public SCREEN {
  public:
    configSCREEN(RUS_LCD* LCD, ENCODER* ENC, WTR_CFG* CFG, BL* BCKL, WTR_PLANT* W) : SCREEN(LCD) {
      update_screen = 0;
      pEnc = ENC;
      pCfg = CFG;
      pBcl = BCKL;
      pW   = W;
      curr_index = 0;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    SCREEN* clock;                                  // Clock Setup SCREEN
    
  private:
    struct cfg Cfg;                                 // Modify all the parameters here
    void restoreLCD(void);                          // Restore LCD brightness
    ENCODER*       pEnc;                            // Pointer to the rotary encoder instance
    WTR_CFG*       pCfg;                            // Pointer to config class
    BL*            pBcl;                            // Pointer to backlight
    WTR_PLANT*     pW;                              // Pointer to waterer Plant
    byte           curr_index;                      // The current item index
    byte           go_index;                        // Index of the global option
    bool           modified;                        // Whether the config was modifed
    bool           lcd_auto;                        // Current lcd parameters;
    byte           lcd_brightness;
    uint32_t       update_screen;                   // Time in ms to update the display
    const uint16_t period = 30000;                  // Period to refresh the screen
    /*
     * 0, 1, ... NWP-1 - setup waterer (flower)
     * NWP   - backlight
     * NWP+1 - nightly brightness
	   * NWP+2 - humidity check period
	   * NWP+3 - waterer check time
     * NWP+4 - reenable time
     * NWP+5 - cancel changes
     * NWP+6 - save changes
     */
};

void configSCREEN::init(void) {
  pD->clear();
  go_index = 0;
  modified = false;
  if (curr_index > NWP+7) curr_index = 0;
  pEnc->reset(curr_index, 0, NWP+6, 1, 0, true);
  pCfg->getConfig(Cfg);
  lcd_auto       = Cfg.backlight_auto;             // Save lcd parameters to restore later on
  lcd_brightness = Cfg.backlight_manual;
  update_screen = 0;
}

void configSCREEN::show(void) {
  if (millis() < update_screen) return;
  update_screen = millis() + period;

  pD->clear();
  pD->msgSettings();
  uint16_t main_index = pEnc->read();
  if (go_index >= NWP) {                            // Modifying global option
    pD->setCursor(13, 0);
    pD->print(F("[*]"));
    main_index = go_index;
  }
  pD->setCursor(1, 1);
  uint32_t data = 0;
  switch(main_index) {                              // The main setup menu
    case NWP:                                       // backlight setup
      pD->msgBacklight();
      if (Cfg.backlight_auto)
        pD->msgAuto();
      else
        pD->print(Cfg.backlight_manual);
      break;
    case NWP+1:                                     // Nightly brightness setup
      pD->msgNightly();
      pD->print(Cfg.backlight_nightly);
      break;
	  case NWP+2:
	    data = Cfg.humidity_period; data *= 600;      // Check humidity period in seconds
      if (go_index >= NWP) {
         pD->print(F("period "));
	       pD->printSeconds(data);
      } else {
        pD->print(F("Check period"));
      }
	    break;
	  case NWP+3:
	    data = Cfg.watering_check; data *= 10;        // Waterer check time in seconds
      if (go_index >= NWP) {
        pD->print(F("wtrd. "));
        pD->printSeconds(data);
      } else {
        pD->print(F("Check watered"));
      }
	    break;
    case NWP+4:                                     // reenable time (30-minutes intervals)
      data = Cfg.reactivate; data *= 30*60;         // Reenable time in seconds
      if (go_index >=NWP) {
        pD->msgReenable(true);
        if (data)
          pD->printSeconds(data);
        else
          pD->msgDisabled();
      } else {
        pD->msgReenable(false);
      }
      break;
    case NWP+5:                                     // cancel changes
      pD->msgCancel();
      break;
    case NWP+6:                                     // save changes
      pD->msgSave();
      break;
    default:                                        // Configure the waterer [0; NWP-1]
      pD->watererName(main_index);
      break;
  }
}

void configSCREEN::rotaryValue(int16_t value) {
  switch (go_index) {
    case NWP:                                       // Backlight (0 is for auto)
      if (value == 0) {
        Cfg.backlight_auto = true;
        pBcl->turnAuto(true);
      } else {
        Cfg.backlight_auto = false;
        Cfg.backlight_manual = value & 0xff;
        pBcl->setBrightness(value & 0xff);
      }
      break;
    case NWP+1:                                     // Nightly brightness
      Cfg.backlight_nightly = value & 0xff;
      pBcl->setBrightness(value & 0xff);
      break;
	  case NWP+2:                                     // Check humidity period (in 10 minutes intervals)
	    Cfg.humidity_period   = value & 0xff;
	    break;
	  case NWP+3:                                     // Check waterer time (in 10 seconds intervals)
	    Cfg.watering_check    = value & 0xff;
	    break;
    case NWP+4:                                     // Reenable time (30-minutes intervals)
      Cfg.reactivate        = value & 0xff;
      break;
    default:
      break;
  }
  update_screen = 0;
}

SCREEN* configSCREEN::menu(void) {
  if (go_index >= NWP) {                            // The global option has been modified
    modified = true;
    pEnc->reset(go_index, 0, NWP+6, 1, 0, true);
    go_index = 0;
    restoreLCD();
    return this;
  }
  go_index = pEnc->read();                          // Menu option is selected
  if (go_index < NWP) {                             // Waterer configuration
    if (next != 0) {
      next->selectWaterer(go_index);                // Select the waterer to be configured
      curr_index = go_index;                        // Save index of currnetly modifying waterer
      go_index = 0;
      if (modified) pCfg->updateConfig(Cfg);        // Save modified options into global config
      return next;                                  // Call the next screen
    } else {
      return this;
    }
  } else {                                          // global option is selected
    uint16_t p = 0;
    switch (go_index) {
      case NWP:                                     // Get ready to setup the backlight (0 - auto)
        p = Cfg.backlight_manual;
        if (Cfg.backlight_auto) p = 0;
        break;
      case NWP+1:                                   // Get ready to setup the nightly brightness
        p = Cfg.backlight_nightly;
        break;
	    case NWP+2:                                   // Get Ready to setup humidity check period
	      p = Cfg.humidity_period;
		    pEnc->reset(p, 1, 143, 1, 6, false);
		    return this;
	    case NWP+3:
	      p = Cfg.watering_check;                     // Get ready to setup waterer check time
		    pEnc->reset(p, 3, 120, 1, 1, false);
		    return this;
      case NWP+4:                                   // Reenable time
        p = Cfg.reactivate;                         // The time in 30 minutes intervals
        pEnc->reset(p, 0, 191, 1, 1, false);        // [0-96 hours]
        break;
      case NWP+5:                                   // Cancel all changes, restore screen settings
        if (nextL) {
          pCfg->init();                             // Reread the config from EEPROM
          pCfg->load();
          curr_index = 0;
          return nextL;
        }
        return this;
      case NWP+6:                                   // Save changes
        return menu_long();
      default:
        return this;
    }
    pEnc->reset(p, 0, 255, 1, 5, false);
  }
  return this;
}

SCREEN* configSCREEN::menu_long(void) {
  // Apply the changes to the backlight only, other config will be applied in the main screen
  pCfg->saveConfig(Cfg);
  if (Cfg.backlight_auto) {
    pBcl->turnAuto(true);
  } else {
    pBcl->setBrightness(Cfg.backlight_manual);
    pBcl->turnAuto(false);
  }
  curr_index = 0;                                     // The setup has been finished, next time start from the first waterer
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
    watererCfgSCREEN(RUS_LCD* LCD, ENCODER* ENC, WTR_CFG* CFG, WTR_PLANT* W) : SCREEN(LCD){
      update_screen = 0;
      pEnc = ENC;
      pCfg = CFG;
      pW   = W;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    
  private:
    void      printTime(uint16_t param);            // Print fwd and bcw times in tens of seconds
    void      testWaterer(void);                    // Test the watere with new parameters
    ENCODER*  pEnc;                                 // Pointer to the rotary encoder instance
    WTR_CFG*  pCfg;                                 // Pointer to config class
    byte      opt_index;                            // Option index to be modified
  /*
   * 0 - enabled
   * 1 - smooth
   * 2 - dry threshold
   * 3 - forward time
   * 4 - backward time
   * 5 - test the waterer times
   * 6 - return to main menu
   */
    WTR_PLANT* pW;                                  // Pointers to the waterer plant
    bool       modified;                            // Whether the config was modifed
    struct cfg Cfg;                                 // The loaded config
    uint32_t   update_screen;                       // Time in ms to update the display
    const uint16_t period = 30000;                  // Period to refresh the screen
    const uint16_t max_time = 100;                  // Maximum value for run motor time in tenth of the second
    const uint16_t min_time = 5;                    // Minimum value for run motor time in tenth of the second
};

void watererCfgSCREEN::init(void) {
  pD->clear();
  SCREEN::setSCRtimeout(30);
  if (w_index >= NWP) w_index = 0;
  opt_index = 255;                                  // [0-NWP-1] - option is modifying, 255 - no option selected
  pEnc->reset(0, 0, 6, 1, 0, true);                 // [0-6] - enable, smooth, dry, fwd, bck, test, return
  pCfg->getConfig(Cfg);
  update_screen = 0;
}

void watererCfgSCREEN::show(void) {
  if (millis() < update_screen) return;

  update_screen = millis() + period;

  pD->clear();
  pD->watererName(w_index);
  if (opt_index != 255) {                           // Modifying the parameter
    pD->setCursor(13, 0);
    pD->print(F("[*]"));
  }
  pD->setCursor(1, 1);
  int index = opt_index;
  if (opt_index == 255) index = pEnc->read();       // No element selected, list the menu
  switch(index) {
    case 0:                                         // Whether the waterer is enabled
      if (Cfg.pump_enabled[w_index])
        pD->msgEnabled();
      else
        pD->msgDisabled();
      break;
    case 1:                                         // Whether the waterer is started smoothly
      if (Cfg.pump_smooth[w_index])
        pD->msgSmooth();
      else
        pD->msgNormal();
      break;
    case 2:                                         // Dry threshold
      pD->msgDryThreshold();
      pD->print(Cfg.dry[w_index]);
      break;
    case 3:                                         // Run forward time
      pD->msgRunFwd();
      printTime(Cfg.fwd_time[w_index]);
      break;
    case 4:                                         // Run backward time
      pD->msgRunBcw();
      printTime(Cfg.bcw_time[w_index]);
      break;
    case 5:                                         // test forward and backward parameters
      pD->msgTestWaterer();
      break;
    case 6:                                         // return to the main menu
      pD->msgReturn();
      break;
    default:
    break;
  }
}

void watererCfgSCREEN::rotaryValue(int16_t value) {
  if (opt_index != 255) {                           // The menu item has been selected, modify the options
    switch (opt_index) {
      case 2:                                       // Dry threshold
        Cfg.dry[w_index]      = value;
      break;
      case 3:                                       // Forward time
        Cfg.fwd_time[w_index] = value;
        break;
      case 4:                                       // Backward time
        Cfg.bcw_time[w_index] = value;
      break;
    default:
      break;
    }
  }
  update_screen = 0;
}

SCREEN* watererCfgSCREEN::menu(void) {
  if (opt_index != 255) {                           // The option has been modified
    modified = true;
    pEnc->reset(opt_index, 0, 6, 1, 0, true);
    opt_index = 255;
    return this;
  }
  opt_index = pEnc->read();
  switch (opt_index) {
    case 0:                                         // Enable|Disable the waterer
      Cfg.pump_enabled[w_index] = !Cfg.pump_enabled[w_index];
      update_screen = 0;
      modified = true;
      opt_index = 255;
      break;
    case 1:                                         // Run smoothly the waterer
      Cfg.pump_smooth[w_index] = !Cfg.pump_smooth[w_index];
      update_screen = 0;
      modified = true;
      opt_index = 255;
      break;
    case 2:                                         // Get ready to setup the dry limit
      pEnc->reset(Cfg.dry[w_index], 100, 20000, 1, 5, false);
      break;
    case 3:                                         // Get ready tosetup the forward time
      pEnc->reset(Cfg.fwd_time[w_index], min_time, max_time, 1, 5, false);
      break;
    case 4:                                         // Get ready to setup the backward time
      pEnc->reset(Cfg.bcw_time[w_index], min_time, max_time, 1, 5, false);
      break;
    case 5:                                         // test current parameters
      testWaterer();
      opt_index = 255;
      update_screen = 0;
      break;       
    case 6:                                         // return to main setup menu
      if (next) {
        if (modified) pCfg->updateConfig(Cfg);      // Save modified options into global config
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

  sprintf(buff, "%2d.%dc", param / 10, param % 10);
  pD->print(buff);
}

void watererCfgSCREEN::testWaterer(void) {
  char buff[4];
 
  pD->clear();
  pD->watererName(w_index);
  uint32_t fwdTime = Cfg.fwd_time[w_index];
  pD->setCursor(0, 1);
  pD->msgRaining();
  pW->startForward(w_index, Cfg.pump_smooth[w_index]);
  SCREEN::remainTime(fwdTime);
  pW->stop(w_index);
  delay(200);
  uint32_t bcwTime = Cfg.bcw_time[w_index];
  pW->startBackward(w_index, Cfg.pump_smooth[w_index]);
  SCREEN::remainTime(bcwTime);
  pW->stop(w_index);
}

//=================================== End of class declarations ================================================
WTR_CFG           watererCfg;
WATERER           w0(S_PIN[0][0], S_PIN[0][1], M_FORWARD, M_BACKWARD, M_PWM[0]);
WATERER           w1(S_PIN[1][0], S_PIN[1][1], M_FORWARD, M_BACKWARD, M_PWM[1]);
WATERER*          pWTR[NWP] = {&w0, &w1};
WTR_PLANT         wtrPlant(pWTR, &watererCfg);

RUS_LCD           lcd(LCD_RS_PIN, LCD_E_PIN, LCD_DB4_PIN, LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN);
ENCODER           rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON            rotButton(R_BUTN_PIN);
BL                bckLight(LIGHT_SENSOR, LCD_BLGHT_PIN);

mainSCREEN        mainScr(&lcd, &rotEncoder, &bckLight, &wtrPlant);
manualSCREEN      manualScr(&lcd, &rotEncoder, &wtrPlant);
configSCREEN      cfgScr(&lcd, &rotEncoder, &watererCfg, &bckLight, &wtrPlant);
watererCfgSCREEN  wtrCfgScr(&lcd, &rotEncoder, &watererCfg, &wtrPlant);

SCREEN* pCurrentScreen = &mainScr;

void setup() {
  //Serial.begin(9600);
  lcd.init();
  lcd.clear();
  lcd.noCursor();
  lcd.msgStarting();
  
  bckLight.init();
  bckLight.setLimits(400, 700, 5, 150);
  watererCfg.init();
  
  if (watererCfg.isBcklightAuto()) {
    bckLight.turnAuto(true);
  } else {
    bckLight.setBrightness(watererCfg.backlight());
    bckLight.turnAuto(false);
  }
  
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
  rotEncoder.changeINTR();
}

void loop() {
  static int16_t old_pos = rotEncoder.read();

  SCREEN* nxt = pCurrentScreen->returnToMain();
  if (nxt != pCurrentScreen) {                      // return to the main screen by timeout
    pCurrentScreen = nxt;
    pCurrentScreen->init();
    if (watererCfg.isBcklightAuto()) {
      bckLight.turnAuto(true);
    } else {
      bckLight.setBrightness(watererCfg.backlight());
      bckLight.turnAuto(false);
    }
  }

  byte bStatus = rotButton.buttonCheck();
  switch (bStatus) {
    case 2:                                         // long press;
      nxt = pCurrentScreen->menu_long();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 1:                                         // short press
      nxt = pCurrentScreen->menu();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 0:                                         // Not pressed
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
  bckLight.adjust();                                // Automatically adjust backlight of the screen
}

