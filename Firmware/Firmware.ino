#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v2.h>
#include <sTune.h> 

// ======================= CONSTANTS & PIN DEFINES ========================
#define HISTORY_SIZE 5
#define RT0 100000       // Ω
#define B 3950           // K
#define VCC 5.0          // Adjust based on your Arduino's operating voltage
#define R 100000         // R=100KΩ

LiquidCrystal_I2C lcd(0x27, 16, 2);   // Adjust if your LCD address is 0x3f

// Pins
const int but_3 = 10; 
const int but_4 = 9;
const int SSR   = 4;
const int buzzer= 6;
const int Thermistor1_PIN = A0;
const int Thermistor2_PIN = A1;

// ======================= PID & TUNER SETTINGS ===========================
double input, output, setpoint = 50, kp, ki, kd;  
float Input, Output, Setpoint = 50, Kp, Ki, Kd;    

// Initialize tuner and PID
sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
PID myPID(&input, &output, &setpoint, kp, ki, kd, P_ON_M, DIRECT);

float outputSpan   = 255;     // Max Output range
double outputStart = 0;
double outputStep  = 510;
double inputSpan   = 300;     // Example max temperature for tuning
uint32_t settleTimeSec = 10;
uint32_t testTimeSec   = 100;
const uint16_t samples = 500;
uint8_t debounce       = 1;

bool tuning       = false; 
bool firstRun     = true;
double tempLimit  = 200;      // Hard safety limit for temperature

// ======================== TIMING VARIABLES ==============================
unsigned long millis_before, millis_before_2;
unsigned long millis_now    = 0;
float        seconds       = 0;  
const float  refresh_rate  = 500;   // ms, LCD + some logic refresh
const float  pid_refresh_rate = 50; // ms, how often to compute PID

// ========================== BUTTON DEBOUNCE ============================
bool lastBut_3State    = HIGH;
bool lastBut_4State    = HIGH;
bool but_3_state       = true;
bool but_4_state       = true;
unsigned long lastDebounceTime_3 = 0;
unsigned long lastDebounceTime_4 = 0;
const long debounceDelay = 50;

// ====================== REFLOW STATE MACHINE ===========================
enum ReflowState {
  IDLE,
  PREHEAT_WARMUP,
  PREHEAT,
  SOAK_WARMUP,
  SOAK,
  REFLOW_WARMUP,
  REFLOW,
  COOLDOWN
};

ReflowState current_state = IDLE;

// ===================== REFLOW PARAMETERS ===============================
float preheat_setpoint  = 100;
float soak_setpoint     = 150;
float reflow_setpoint   = 180;
float cooldown_setpoint = 0;

float preheat_time      = 60;   // seconds
float soak_time         = 90;
float reflow_time       = 40;
float cooldown_time     = 10;

float cooldown_temp     = 40;   // Safe to touch
float cooling_rate      = 3.0;  // deg/sec
float min_temp          = 0;

float total_time_before_cooling = (preheat_time + soak_time + reflow_time);

// ====================== OTHER GLOBALS =================================
int  running_mode    = 0;   
int  selected_mode   = 0;   
const int  max_modes       = 2;   
bool idleState       = true;
float temperature    = 0;  
float pwm_value      = 255;  // SSR OFF with HIGH
float temp_setpoint  = 50;  
unsigned long state_start_time = 0;

// LCD previous-line trackers
char line1[16], line2[16];
float prev_temperature   = -1;
float prev_temp_setpoint = -1;
float prev_pwm_value     = -1;
int   prev_running_mode  = -1;
int   prev_selected_mode = -1;
float prev_seconds       = -1;

// ====================== TEMPERATURE FILTERS ============================
float historyA0[HISTORY_SIZE];
float historyA1[HISTORY_SIZE];
int   historyIndexA0 = 0;
int   historyIndexA1 = 0;

// ===================== FUNCTION DECLARATIONS ===========================
float  readTemperature();
float  readRawTemperature(int pin);
float  average(float arr[], int size);
void   printSelectedMode(int selected_mode);
void   updateDisplay(float temperature, float temp_setpoint, float pwm_value, 
                     int running_mode, int selected_mode, float seconds, 
                     const char* transition_phase = nullptr);
void   calculatePID();
void   controlReflowProcess();
void   handleButtons();

void setup() {
  Serial.begin(9600);  
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, HIGH); // SSR OFF
  pinMode(buzzer, OUTPUT); 
  digitalWrite(buzzer, LOW);

  pinMode(but_3, INPUT_PULLUP);
  pinMode(but_4, INPUT_PULLUP);
  pinMode(Thermistor1_PIN, INPUT);
  pinMode(Thermistor2_PIN, INPUT);

  lcd.init();
  lcd.backlight();
  tone(buzzer, 1800, 200);  

  millis_before   = millis();
  millis_before_2 = millis();

  // **Adjust analogReference based on your setup**
  // If using default reference, no need to set it
  // If using internal reference, uncomment the following line:
  
  analogReference(INTERNAL);
  // Remove or comment out analogReference(EXTERNAL);

  // Configure tuner
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);

  // Initialize the sTune + PID
  myPID.SetOutputLimits(0, outputSpan);
  myPID.SetMode(MANUAL); // Set to AUTOMATIC when running

  // Initialize temperature reading history
  temperature = readTemperature();
  for (int i = 0; i < HISTORY_SIZE; i++) {
    historyA0[i] = temperature;
    historyA1[i] = temperature;
  }
}

void loop() {
  handleButtons();

  millis_now = millis();
  
  // ================== PID REFRESH =====================
  if (millis_now - millis_before_2 > pid_refresh_rate) {
    millis_before_2 = millis_now;

    temperature = readTemperature();

    // Hard limit protection
    if (temperature >= tempLimit) {
      // Force SSR OFF
      digitalWrite(SSR, HIGH);
      temp_setpoint = 0; 
      Serial.println("EMERGENCY STOP: Over tempLimit!");
      running_mode = 0; // Optionally set to IDLE
    }

    // Operating Modes
    if (running_mode == 1) { // Reflow
      myPID.SetMode(AUTOMATIC);
      controlReflowProcess();
    }
    else if (running_mode == 2) { // PID Tuning
      myPID.SetMode(AUTOMATIC);
      calculatePID(); // Tuning/AutoTune via sTune
    }
    else { // IDLE
      current_state = IDLE;
      temp_setpoint = 0;
      state_start_time = 0;
      myPID.SetMode(MANUAL);
      digitalWrite(SSR, HIGH); // SSR OFF
    }
  }

  // ================== LCD REFRESH =====================
  if (millis_now - millis_before > refresh_rate) {
    millis_before = millis_now;  
    seconds += (refresh_rate / 1000.0f);

    // Transition message handling
    static float last_temp_setpoint = -1;
    const char* transition_message = nullptr;
    if (temp_setpoint != last_temp_setpoint) {
      if      (temp_setpoint == preheat_setpoint) transition_message = "PREHEAT PHASE";
      else if (temp_setpoint == soak_setpoint)    transition_message = "SOAK PHASE";
      else if (temp_setpoint == reflow_setpoint)  transition_message = "REFLOW PHASE";
      else if (temp_setpoint == cooldown_setpoint)transition_message = "COOL DOWN PHASE";
      last_temp_setpoint = temp_setpoint;
    }

    updateDisplay(temperature, temp_setpoint, pwm_value, 
                  running_mode, selected_mode, seconds, transition_message);

    // Completion handling
    if (running_mode == 11) {
      // Wait for cooldown below threshold
      if (temperature < cooldown_temp) {
        running_mode = 0;
        tone(buzzer, 1000, 100); 
      }
      digitalWrite(SSR, HIGH);
    }
  }
}

// ================== REFLOW PROCESS CONTROL =========================
void controlReflowProcess() {
  // Initialize state if IDLE
  if (current_state == IDLE) {
    current_state     = PREHEAT_WARMUP;
    temp_setpoint     = preheat_setpoint;
    state_start_time  = millis(); 
  }

  unsigned long elapsed_time = millis() - state_start_time;

  switch (current_state) {
    case PREHEAT_WARMUP:
      if (temperature >= temp_setpoint) {
        current_state    = PREHEAT;
        state_start_time = millis();  // Set to current time
      }
      Serial.println("Preheat Warmup");
      calculatePID();
      break;

    case PREHEAT:
      if (elapsed_time >= (unsigned long)(preheat_time * 1000)) {
        current_state    = SOAK_WARMUP;
        temp_setpoint    = soak_setpoint;
        state_start_time = millis(); 
      }
      Serial.print("Preheat Phase - Wait ");
      Serial.print(preheat_time);
      Serial.println(" sec");
      calculatePID();
      break;

    case SOAK_WARMUP:
      if (temperature >= temp_setpoint) {
        current_state    = SOAK;
        state_start_time = millis(); 
      }
      Serial.println("Soak Warmup");
      calculatePID();
      break;

    case SOAK:
      if (elapsed_time >= (unsigned long)(soak_time * 1000)) {
        current_state    = REFLOW_WARMUP;
        temp_setpoint    = reflow_setpoint;
        state_start_time = millis(); 
      }
      Serial.print("Soak Phase - Wait ");
      Serial.print(soak_time);
      Serial.println(" sec");
      calculatePID();
      break;

    case REFLOW_WARMUP:
      if (temperature >= temp_setpoint) {
        current_state    = REFLOW;
        state_start_time = millis();
      }
      Serial.println("Reflow Warmup");
      calculatePID();
      break;

    case REFLOW:
      if (elapsed_time >= (unsigned long)(reflow_time * 1000)) {
        current_state    = COOLDOWN;
        temp_setpoint    = cooldown_setpoint;
        state_start_time = millis(); 
      }
      Serial.print("Reflow Phase - Wait ");
      Serial.print(reflow_time);
      Serial.println(" sec");
      calculatePID();
      break;

    case COOLDOWN:
      if (elapsed_time >= (unsigned long)(cooldown_time * 1000)) {
        temp_setpoint    = cooldown_setpoint; 
        state_start_time = millis(); 
        running_mode     = 10; // Define your own meaning
        selected_mode    = 0; 
        idleState        = true;
        Serial.println("Cooldown Phase Complete");
        // Optionally, set running_mode to 11 for COMPLETE state
        running_mode = 11;
      }
      Serial.println("Cooldown Phase");
      // Force SSR OFF during cooldown
      analogWrite(SSR, 255);
      break;

    default:
      break;
  }
}

// ================== TUNING OR NORMAL PID CALC ======================
void calculatePID() {
  // sTune-based approach
  float optimumOutput = tuner.softPwm(SSR, Input, Output, Setpoint, outputSpan, debounce);

  switch (tuner.Run()) {
    case sTune::sample:
      Input = readTemperature();
      tuner.plotter(Input, Output, temp_setpoint, 0.5f, 1);
      break;

    case sTune::tunings:
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); 
      myPID.SetOutputLimits(0, outputSpan);
      debounce  = 0;
      setpoint  = Setpoint;
      output    = outputStep;
      kp        = Kp;
      ki        = Ki;
      kd        = Kd;
      myPID.SetTunings(kp, ki, kd);
      myPID.SetMode(AUTOMATIC);

      Serial.print("New PID Values => Kp:");
      Serial.print(Kp);
      Serial.print("  Ki:");
      Serial.print(Ki);
      Serial.print("  Kd:");
      Serial.println(Kd);
      break;

    case sTune::runPid:
      Input = readTemperature();
      // Actual compute
      myPID.Compute();
      tuner.plotter(Input, optimumOutput, temp_setpoint, 0.5f, 3);
      break;
  }

  // Final SSR output
  pwm_value = outputSpan - Output;  
  analogWrite(SSR, pwm_value);
}

// ===================== BUTTON HANDLING =============================
void handleButtons() {
  // Button 3 - Mode Selection
  bool but_3Reading = digitalRead(but_3);
  if (but_3Reading != lastBut_3State) {
    lastDebounceTime_3 = millis();
  }
  if ((millis() - lastDebounceTime_3) > debounceDelay) {
    if (but_3Reading != but_3_state) {
      but_3_state = but_3Reading;
      if (but_3_state == LOW) {
        selected_mode++;
        tone(buzzer, 2300, 40);
        if (selected_mode > max_modes) {
          selected_mode = 0;
        }
      }
    }
  }
  lastBut_3State = but_3Reading;

  // Button 4 - Mode Activation/Stop
  bool but_4Reading = digitalRead(but_4);
  if (but_4Reading != lastBut_4State) {
    lastDebounceTime_4 = millis();
  }
  if ((millis() - lastDebounceTime_4) > debounceDelay) {
    if (but_4Reading != but_4_state) {
      but_4_state = but_4Reading;
      if (but_4_state == LOW) {
        if (running_mode == 1) {
          // Cancel the process
          digitalWrite(SSR, HIGH); // OFF
          running_mode = 0;
          selected_mode = 0; 
          tone(buzzer, 2500, 150);
          delay(130);
          tone(buzzer, 2200, 150);
          delay(130);
          tone(buzzer, 2000, 150);
          delay(130);
        }

        if (selected_mode == 0) {
          running_mode = 0;
        }
        else if (selected_mode == 1) {
          running_mode = 1;
          myPID.SetMode(AUTOMATIC);
          tone(buzzer, 2000, 150);
          delay(130);
          tone(buzzer, 2200, 150);
          delay(130);
          tone(buzzer, 2400, 150);
          delay(130);
          seconds = 0;  // Reset
        }
        else if (selected_mode == 2) {
          running_mode = 2;
          if (!tuning) {
            myPID.SetMode(AUTOMATIC);
            tuning = true;
          }
          seconds = 0;  
        }
      }
    }
  }
  lastBut_4State = but_4Reading;
}

// ==================== LCD / DISPLAY UPDATES =========================
void updateDisplay(float temperature, float temp_setpoint, float pwm_value,
                   int running_mode, int selected_mode, float seconds,
                   const char* transition_phase) {

  // Line 1: Temperature
  if (temperature != prev_temperature) {
    dtostrf(temperature, 5, 1, line1);
    lcd.setCursor(0, 0);
    lcd.print("T: ");
    lcd.print(line1);
    prev_temperature = temperature;
  }

  // If we have a transition message
  if (transition_phase) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(transition_phase);
    delay(1000);
    lcd.clear();
  }

  // SSR ON/OFF display in the top row
  if (running_mode != prev_running_mode) {
    lcd.setCursor(9, 0);
    if (running_mode == 0) {
      lcd.print("SSR OFF   ");
    } else if (running_mode == 1) {
      lcd.print("SSR ON    ");
    } else if (running_mode == 10) {
      lcd.setCursor(0, 1);
      lcd.print("    COOLING    ");
    } else if (running_mode == 11) {
      lcd.setCursor(0, 1);
      lcd.print("   COMPLETE    ");
    }
    prev_running_mode = running_mode;
  }

  // If IDLE (running_mode=0), show the selected_mode on the second line
  if (running_mode == 0 && selected_mode != prev_selected_mode) {
    lcd.setCursor(0, 1);
    printSelectedMode(selected_mode);
    prev_selected_mode = selected_mode;
  }

  // If running_mode=1 => reflow, show setpoint, PWM, time
  if (running_mode == 1) {
    if (temp_setpoint != prev_temp_setpoint || pwm_value != prev_pwm_value || seconds != prev_seconds) {
      sprintf(line2, "S%d PWM%d %ds  ", int(temp_setpoint), int(pwm_value), int(seconds));
      lcd.setCursor(0, 1);
      lcd.print(line2);
      prev_temp_setpoint = temp_setpoint;
      prev_pwm_value     = pwm_value;
      prev_seconds       = seconds;
    }
  }
}

void printSelectedMode(int selected_mode) {
  lcd.print("                "); // Clear the line
  lcd.setCursor(0, 1);
  switch(selected_mode) {
    case 0:
      lcd.print("Select Mode");     
      break;
    case 1:
      lcd.print("REFLOW MODE");     
      break;
    case 2:
      lcd.print("PID TUNING");     
      break;
    default:
      lcd.print("UNKNOWN MODE");     
      break;
  }
}

// ================== TEMPERATURE READINGS ============================
float readTemperature() {
  float newTempA0 = readRawTemperature(Thermistor1_PIN);
  float newTempA1 = readRawTemperature(Thermistor2_PIN);

  if (firstRun) {
    for (int i = 0; i < HISTORY_SIZE; i++) {
      historyA0[i] = newTempA0;
      historyA1[i] = newTempA1;
    }
    firstRun = false;
  }

  float avgA0 = average(historyA0, HISTORY_SIZE);
  float avgA1 = average(historyA1, HISTORY_SIZE);

  // Outlier check
  if (abs(newTempA0 - avgA0) <= (0.2f * avgA0)) {
    historyA0[historyIndexA0] = newTempA0;
    historyIndexA0 = (historyIndexA0 + 1) % HISTORY_SIZE;
  }
  if (abs(newTempA1 - avgA1) <= (0.2f * avgA1)) {
    historyA1[historyIndexA1] = newTempA1;
    historyIndexA1 = (historyIndexA1 + 1) % HISTORY_SIZE;
  }

  // Compute final temperature with smoothing
  float finalTemp = (average(historyA0, HISTORY_SIZE) + average(historyA1, HISTORY_SIZE)) / 2.0f;
  static float previousTemp = finalTemp;
  finalTemp = 0.8f * previousTemp + 0.2f * finalTemp;
  previousTemp = finalTemp;

  return finalTemp;
}

float readRawTemperature(int pin) {
  float voltage = (analogRead(pin) / 1023.0f) * VCC;
  if ((VCC - voltage) == 0) return 9999; // Safety to avoid division by zero

  float RT   = (voltage * R) / (VCC - voltage); 
  float ln   = log(RT / RT0);
  float TX   = (1 / ((ln / B) + (1 / (25 + 273.15f))));
  TX         = TX - 273.15f; 
  return TX;
}

float average(float arr[], int size) {
  float sum = 0.0f;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}
