#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

byte ATuneModeRemember=2;
double input=80, output=50, setpoint=70;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

int raw = analogRead(A0); // Assuming the thermistor is connected to pin A0

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

// Constants for the new temperature reading method
#define HISTORY_SIZE 5
#define RT0 100000   // Ω
#define B 3950       // K
#define VCC 3.3        // Supply voltage
#define R 100000     // R=100KΩ
#define T0 298.15     // Kelvin equivalent of 25 degrees Celsius

int Thermistor1_PIN = A0;
int Thermistor2_PIN = A1;

bool firstRun = true;
float historyA0[HISTORY_SIZE];
float historyA1[HISTORY_SIZE];

int historyIndexA0 = 0;
int historyIndexA1 = 0;

//set to false to connect to the real world
boolean useSimulation = false;

const long controlInterval = 500; // half second
unsigned long onTime = map(output, 0, 255, 0, controlInterval);


void setup()
{
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);
  pinMode(4, OUTPUT);

}

void loop()
{
  int raw = analogRead(A0);
  float resistance = (VCC * R) / ((VCC * raw) / 1023.0) - R;
  float temperatureK = 1 / ((log(resistance / RT0) / B) + (1 / 298.15)); // 298.15 is room temperature in Kelvin
  float temperatureC = temperatureK - 273.15;

  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    input = readTemperature();
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  controlSSR();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}

float readTemperature() {
  // Read from both sensors
  float newTempA0 = readRawTemperature(Thermistor1_PIN);
  float newTempA1 = readRawTemperature(Thermistor2_PIN);

  // Initialize history with first reading
  if (firstRun) {
    for (int i = 0; i < HISTORY_SIZE; i++) {
      historyA0[i] = newTempA0;
      historyA1[i] = newTempA1;
    }
    firstRun = false;
  }

  // Calculate average
  float avgA0 = average(historyA0, HISTORY_SIZE);
  float avgA1 = average(historyA1, HISTORY_SIZE);

  // Check for outliers using a simpler method and ignore them if found
  if (abs(newTempA0 - avgA0) <= (0.2 * avgA0)) {
    historyA0[historyIndexA0] = newTempA0;
    historyIndexA0 = (historyIndexA0 + 1) % HISTORY_SIZE;
  }
  if (abs(newTempA1 - avgA1) <= (0.2 * avgA1)) {
    historyA1[historyIndexA1] = newTempA1;
    historyIndexA1 = (historyIndexA1 + 1) % HISTORY_SIZE;
  }

  // Compute the final temperature with simple smoothing
  float finalTemp = (average(historyA0, HISTORY_SIZE) + average(historyA1, HISTORY_SIZE)) / 2.0;
  static float previousTemp = finalTemp;
  finalTemp = 0.8 * previousTemp + 0.2 * finalTemp;
  previousTemp = finalTemp;

  return finalTemp;
}

float readRawTemperature(int pin) {
  float voltage = (analogRead(pin) / 1023.0) * VCC;
  float RT = (voltage * R) / (VCC - voltage); 

  float ln = log(RT / RT0);
  float TX = (1 / ((ln / B) + (1 / T0))); 
  TX = TX - 273.15; // Convert Kelvin to Celsius

  return TX;
}

float average(float arr[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}

void controlSSR() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  
  onTime = map(output, 0, 255, 0, controlInterval);

  if (currentMillis - previousMillis >= controlInterval) {
    previousMillis = currentMillis;
  }
  else if (currentMillis - previousMillis < onTime) {
    digitalWrite(4, LOW); // Turn on the SSR
  }
  else {
    digitalWrite(4, HIGH); // Turn off the SSR
  }
}
