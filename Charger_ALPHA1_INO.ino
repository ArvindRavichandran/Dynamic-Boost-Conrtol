#include <Servo.h>
#include <LiquidCrystal.h>


//#############        Value Definitions                  #############

#define MAX_SIGNAL 2250     //Max pulse width for super motor ESC
#define MIN_SIGNAL 650
#define NOM_SIGNAL 1450     //Min Throttle pulse width
#define MOTOR_PIN 14        //Super motor pin
#define MAX_INIT_TIME 2000  //Initiation time during which MAX pulse width is supplied
#define MIN_INIT_TIME 5000
#define IGN_TIMEOUT 2000    //Timeout (With Clutch and E-Brakes ON) before the ignition cuts-off
#define IGN_OFF_HOLD 2000   //Time for which the IGNITION id held OFF to prevent damage to the engine
#define STR_ON_HOLD 1500    //TIME for which the STARTER is held on to get the engine running
#define IGN_REFRESH 1000    //Refresh interval for ignition state read
#define ACC_IGN_REF 1000    //Accidental Ignition Refresh avoidance refresh time


//#############        IO Declarations                     #############

const int THROTTLE_PIN = A0;
const int SUPER_POWER_BUTTON = 22;
const int ESC_PIN = 23;
const int AUTO_IGN_BUTTON = 24;
const int IGNITION_PIN = 25;
const int IGN_OUT_PIN = 26;
const int STR_OUT_PIN = 27;
const int CLUTCH_PIN = 28;
const int E_BRAKE_PIN = 29;
//THROTTLE INTERRUPT PIN = 30

//#############        Hardware Definitions               #############

LiquidCrystal MONITOR(51, 50, 11, 10, 9, 8);
Servo MOTOR;


//#############        Global Variable Declarations       #############

bool SUPER_STATE;     //Supercharger ON/OFF state
volatile byte rpm_count = 0;
unsigned int TACHO = 0;
int THROTTLE = 0;
unsigned long rpm_refresh_time = 0;
int Superon = 0;            //Supercharger ON state loop control variable
int Superoff = 0;           //Supercharger OFF state loop control variable
int Auto_Ign_Init = 0;      //Auto Ignition System initiation variable
int SUPER_INITIAL_TIME = 0; //Supercharger initiation variable
int Ign_Off_Init = 0;       //Ignition OFF process initiation variable
unsigned long IGN_OFF_TIME = 0;       //Ignition OFF timeout variable
int Ign_On_Init = 0;        //Ignition ON process initiation variable
unsigned long STR_ON_TIME = 0;        //Starter ON timeout variable
int CLUTCH_COUNT = 0;       //Clutch change count at off condition
bool CLUTCH_2 = HIGH;       //Clutch Signal copy for change detection
int EBRAKE_COUNT = 0;       //E-Brake change count at off condition
bool EBRAKE_2 = HIGH;       //E-Brake Signal copy for change detection
unsigned long IGN_REF_TIME = 0;
int Ign_Refresh = 0;

//#############        RPM ISR Function                   #############

void rpm_read()
{
  rpm_count++;
}

void setup()
{
  MOTOR.attach(MOTOR_PIN);
  pinMode(SUPER_POWER_BUTTON, INPUT);
  pinMode(ESC_PIN, OUTPUT);
  pinMode(AUTO_IGN_BUTTON, INPUT);
  pinMode(IGNITION_PIN, INPUT);
  pinMode(IGN_OUT_PIN, OUTPUT);
  pinMode(CLUTCH_PIN, INPUT);
  pinMode(E_BRAKE_PIN, INPUT);

  MONITOR.begin(16, 2);
  MONITOR.cursor();
  MONITOR.clear();
digitalWrite(STR_OUT_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(21), rpm_read, RISING);
  
}


void loop()
{

  //#############        Supercharger Control Program       #############

  SUPER_STATE = digitalRead(SUPER_POWER_BUTTON);
  digitalWrite(ESC_PIN, ~SUPER_STATE);
  detachInterrupt(digitalPinToInterrupt(30));
  TACHO = (15*1000/(millis()- rpm_refresh_time))*rpm_count;
  rpm_refresh_time = 0;
  rpm_count = 0;
  attachInterrupt(digitalPinToInterrupt(30), rpm_read, RISING);
  if (SUPER_STATE == HIGH)
  {
    THROTTLE = analogRead(THROTTLE_PIN);
    if (Superon == 0)
    {
      SUPER_INITIAL_TIME = millis();
      Superoff = 0;
      Superon++;
    }
    if (millis() <= SUPER_INITIAL_TIME + MAX_INIT_TIME)
    {
      MOTOR.writeMicroseconds(MAX_SIGNAL);
    }
    else if (millis() <= SUPER_INITIAL_TIME + MIN_INIT_TIME + MAX_INIT_TIME)
    {
      MOTOR.writeMicroseconds(MIN_SIGNAL);
    }
    else
    {
      int RPM = NOM_SIGNAL;
      RPM = map(THROTTLE, 0, 1023, NOM_SIGNAL, MAX_SIGNAL + 20);
      int OVERSHOOT = 30;

      if (THROTTLE > TACHO)
      {
        int DIFFERENCE = THROTTLE - TACHO;
        OVERSHOOT = map(DIFFERENCE, 0, 1023, 50, MAX_SIGNAL - NOM_SIGNAL);
        RPM = RPM + OVERSHOOT;

        MONITOR.leftToRight();
        MONITOR.setCursor(6, 1);
        MONITOR.print(DIFFERENCE);

        if (RPM > MAX_SIGNAL)
        {
          RPM = MAX_SIGNAL;
        }
      }
      MOTOR.writeMicroseconds(RPM);

      MONITOR.leftToRight();
      MONITOR.setCursor(3, 0);
      MONITOR.print(THROTTLE);

      MONITOR.leftToRight();
      MONITOR.setCursor(0, 1);
      MONITOR.print(OVERSHOOT);

      MONITOR.leftToRight();
      MONITOR.setCursor(7, 0);
      MONITOR.print("--");

      MONITOR.leftToRight();
      MONITOR.setCursor(5, 1);
      MONITOR.print("=");


      MONITOR.rightToLeft();
      MONITOR.setCursor(12, 0);
      MONITOR.print(TACHO);

      MONITOR.rightToLeft();
      MONITOR.setCursor(15, 1);
      MONITOR.print(RPM);
    }
  }

  else
  {
    if (Superoff == 0)
    {
      MOTOR.writeMicroseconds(NOM_SIGNAL);
      Superon = 0;
      Superoff++;
    }
  }


  //#############        Auto-Ignition Control Program      #############
if(digitalRead(AUTO_IGN_BUTTON) == HIGH)
{
  bool IGNITION;
  bool IGNITION_2;
  bool E_BRAKE;
  bool CLUTCH;

  IGNITION_2 = digitalRead(IGNITION_PIN);
  if (IGNITION != IGNITION_2 )
  {
    if (Ign_Refresh == 0)
    {
      IGN_REF_TIME = millis();
      Ign_Refresh++;
    }
    if (IGN_REF_TIME + IGN_REFRESH + ACC_IGN_REF > millis() >= IGN_REF_TIME + IGN_REFRESH )
    {
      IGNITION = IGNITION_2;
      Ign_Refresh = 0;
    }
    else if (millis() > IGN_REF_TIME + IGN_REFRESH + ACC_IGN_REF)
    {
      Ign_Refresh = 0;
    }
  }

  E_BRAKE = digitalRead(E_BRAKE_PIN);
  CLUTCH = digitalRead(CLUTCH_PIN);

  if (Auto_Ign_Init == 1 || (CLUTCH == HIGH && E_BRAKE == HIGH))
  {
    Auto_Ign_Init = 1;

    if (CLUTCH != CLUTCH_2 && IGNITION == LOW)
    {
      CLUTCH_COUNT++;
      CLUTCH_2 = CLUTCH;
      EBRAKE_COUNT = 0;
    }
    else if (E_BRAKE != EBRAKE_2 && IGNITION == HIGH)
    {
      EBRAKE_COUNT++;
      EBRAKE_2 = E_BRAKE;
      CLUTCH_COUNT = 0;
    }
    if (IGNITION == CLUTCH == E_BRAKE == HIGH && CLUTCH_COUNT == 0 && EBRAKE_COUNT % 2 == 0 && EBRAKE_COUNT != 0)
    {
      if (Ign_Off_Init == 0)
      {
        IGN_OFF_TIME = millis();
        Ign_Off_Init++;
      }
      if (IGN_OFF_TIME + IGN_TIMEOUT < millis() < IGN_OFF_TIME + IGN_TIMEOUT + IGN_OFF_HOLD)
      {
        digitalWrite(IGN_OUT_PIN, HIGH);
        Ign_On_Init = 0;
      }
      else if (millis() >= IGN_OFF_TIME + IGN_TIMEOUT + IGN_OFF_HOLD)
      {
        Ign_Off_Init = 0;
      }
    }
    else if (IGNITION == LOW && CLUTCH == HIGH && CLUTCH_COUNT % 2 == 0 && CLUTCH_COUNT != 0 && THROTTLE > 500)
    {
      digitalWrite(IGN_OUT_PIN, LOW);
      if (Ign_On_Init == 0)
      {
        STR_ON_TIME = millis();
        Ign_On_Init++;
      }
      while ( millis() < STR_ON_TIME + STR_ON_HOLD && digitalRead(CLUTCH_PIN) == HIGH)
      {
        digitalWrite(STR_OUT_PIN, LOW);
        MOTOR.writeMicroseconds(NOM_SIGNAL);
        Ign_Off_Init = 0;
      }
      if (millis() >= STR_ON_TIME + STR_ON_HOLD)
      {
        Ign_On_Init = 0;
      }
    }
    else
    {
      digitalWrite(IGN_OUT_PIN, LOW);
      digitalWrite(STR_OUT_PIN, HIGH);
    }
  }
}
else
{
  Auto_Ign_Init = 0;
  digitalWrite(IGN_OUT_PIN, LOW);
  digitalWrite(STR_OUT_PIN, HIGH);
}
}
