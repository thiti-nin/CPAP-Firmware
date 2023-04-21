#include <PID_v1.h>
#include <sfm3000wedo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HoneywellTruStabilitySPI.h>

SFM3000wedo measflow(64);

#define SLAVE_SELECT_PIN SS
TruStabilityPressureSensor sensor(SLAVE_SELECT_PIN, -1.0, 1.0);
LiquidCrystal_I2C lcd(0x27, 20, 4);
double Setpoint, Input, Output, flow_hp, flow_bp, flow_bp2, lastOutput; // variables
double Kp = 10, Ki = 3, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


int offset = 32000;
float scale = 140.0, brRatio, inhalePeriod, exhalePeriod;
double Flow, sensorPres, totalzeroFlow, totalzeroPres, totalPres, totalOutput, totalFlow, ledVal, filtered, caliPeak;
double averagezeroFlow, averagezeroPres, calibzeroFlow, calibzeroPres, averagePres, averageFlow, averageOutput;
unsigned int result;
int samplePres[25];
int sampleFlow[25];
int apneaHour[50];
int apneaMinute[50];
int i, j, j1 = -1, k =0,l, bipapSetpoint, bpm, breathNum, peak, sampling, peakFlow;
float fc_hp = 0.125; // cutoff frequency of the high-pass filter
float fc_lp = 1.125; // cutoff frequency of the low-pass filter
float RC_hp = 1 / (2 * PI * fc_hp);
float RC_lp = 1 / (2 * PI * fc_lp);
double data_filt_hp[] = {0, 0};
double data_hp[] = {0, 0};
double data_filt_lp[] = {0, 0};
double data_lp[] = {0, 0};
double maximo = 0, vmax = 0, minima = 0, maxFlow, minFlow, minPres, maxPres, PEF, inhaleSec, exhaleSec, PEEP1, totalPeak, avePeak, peakVal;

const int motorPin = 3, ledPin = 9, soundsen = A0;
const int powPin = 6, incrPin = 4, decrPin = 5;

unsigned long t0 = micros(), time1 = millis(), time2, timeCount, timeCpap, iTime, eTime;
unsigned long inhaleTime, lastinhaleTime, exhaleTime, lastexhaleTime, apneaMill;
int samplingTime = 10000, presSet, cpapState, bipapState, inhaleEvent, lastinhaleEvent, exhaleEvent, lastexhaleEvent, totalBreath = 0, maximaNumber, inhaling, exhaling;
int ipap = 8, epap = 4, once = 0, once2 = 0, lastI, apneaPulse, minutePassed = 0, thirtyMinPassed = 0, fiveMinute = 0;
int setPres = 10, displayPres = 4, apnea = 0, middle, hour = 0, lastbreathing, breathing, apneaState,lastApnea,oneloop, lastapneaState;
int lastreadPow, power, powState, lastreadIncr, incrState, increase, lastreadDecr, decrState, decrease, cursorPos, mode = 0, bipapStart = 0, brPeriod, trough;
float timing0 = 0, apneaTime, threshold, minThreshold;

char sensorstringFlow[4];
char sensorstringPres[4];
double presAve;
int breathAverage[25];
byte upanddown[] = {
    B00100,
    B01110,
    B11111,
    B00000,
    B00000,
    B11111,
    B01110,
    B00100};
byte up[] = {
    B00100,
    B01110,
    B11111,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000};
byte down[] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B11111,
    B01110,
    B00100};
byte b[] = {
    B00000,
    B00000,
    B01100,
    B01010,
    B01100,
    B01010,
    B01100,
    B00000};
byte c[] = {
    B00000,
    B00000,
    B01110,
    B01000,
    B01000,
    B01000,
    B01110,
    B00000};
byte load[] = {
    B11111,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B11111};
byte moon[] = {
    B11100,
    B00110,
    B00011,
    B00011,
    B00011,
    B01110,
    B11100,
    B00000};
byte filled[] = {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111};

//===================================================================================================================
//                                                    SET UP
//===================================================================================================================
void setup()
{
  myPID.SetMode(AUTOMATIC);
  Wire.begin();
  Serial.begin(115200);
  sensor.begin();
  SPI.begin();
  measflow.init();
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, upanddown);
  lcd.createChar(1, up);
  lcd.createChar(2, down);
  lcd.createChar(3, b);
  lcd.createChar(4, c);
  lcd.createChar(5, load);
  lcd.createChar(6, moon);
  lcd.createChar(7, filled);
  pinMode(powPin, INPUT_PULLUP);
  pinMode(incrPin, INPUT_PULLUP);
  pinMode(decrPin, INPUT_PULLUP);
  pinMode(motorPin, OUTPUT);

  lcd.setCursor(7, 1);
  lcd.print("somna");
  lcd.setCursor(8, 1);
  lcd.write(byte(6));
  delay(200);

  //------------------------------------ Finding Serial availability -----------------------------------------
  while (!Serial)
  {
    lcd.setCursor(7, 1);
    lcd.print("somna");
    lcd.setCursor(8, 1);
    lcd.write(byte(6));
    i++;

    if (i == 0 || i == 6 || i == 12 || i == 18 || i == 24 || i == 30 || i == 36 || i == 42 || i == 48 || i == 54 || i == 60 || i == 66 || i == 72 || i == 78 || i == 84 || i == 90 || i == 96 || i == 102 || i == 108 || i == 114)
    {
      lcd.setCursor(i / 6, 2);
      lcd.write(byte(7));
      delay(200);
    }

    if (i > 113)
    {
      i = 0;
      lcd.clear();
      lcd.setCursor(1, 1);
      lcd.print("Please Check Your");
      lcd.setCursor(1, 2);
      lcd.print("Connection . . . ");
      delay(2000);
    }
  }

  delay(2000);
  lcd.clear();
}

//===================================================================================================================
//                                                    LOOP
//===================================================================================================================
void loop()
{


  // if (millis() - iTime < 2100)
  // {
  //   Output = 115;
  // }
  // if (millis() - iTime > 2100)
  // {
  //   Output = 0;
  // }
  // if (millis() - iTime > 6000)
  // {
  //   iTime = millis();
  // }

  // ------------------------- Track Apnea at Hour/Minute ------------------------------------
  if (avePeak == 0)
  {

    apnea = 0;
    time2 = millis();
    timeCount = millis();
  }

  if (avePeak > 0)
  {

    if (apneaState == LOW && j1 > -1)     //Keep events in array in the form of Hour:Minute
    {

      apneaHour[j1] = thirtyMinPassed;
      apneaMinute[j1] = minutePassed;
    }


    if (millis() - timeCount > 60000)   //Time count 1 minute
    {
      minutePassed++;
      timeCount = millis();

      if (minutePassed > 29)
      {
        thirtyMinPassed++;
      }

      if (thirtyMinPassed > 1 )
      {
        thirtyMinPassed = 0;
        minutePassed = 0;
        for(l=0; l<49; l++)
        {
          Serial.print(apneaHour[l]);
          Serial.print(":");
          Serial.print(apneaMinute[l]);
          Serial.print(",");
        }
        Serial.println(" ");
      }
      
    }


    // ------------------------- Increase Prssure 1cmH20 / 1 hour if apnea >=2 ------------------------------------

    if (hour == 0)
    {
      if (minutePassed - fiveMinute > 4)        //check for 5 minutes
      {
        time2 = millis();
        fiveMinute = minutePassed;
        if (apnea - lastApnea >= 2)
        {
          hour = 0;

          if (cpapState == 1)         //If Cpap is running
          {
            displayPres++;            //Increase pressure by 1 cmH2o
            setPres++;
          }
          if (bipapState == 1)
          {
            ipap++;
          }
        }

        if (lastApnea > 2 && apnea -lastApnea < 2)
        {
          oneloop ++;
        }
        if(oneloop > 1 && apnea - lastApnea >2)
        {
          oneloop = 0;
        }
        lastApnea = apnea;
      }
    }

  //   if (sixloop > 5){
      
  //   }
  //   // ------------------------- Increase Pressure 1cmH20 / 5 min if apnea>=2 ------------------------------------
  //   if (hour == 1)
  //   {
  //     if (minutePassed - fiveMinute > 4)
  //     {
  //       fiveMinute = minutePassed;

  //       if (apnea >= 2)
  //       {
  //         time2 = millis();
  //         // displayPres++;
  //         // setPres++;
  //         ipap++;
  //       }

  //       if (apnea < 2)
  //       {
  //         hour = 0;
  //       }
  //       apnea = 0;
  //     }
  //   }
  }

  // ------------------------- Set threshold for Peak Detection ------------------------------------
  threshold = 0.3 * avePeak;

  if (threshold < 4)
  {
    threshold = 4;
  }


  // ------------------------- button Functions ------------------------------------------------------------
  button();

  // ------------------------- Menu -------------------------------------------------------------------------

  if (avePeak > 0)
  {
    if (mode == 0)
    {
      menu();
    }
    else if (mode == 1)
    {
      cpap();
    }
    else if (mode == 2)
    {
      bipap();
    }
    else if (mode == 3)
    {
      stats();
    }
    else if (mode == 4)
    {
      pressureSet();
    }
    else if (mode == 5)
    {
      ipapSet();
    }
    else if (mode == 6)
    {
      epapSet();
    }
    else if (mode == 7)
    {
      cpapStat();
    }
    else if (mode == 8)
    {
      cpapStat2();
    }
  }

  // ------------------------- Calibrated for average Peak 10 times ------------------------------------
  else
  {
    lcd.setCursor(5, 0);
    lcd.print("Calibrating...");
    lcd.setCursor(8, 2);
    lcd.print(i);
    lcd.setCursor(10, 2);
    lcd.print("/10");
  }


  // ------------------------- ICON at top right corner ------------------------------------
  if (bipapState == 1)
  {
    lcd.setCursor(19, 0);
    lcd.write(byte(3));
  }

  if (cpapState == 1)
  {
    lcd.setCursor(19, 0);
    lcd.write(byte(4));
  }

  if (cpapState == 0 && bipapState == 0)
  {
    lcd.setCursor(19, 0);
    lcd.print(" ");
  }


  //============================================================================================================================
  //                                                    Sampling Time = 10000 ms
  //============================================================================================================================

  if (micros() - (t0) >= samplingTime)
  {

    timing0 = (micros() - t0);
    t0 = micros();

    // ------------------------- Analyze inhaleTime, ExhaleTime, BPM, etc. ------------------------------------
    if (avePeak > 0)
    {

      if (flow_bp > threshold)
      {

        inhaleTime = millis();
        brPeriod = inhaleTime - lastinhaleTime;
        lastexhaleTime = exhaleTime;
        inhalePeriod = inhaleTime - exhaleTime;
        exhalePeriod = brPeriod - inhalePeriod;
        brRatio = inhalePeriod / exhalePeriod;

        inhaleEvent = HIGH;
        breathing = 10;
      }

      else
      {
        exhaleTime = millis();
        lastinhaleTime = inhaleTime;
        breathing = 0;

        inhaleEvent = LOW;
        inhaling = 0;
        exhaling = 15;
      }

      // ------------------------- Track Apnea using above data ------------------------------------

      if (peakVal < (0.85 * caliPeak))
      {
        apneaMill = millis();
      }

      if (millis() - apneaMill >= 10000)
      {
        apneaState = HIGH;
      }

      else
      {
        apneaState = LOW;
      }

      if (apneaState == HIGH && lastapneaState == LOW)
      {
        apnea++;
        j1++;
      }

      lastapneaState = apneaState;

      if (inhaleEvent == HIGH && lastbreathing == LOW)
      {
        breathNum++;
      }
    }

    // ------------------------- Convert ms to seconds ------------------------------------
    inhaleSec = inhalePeriod / 1000;
    exhaleSec = exhalePeriod / 1000;
    lastbreathing = inhaleEvent;

    // ------------------------- BPM ------------------------------------
    if (breathNum >= 1)
    {
      if (millis() - timeCpap > 60000)
      {
        timeCpap = millis();
        bpm = breathNum;
        breathNum = 0;
      }
    }

    // ------------------------- Get flow/pressure signal ------------------------------------
    result = measflow.getvalue();
    Flow = ((float)result - offset) / scale;

    if (sensor.readSensor() == 0)
    {
      sensorPres = 70.307 * sensor.pressure();            //convert psi to cmH2O
      dtostrf(sensorPres, 2, 1, sensorstringPres);
    }

    // ------------------------- Checking Sampling Time of signal ------------------------------------
    //   sampling += 1;
    //  if(sampling == 1){
    //   iTime = millis();
    //   lcd.setCursor(11,3);
    //   lcd.print(iTime);
    //  }
    //   if(sampling == 1000){
    //     eTime = millis();
    //     lcd.setCursor(11,2);
    //     lcd.print(eTime);
    //   }

    // ------------------------- Averaged the pressure detected ------------------------------------
    if (avePeak > 0)
    {

      totalPres += sensorPres;
      j++;
      if (j > 49)
      {

        averagePres = totalPres / 50;
        j = 0;
        totalPres = 0;
      }
    }

    // ------------------------- Filtering signal(bandpass) ------------------------------------

    flow_hp = mifiltroHP(Flow, timing0 / 1000000, RC_hp);
    flow_bp = mifiltroLP(flow_hp, timing0 / 1000000, RC_lp);            //Use for controlling, Display used raw flow


    // ------------------------- PID control and optimization ------------------------------------

    if (cpapState == 1)
    {
      Input = sensorPres;
      Setpoint = displayPres;
      myPID.Compute();
      
      

      if (k == 0)
      {
        switch (displayPres)
          {
          case 4:
            averageOutput = 47;
            break;
          case 8:
            averageOutput = 95;
            break;
          case 10:
            averageOutput = 111;
            break;
          case 12:
            averageOutput = 132;
            break;
          case 15:
            averageOutput = 160;
            break;

          default:
            averageOutput = 0;
            break;
          }
        analogWrite(motorPin, 115);
        // if (displayPres > 20)
        if (displayPres - sensorPres <= 0.01 * displayPres)
        {
          k++;

            // 4=45 8=95 10=115 12 =132 15 = 160
        }
      }
      if (k > 0)
      {
        analogWrite(motorPin, averageOutput);
        if (averageOutput - Output > 15)
        {
          averageOutput --;
        }
        if(Output - averageOutput > 15)
        {
          averageOutput++;
        }
      
      }
    }

//-----------------------------------------------BiPAP-------------------------------------------------------------
    if (bipapState == 1)
    {
      Input = sensorPres;
      Setpoint = bipapSetpoint;
      myPID.Compute();

      bipapStart++;

      analogWrite(motorPin, Output);

      if (bipapStart <= 10)
      {
        Input = sensorPres;
        Setpoint = epap;
        myPID.Compute();
        analogWrite(motorPin, Output);
      }
    }

    if (mode == 1)
    {
      if (cpapState == 0)
      {
        Output = 0;
        analogWrite(motorPin, Output);
        k = 0;
      }
    }

    if (mode == 2)
    {
      if (bipapState == 0)
      {
        Output = 0;
        analogWrite(motorPin, Output);
      }
    }

    // ------------------------- calculate Peak value via max/min ------------------------------------
    if (flow_bp > threshold)
    {
      // maxFlow = 0;
      time1 = millis();
      maximo = max(maximo, flow_bp);
      maxFlow = max(maxFlow, flow_bp);
      // peakFlow = maxFlow;
      trough = 0;
    }

    else if (flow_bp < minThreshold)
    {
      minima = min(minima, flow_bp);
      minThreshold = 0.2 * minima;
      maxPres = max(maxPres, sensorPres);
      minFlow = min(minFlow, minima);
      PEF = minFlow * (-1);
      PEEP1 = min(PEEP1, sensorPres);
      trough = 1;
    }

    // ------------------------- Finding averagePeak/breath cycle ------------------------------------
    else if (flow_bp < 0)
    {
      peakVal = maximo;

      if (peakVal > threshold)
      {
        i++;
      }

      if (i > lastI)
      {
        totalPeak += peakVal;
      }

      if (i > 9)
      {

        if (avePeak < 1)
        {
          avePeak = totalPeak / 10;
          caliPeak = avePeak;
          lcd.setCursor(5, 3);
          lcd.print(avePeak);
          lcd.clear();
        }

        avePeak = totalPeak / 10;
        totalPeak = 0;
        i = 0;
      }
      // ------------------------- Reset value/breath cycle -----------------------------------------------
      lastI = i;
      maximo = 0;
      minima = 0;
      vmax = 0;
    }

    // ------------------------- Serial Print -----------------------------------------------------------
    SerialPrint();
  }

  increase = 0;
  decrease = 0;
  power = 0;
  bipapSetpoint = epap;
}

//=========================================================================================================================
//                                                           FUNCTIONS
//=========================================================================================================================

// ------------------------------------------ Serial Print -----------------------------------------------------------
void SerialPrint()
{

  Serial.print(flow_bp);
  Serial.print(",");
  Serial.print(displayPres);
  Serial.print(",");
  Serial.print(apnea);
  Serial.print(",");
  Serial.print(lastApnea);
  Serial.print(",");
  Serial.println(sensorPres);
}

// ------------------------------------------ BUTTON -----------------------------------------------------------------
void button()
{
  powState = digitalRead(powPin);

  if (lastreadPow == HIGH && powState == LOW)
  {
    power = 1;
  }
  incrState = digitalRead(incrPin);
  if (lastreadIncr == HIGH && incrState == LOW)
  {
    increase = 1;
  }
  decrState = digitalRead(decrPin);
  if (lastreadDecr == HIGH && decrState == LOW)
  {
    decrease = 1;
  }

  lastreadPow = powState;
  lastreadIncr = incrState;
  lastreadDecr = decrState;
}

// ------------------------------------------ MODE 0 -----------------------------------------------------------------
void menu()
{
  lcd.setCursor(0, 0);
  lcd.print("MODE SELECTION");
  lcd.setCursor(1, 1);
  lcd.print("CPAP");
  lcd.setCursor(1, 2);
  lcd.print("BiPAP");
  lcd.setCursor(1, 3);
  lcd.print("Statistics");

  lcd.setCursor(0, cursorPos);
  lcd.print(">");

  if (decrease == 1)
  {

    decrease = 0;
    cursorPos++;
    lcd.setCursor(0, cursorPos - 1);
    lcd.print(" ");
  }

  if (increase == 1)
  {

    increase = 0;
    cursorPos--;
    lcd.setCursor(0, cursorPos + 1);
    lcd.print(" ");
  }

  if (cursorPos > 3 || cursorPos == 0)
  {
    cursorPos = 1;
  }

  if (power == 1)
  {
    power = 0;
    mode = cursorPos;
    cursorPos = 0;
    lcd.clear();
  }
}

// ------------------------------------------ MODE 1 -----------------------------------------------------------------
void cpap()
{
  lcd.setCursor(1, 0);
  lcd.print("CPAP MODE: ");
  lcd.setCursor(1, 2);
  lcd.print("PRESSURE: ");
  lcd.setCursor(11, 2);
  lcd.print(displayPres);
  lcd.setCursor(14, 2);
  lcd.print("cmH20");
  lcd.setCursor(1, 3);
  lcd.print("BACK");

  if (cpapState == 1)
  {
    lcd.setCursor(13, 0);
    lcd.print("ON ");
  }
  else
  {
    lcd.setCursor(13, 0);
    lcd.print("OFF");
  }

  lcd.setCursor(0, cursorPos);
  lcd.print(">");

  if (decrease == 1)
  {
    decrease = 0;
    cursorPos++;
    lcd.setCursor(0, cursorPos - 1);
    lcd.print(" ");
  }

  if (increase == 1)
  {
    increase = 0;
    cursorPos--;
    lcd.setCursor(0, cursorPos + 1);
    lcd.print(" ");
  }

  if (cursorPos > 3)
  {
    cursorPos = 0;
  }

  if (power == 1)
  {
    power = 0;

    if (cursorPos == 0)
    {
      cpapState = !cpapState;
      bipapState = 0;
    }
    if (cursorPos == 1)
    {
      mode = 1;
    }
    if (cursorPos == 2)
    {
      mode = 4;
      cpapState = 0;
    }
    if (cursorPos == 3)
    {
      mode = 0;
      lcd.clear();
      cursorPos = 0;
    }
  }
}

// ------------------------------------------ MODE 2 -----------------------------------------------------------------
void bipap()
{
  lcd.setCursor(1, 0);
  lcd.print("BiPAP MODE: ");
  lcd.setCursor(1, 1);
  lcd.print("IPAP: ");
  lcd.setCursor(7, 1);
  lcd.print(ipap);
  lcd.setCursor(1, 2);
  lcd.print("EPAP: ");
  lcd.setCursor(7, 2);
  lcd.print(epap);
  lcd.setCursor(1, 3);
  lcd.print("BACK");
  lcd.setCursor(0, cursorPos);
  lcd.print(">");

  if (bipapState == 1)
  {
    lcd.setCursor(13, 0);
    lcd.print("ON ");
  }
  else
  {
    lcd.setCursor(13, 0);
    lcd.print("OFF");
    bipapStart = 0;
  }

  if (decrease == 1)
  {
    decrease = 0;
    cursorPos++;
    lcd.setCursor(0, cursorPos - 1);
    lcd.print(" ");
  }
  if (increase == 1)
  {
    increase = 0;
    cursorPos--;
    lcd.setCursor(0, cursorPos + 1);
    lcd.print(" ");
  }
  if (cursorPos > 3)
  {
    cursorPos = 0;
  }
  if (power == 1)
  {
    power = 0;

    if (cursorPos == 0)
    {
      bipapState = !bipapState;
      cpapState = 0;
    }
    if (cursorPos == 1)
    {
      mode = 5;
      bipapState = 0;
    }
    if (cursorPos == 2)
    {
      mode = 6;
      bipapState = 0;
    }
    if (cursorPos == 3)
    {
      mode = 0;
      lcd.clear();
      cursorPos = 0;
    }
  }
}

// ------------------------------------------ MODE 3 -----------------------------------------------------------------
void stats()
{
  lcd.setCursor(1, 0);
  lcd.print("CPAP Stats");
  lcd.setCursor(1, 1);
  lcd.print("BiPAP Stats");
  lcd.setCursor(1, 3);
  lcd.print("BACK");
  lcd.setCursor(0, cursorPos);
  lcd.print(">");

  if (decrease == 1)
  {
    decrease = 0;
    cursorPos++;
    lcd.setCursor(0, cursorPos - 1);
    lcd.print(" ");
  }

  if (increase == 1)
  {
    increase = 0;
    cursorPos--;
    lcd.setCursor(0, cursorPos + 1);
    lcd.print(" ");
  }

  if (cursorPos > 3)
  {
    cursorPos = 0;
  }

  if (power == 1)
  {
    power = 0;

    if (cursorPos == 0)
    {
      mode = 7;
      lcd.clear();
      cursorPos = 0;
    }
    if (cursorPos == 1)
    {
      mode = 3;
    }
    if (cursorPos == 3)
    {
      mode = 0;
      lcd.clear();
      cursorPos = 0;
    }
  }
}

// ------------------------------------------ MODE 7 -----------------------------------------------------------------
void cpapStat()
{
  lcd.setCursor(1, 0);
  lcd.print("CPAP Stats");
  lcd.setCursor(1, 1);
  lcd.print("Ti: ");
  lcd.setCursor(10, 1);
  lcd.print("PRES: ");
  lcd.setCursor(16, 1);
  lcd.print(" ");
  lcd.setCursor(5, 2);
  lcd.print(inhaleSec);
  lcd.setCursor(1, 2);
  lcd.print("Te: ");
  lcd.setCursor(5, 1);
  lcd.print(exhaleSec);
  lcd.setCursor(1, 3);
  lcd.print("Flow:");
  lcd.setCursor(7, 3);
  lcd.print(flow_bp);
  lcd.setCursor(0, cursorPos);
  lcd.print(">");

  if (decrease == 1)
  {
    decrease = 0;
    cursorPos++;
    lcd.setCursor(0, cursorPos - 1);
    lcd.print(" ");
  }
  if (increase == 1)
  {
    increase = 0;
    cursorPos--;
    lcd.setCursor(0, cursorPos + 1);
    lcd.print(" ");
  }

  if (cursorPos > 3)
  {
    cursorPos = 0;
    mode = 8;
    lcd.clear();
  }
}

// ------------------------------------------ MODE 8 -----------------------------------------------------------------
void cpapStat2()
{
  lcd.setCursor(1, 0);
  lcd.print("PIP: ");
  lcd.setCursor(6, 0);
  lcd.print(maxPres);
  lcd.setCursor(11, 0);
  lcd.print("BPM: ");
  lcd.setCursor(17, 0);
  lcd.print(bpm);

  if (bpm < 10)
  {
    lcd.setCursor(18, 0);
    lcd.print(" ");
  }

  lcd.setCursor(11, 1);
  lcd.print("AHI: ");
  lcd.setCursor(17, 1);
  lcd.print(apnea);
  lcd.setCursor(1, 1);
  lcd.print("PIF: ");
  lcd.setCursor(6, 1);
  lcd.print(maxFlow);
  lcd.setCursor(1, 2);
  lcd.print("PEF: ");
  lcd.setCursor(6, 2);
  lcd.print(PEF);
  lcd.setCursor(1, 3);
  lcd.print("BACK");
  lcd.setCursor(0, cursorPos);
  lcd.print(">");
  if (decrease == 1)
  {
    decrease = 0;
    cursorPos++;
    lcd.setCursor(0, cursorPos - 1);
    lcd.print(" ");
  }

  if (increase == 1)
  {
    increase = 0;
    cursorPos--;
    lcd.setCursor(0, cursorPos + 1);
    lcd.print(" ");
  }

  if (cursorPos > 3)
  {
    mode = 7;
    cursorPos = 0;
    lcd.clear();
  }

  if (power == 1)
  {
    power = 0;

    if (cursorPos == 3)
    {
      mode = 3;
      lcd.clear();
      cursorPos = 0;
    }
  }

  if (breathNum < 10)
  {
    lcd.setCursor(7, 0);
    lcd.print(" ");
  }
}

// ------------------------------------------ MODE 4 -----------------------------------------------------------------
void pressureSet()
{
  lcd.setCursor(1, 0);
  lcd.print("CPAP MODE: ");
  lcd.setCursor(1, 2);
  lcd.print("PRESSURE: ");
  lcd.setCursor(11, 2);
  lcd.print(displayPres);
  lcd.setCursor(11, 1);
  lcd.write(byte(1));
  lcd.setCursor(11, 3);
  lcd.write(byte(2));

  if (displayPres <= 4)
  {
    lcd.setCursor(11, 3);
    lcd.print(" ");
  }

  if (displayPres >= 20)
  {
    lcd.setCursor(11, 1);
    lcd.print(" ");
  }

  lcd.setCursor(14, 2);
  lcd.print("cmH20");
  lcd.setCursor(1, 3);
  lcd.print("BACK");
  lcd.setCursor(0, cursorPos);
  lcd.print(" ");

  if (cpapState == 1)
  {
    lcd.setCursor(13, 0);
    lcd.print("ON ");
  }
  else
  {
    lcd.setCursor(13, 0);
    lcd.print("OFF");
  }

  if (increase == 1)
  {
    increase = 0;
    displayPres++;
  }

  if (decrease == 1)
  {
    decrease = 0;
    displayPres--;
  }

  if (displayPres >= 20)
  {
    displayPres = 20;
  }

  if (displayPres <= 4)
  {
    displayPres = 4;
  }

  if (displayPres < 10)
  {
    lcd.setCursor(12, 2);
    lcd.print(" ");
  }

  if (power == 1)
  {
    power = 0;
    mode = 1;
    lcd.clear();
  }
}

// ------------------------------------------ MODE 5 -----------------------------------------------------------------
void ipapSet()
{
  lcd.setCursor(1, 0);
  lcd.print("BiPAP MODE: ");
  lcd.setCursor(1, 1);
  lcd.print("IPAP: ");
  lcd.setCursor(7, 1);
  lcd.print(ipap);
  lcd.setCursor(1, 2);
  lcd.print("EPAP: ");
  lcd.setCursor(7, 2);
  lcd.print(epap);
  lcd.setCursor(1, 3);
  lcd.print("BACK");
  lcd.setCursor(0, cursorPos);
  lcd.print(" ");

  lcd.setCursor(10, 1);
  lcd.write(byte(0));

  if (ipap <= 8)
  {
    lcd.setCursor(10, 1);
    lcd.write(byte(1));
  }
  else if (ipap >= 20)
  {
    lcd.setCursor(10, 1);
    lcd.write(byte(2));
  }

  if (bipapState == 1)
  {
    lcd.setCursor(13, 0);
    lcd.print("ON ");
  }
  else
  {
    lcd.setCursor(13, 0);
    lcd.print("OFF");
  }

  if (increase == 1)
  {
    increase = 0;
    ipap++;
  }

  if (decrease == 1)
  {
    decrease = 0;
    ipap--;
  }

  if (ipap >= 20)
  {
    ipap = 20;
  }

  if (ipap <= 8)
  {
    ipap = 8;
  }

  if (ipap < 10)
  {
    lcd.setCursor(8, 1);
    lcd.print(" ");
  }

  if (power == 1)
  {
    power = 0;
    mode = 2;
    lcd.clear();
  }
}

// ------------------------------------------ MODE 6 -----------------------------------------------------------------
void epapSet()
{
  lcd.setCursor(1, 0);
  lcd.print("BiPAP MODE: ");
  lcd.setCursor(1, 1);
  lcd.print("IPAP: ");
  lcd.setCursor(7, 1);
  lcd.print(ipap);
  lcd.setCursor(1, 2);
  lcd.print("EPAP: ");
  lcd.setCursor(7, 2);
  lcd.print(epap);
  lcd.setCursor(1, 3);
  lcd.print("BACK");
  lcd.setCursor(0, cursorPos);
  lcd.print(" ");
  lcd.setCursor(10, 2);
  lcd.write(byte(0));

  if (epap <= 4)
  {
    lcd.setCursor(10, 2);
    lcd.write(byte(1));
  }
  else if (epap >= 10)
  {
    lcd.setCursor(10, 2);
    lcd.write(byte(2));
  }

  if (bipapState == 1)
  {
    lcd.setCursor(13, 0);
    lcd.print("ON ");
  }
  else
  {
    lcd.setCursor(13, 0);
    lcd.print("OFF");
  }

  if (increase == 1)
  {
    increase = 0;
    epap++;
  }

  if (decrease == 1)
  {
    decrease = 0;
    epap--;
  }

  if (epap >= 10)
  {
    epap = 10;
  }

  if (epap <= 4)
  {
    epap = 4;
  }

  if (epap < 10)
  {
    lcd.setCursor(8, 2);
    lcd.print(" ");
  }

  if (epap >= ipap)
  {
    epap = ipap;
  }

  if (power == 1)
  {
    power = 0;
    mode = 2;
    lcd.clear();
  }
}

//------------------------------------------- I:E SETTING -----------------------------------------------------------------
void IEsetting()
{
  cpapState = 1;
  if (millis() - iTime < 2000)
  {
    displayPres = 10;
  }
  if (millis() - iTime > 2000)
  {
    displayPres = 0;
  }
  if (millis() - iTime > 6000)
  {
    iTime = millis();
  }

}

// ------------------------------------------ HIGH PASS --------------------------------------------------------------
double mifiltroHP(double dato, double dt, double RC)
{
  double alpha = RC / (RC + dt);
  data_hp[1] = dato;

  // High Pass Filter
  data_filt_hp[1] = alpha * (data_filt_hp[0] + data_hp[1] - data_hp[0]);

  // Store the previous data in correct index
  data_hp[0] = data_hp[1];
  data_filt_hp[0] = data_filt_hp[1];

  return (data_filt_hp[1]);
}

// ------------------------------------------ LOW PASS ---------------------------------------------------------------
double mifiltroLP(double dato, double dt, double RC)
{
  double alpha = dt / (RC + dt);
  data_lp[1] = dato;

  // low Pass Filter
  data_filt_lp[1] = alpha * data_lp[1] + (data_filt_lp[0] * (1 - alpha));

  // Store the previous data in correct index
  data_lp[0] = data_lp[1];
  data_filt_lp[0] = data_filt_lp[1];

  return (data_filt_lp[1]);
}

// END