#define sensor1 1
#define sensor2 2
#define sensor3 3
#define sensor4 4
#define sensor5 5
#define sensor6 6
#define sensor7 7
#define sensor8 8

#define MotorKiri 12
#define MotorKanan 13

//Program 1, jika tombol di pencet maka ROBOT BERHENTI, kalau tidak di pencet maka ROBOT JALAN
#define tombol_program_1 20

int s1, s2, s3, s4, s5, s6, s7, s8;
int sensorgaris;
int errorsensorgaris;
int posisisensorgaris;


/*Variable PID*/
unsigned long lastTime;
double Input, PID;
double Setpoint = 0;
double errSum, lastErr;
// double kp = 0.3;        // yg kostum 0.3
double kp = 0.3;        // yg kostum 0.3
double ki = 0;
double kd = 25;
/*Variable PID*/


float PIDLeft, PIDRight;
float gas = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);
  pinMode(sensor7, INPUT);
  pinMode(sensor8, INPUT);
  pinMode(MotorKiri, OUTPUT);
  pinMode(MotorKanan, OUTPUT);

  pinMode(tombol_program_1, INPUT_PULLUP);
}

void loop() {
  BacaSensorGaris();
  hitungPID();
  eksekusikemotor();
  Program_1();
}

void KonversiDecimalkeBinner() {
  s1  = 1 * digitalRead(sensor1);
  s2  = 2 * digitalRead(sensor2);
  s3  = 4 * digitalRead(sensor3);
  s4  = 8 * digitalRead(sensor4);
  s5  = 16 * digitalRead(sensor5);
  s6  = 32 * digitalRead(sensor6);
  s7  = 64 * digitalRead(sensor7);
  s8  = 128 * digitalRead(sensor8);
  sensorgaris = s8 + s7 + s6 + s5 + s4 + s3 + s2 + s1;
}

void SettingNilaiError() {
  if     (sensorgaris == 0b00000001)   {errorsensorgaris =  7;    posisisensorgaris =  7;}
  else if(sensorgaris == 0b00000011)   {errorsensorgaris =  6;    posisisensorgaris =  6;}
  else if(sensorgaris == 0b00000111)   {errorsensorgaris =  5;    posisisensorgaris =  5;}
  else if(sensorgaris == 0b00000110)   {errorsensorgaris =  4;    posisisensorgaris =  4;}
  else if(sensorgaris == 0b00001110)   {errorsensorgaris =  3;    posisisensorgaris =  3;}
  else if(sensorgaris == 0b00001100)   {errorsensorgaris =  2;    posisisensorgaris =  2;}
  else if(sensorgaris == 0b00011100)   {errorsensorgaris =  1;    posisisensorgaris =  1;}
  else if(sensorgaris == 0b00011000)   {errorsensorgaris =  0;    posisisensorgaris =  0;}
  else if(sensorgaris == 0b00111000)   {errorsensorgaris = -1;    posisisensorgaris = -1;}
  else if(sensorgaris == 0b00110000)   {errorsensorgaris = -2;    posisisensorgaris = -2;}
  else if(sensorgaris == 0b01110000)   {errorsensorgaris = -3;    posisisensorgaris = -3;}
  else if(sensorgaris == 0b01100000)   {errorsensorgaris = -4;    posisisensorgaris = -4;}
  else if(sensorgaris == 0b11100000)   {errorsensorgaris = -5;    posisisensorgaris = -5;}
  else if(sensorgaris == 0b11000000)   {errorsensorgaris = -6;    posisisensorgaris = -6;}
  else if(sensorgaris == 0b10000000)   {errorsensorgaris = -7;    posisisensorgaris = -7;}
}

void BacaSensorGaris() {
  KonversiDecimalkeBinner();
  SettingNilaiError();
}


// --------------------------------- Fungsi PID ---------------------------------
/*Mengkalkulasi seberapa lama waktu yang dibutuhkan setelah eksekusi terakhir*/
void hitungPID() {
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);
    Input = errorsensorgaris;
    double Error = Setpoint - Input;
    errSum += (Error * timeChange);
    double dErr = (Error - lastErr) / timeChange;
    PID = kp * Error + ki * errSum + kd * dErr;        /*Hitung PID*/
    lastErr = Error;                                                          /*Simpan variable untuk ekesekusi selanjutnya*/
    lastTime = now;
}
// --------------------------------- Fungsi PID ---------------------------------



// --------------------------------- Eksekusi ke Motor ---------------------------------
void eksekusikemotor() {
    PIDLeft = gas - PID;
    PIDRight = gas + PID;
    if(PIDLeft > gas)           {PIDLeft = gas;}
    if(PIDLeft <= 0)            {PIDLeft = 0;}
    if(PIDRight > gas)          {PIDRight = gas;}
    if(PIDRight <= 0)           {PIDRight = 0;}
    analogWrite(MotorKiri, PIDRight);
    analogWrite(MotorKanan, PIDLeft);
}
// --------------------------------- Eksekusi ke Motor ---------------------------------

void Program_1() {
  if(digitalRead(tombol_program_1) == LOW) {
    PIDRight = 0;
    PIDLeft = 0;
    analogWrite(MotorKiri, PIDRight);
    analogWrite(MotorKanan, PIDLeft);
  }
}

