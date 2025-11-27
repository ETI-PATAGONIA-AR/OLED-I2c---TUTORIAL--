//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//             Frecuencimetro-Inductometro-Voltimetro-Osciloscopio                      //
//             prof.martintorres@educ.ar - ETI Patagonia Argentina                      //
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/pgmspace.h>               // PROGMEM
#include <EEPROM.h>
#define SCREEN_WIDTH 128                // ancho display OLED
#define SCREEN_HEIGHT 64                // alto display OLED
#define REC_LENGTH 200                  //

// Declaramos la coneccion I2C de un display OLED SSD1306 (SDA, SCL pines)
#define OLED_RESET     -1      // Reset pin #
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//#include <FreqCount.h>
////////////////////VARIABLES DEL FRECUENCIMETRO////////////////////
unsigned long count = 0;

////////////////////VARIABLES DEL INDUCTOMETRO////////////////////
//13 is the input to the circuit (connects to 150ohm resistor), 11 is the comparator/op-amp output.
double pulse, frequency, capacitance, inductance, inductance_mH;

////////////////////VARIABLES DEL VOLTIMETRO////////////////////
int PinADC = A1;
float voltaje=0.0;
int medida = 0;
int porcent = 0;
int pinOFFset=A2;
float offset=0.0;
int valorPot=0;
float vcc = 4.98;  //valor real de la alimentacion de Arduino, Vcc
float r1 = 1000000.0; // 1M
float r2 = 100000.0; // 100K
float input_volt = 0.0;
float temp = 0.0;

////////////////////VARIABLES DEL OSCILOSCOPIO////////////////////
const char vRangeName[10][5] PROGMEM = {"A50V", "A 5V", " 50V", " 20V", " 10V", "  5V", "  2V", "  1V", "0.5V", "0.2V"}; // \0
const char * const vstring_table[] PROGMEM = {vRangeName[0], vRangeName[1], vRangeName[2], vRangeName[3], vRangeName[4], vRangeName[5], vRangeName[6], vRangeName[7], vRangeName[8], vRangeName[9]};
const char hRangeName[8][6] PROGMEM = {" 50ms", " 20ms", " 10ms", "  5ms", "  2ms", "  1ms", "500us", "200us"};          // (48
const char * const hstring_table[] PROGMEM = {hRangeName[0], hRangeName[1], hRangeName[2], hRangeName[3], hRangeName[4], hRangeName[5], hRangeName[6], hRangeName[7]};

int waveBuff[REC_LENGTH];      //  (RAM)
char chrBuff[10];              //
String hScale = "xxxAs";
String vScale = "xxxx";

float lsb5V = 0.0055549;       // 5V0.005371 V/1LSB
float lsb50V = 0.051513;       // 50V 0.05371

volatile int vRange;           //   0:A50V, 1:A 5V, 2:50V, 3:20V, 4:10V, 5:5V, 6:2V, 7:1V, 8:0.5V
volatile int hRange;           // 0:50m, 1:20m, 2:10m, 3:5m, 4;2m, 5:1m, 6:500u, 7;200u
volatile int trigD;            // 0:1:
volatile int scopeP;           //  0:, 1:, 2:
volatile boolean hold = false; //
volatile boolean paraChanged = false; //  true
volatile int saveTimer;        // EEPROM
int timeExec;                  // (ms)

int dataMin;                   // (min:0)
int dataMax;                   // (max:1023)
int dataAve;                   // 10 max:10230)
int rangeMax;                  //
int rangeMin;                  //
int rangeMaxDisp;              // max100
int rangeMinDisp;              // min
int trigP;                     //
boolean trigSync;              //
int att10x;                    // 1

int SelecREFERENCIA = 0;

void setup()
{
// analogReference(INTERNAL);                // ADC1.1Vvref)
//Serial.begin(9600);
//FreqCount.begin(1000);
  pinMode(7, INPUT);    // funcion OSCILOSCOPIO
  pinMode(6, INPUT);    // funcion VOLTIMETRO
  pinMode(4, INPUT);    // funcion INDUCTOMETRO
  pinMode(2, INPUT_PULLUP);    // (int0
  pinMode(8, INPUT_PULLUP);    // Select
  pinMode(9, INPUT_PULLUP);    // Up
  pinMode(10, INPUT_PULLUP);   // Down
  pinMode(11, INPUT_PULLUP);   // Hold
  pinMode(12, INPUT);          // 1/10
  pinMode(13, INPUT);    // funcion FRECUENCIMETRO
  pinMode (PinADC, INPUT); //A1
  pinMode (pinOFFset, INPUT); //A1
 
  pinMode(5, OUTPUT);//output through a 150 ohm resistor to thr LC circuit
  pinMode(3, INPUT);//Input from the comparator output//Use any other pin you select
   
   //     Serial.begin(115200);        // RAM
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address I2C 0x3C for 128x64
    //       Serial.println(F("SSD1306 failed"));
    for (;;);                               //
  }
  loadEEPROM();                             // EEPROM
 
  attachInterrupt(0, pin2IRQ, FALLING);     //
  startScreen();
 }
 
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
// if (digitalRead(13) == LOW)
//   {
//   analogReference(DEFAULT);
//   FreqCount.begin(1000);
//   frecuencimetro();
//  }
 
   if (digitalRead(7) == LOW)
  {
   analogReference(INTERNAL);                // ADC1.1Vvref)
   //FreqCount.end();
   osciloscopio();
  }
 
   if (digitalRead(6) == LOW)
  {
   analogReference(DEFAULT);
   //FreqCount.end();
   fuente();
  }
 
   if (digitalRead(4) == LOW)
  {
   analogReference(DEFAULT);
   //FreqCount.end();
   inductometro();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//void frecuencimetro()
//{
// if (FreqCount.available())
// {
//  count = FreqCount.read();
//  display.clearDisplay();
//  display.setTextSize(1);            // 2
//  display.setTextColor(WHITE);       //
//  display.setCursor(5, 4);         //(10, 25);
//  display.println(F("Frecuencimetro"));   //
//  display.drawFastHLine(3, 2, 120, WHITE);
//  display.drawFastHLine(3, 12, 120, WHITE);
//  display.drawFastHLine(3, 62, 120, WHITE);
 
//  display.drawFastVLine(3, 3, 60, WHITE);
//  display.drawFastVLine(123, 3, 60, WHITE);
//   display.setTextSize(1);            // display.setTextSize(2);
//  display.setTextColor(WHITE);       //
//  display.setCursor(5, 20);         //
//  display.println(F("Frecuencia:"));
//  display.setCursor(15, 39);         //
//  display.println(count);
//  display.setCursor(52, 39);         //
//  display.println(F(" Hz"));
//  display.setCursor(52, 48);         //
//  display.display();                         //
//  delay(1000);
//  display.clearDisplay();
// }
//}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void inductometro()
{
  digitalWrite(5, HIGH);
  delay(5);//give some time to charge inductor.
  digitalWrite(5,LOW);
  delayMicroseconds(100); //make sure resination is measured
  pulse = pulseIn(3,HIGH,5000);//returns 0 if timeout
  if(pulse > 0.1) //if a timeout did not occur and it took a reading:
  {
 // #error insert your used capacitance value here. Currently using 2uF. Delete this line after that
  capacitance = 2.E-6; // <- insert value here
  frequency = 1.E6/(2*pulse);
  inductance = 1./(capacitance*frequency*frequency*4.*3.14159*3.14159);//one of my profs told me just do squares like this
  inductance *= 1E6; //note that this is the same as saying inductance = inductance*1E6
  inductance_mH = inductance * 1000; //note that this is the same as saying inductance = inductance*1E6

  display.clearDisplay();
  display.setTextSize(1);            // 2
  display.setTextColor(WHITE);       //
  display.setCursor(5, 4);         //(10, 25);
  display.println(F("INDUCTOMETRO"));   //
  display.drawFastHLine(3, 2, 120, WHITE);
  display.drawFastHLine(3, 12, 120, WHITE);
  display.drawFastHLine(3, 62, 120, WHITE);
  display.drawFastVLine(3, 3, 60, WHITE);
  display.drawFastVLine(123, 3, 60, WHITE);
  display.setCursor(5, 16);         //
  display.println(F("High for uS:"));
  display.setCursor(80, 16);
  display.println(pulse);
  display.setCursor(5, 25);         //
  display.println(F("\tFrec Hz:"));
  display.setCursor(65, 25);         //
  display.println( frequency );
  display.setCursor(5, 34);         //
  display.println(F("\tInd uH:"));
  display.setCursor(55, 34);  
  display.println( inductance );
    display.setTextSize(1);            // 2
  display.setTextColor(WHITE);       //
  display.setCursor(5, 43);         //
  display.println(F("\tInd mH:"));
  display.setCursor(55,43);
  display.println(inductance_mH);
  display.display();                 //
  delay(900);
  display.clearDisplay();
  display.setTextSize(1);            //
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void fuente()
{

int analogvalue = analogRead (A1);
temp = (analogvalue * 5.0)/1024;
input_volt = temp/(r2/(r1+r2));
if (input_volt <0.1)
{
  input_volt =0.0;
}
  display.clearDisplay();
  display.setTextSize(1);            // 2
  display.setTextColor(WHITE);       //
  display.setCursor(5, 4);         //(10, 25);
  display.println(F("VOLTIMETRO"));   //
  display.drawFastHLine(3, 2, 120, WHITE);
  display.drawFastHLine(3, 12, 120, WHITE);
  display.drawFastHLine(3, 62, 120, WHITE);
 
  display.drawFastVLine(3, 3, 60, WHITE);
  display.drawFastVLine(123, 3, 60, WHITE);
   display.setTextSize(1);            // display.setTextSize(2);
  display.setTextColor(WHITE);       //
  display.setCursor(5, 20);         //
  display.println(F("Tension:"));
  display.setCursor(15, 39);         //
  display.println(input_volt);
  display.setCursor(52, 39);         //
  display.println(F(" V"));
  display.setCursor(52, 48);         //
  display.display();                         //
  delay(1000);
  display.clearDisplay();
  //display.setTextSize(1);            //
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void osciloscopio()
{
  digitalWrite(13, HIGH);
  setConditions();                          // RAM40
  readWave();                               //  (1.6ms )
  digitalWrite(13, LOW);                    //
  dataAnalize();                            // (0.4-0.7ms)
  writeCommonImage();                       // (4.6ms)
  plotData();                               // (5.4ms+)
  dispInf();                                // (6.2ms)
  display.display();                        // (37ms)
  saveEEPROM();                             // EEPROM
  while (hold == true) {                    // Hold
    dispHold();
    delay(10);
  }
}

void setConditions() {   //
  // PROGMEM
  strcpy_P(chrBuff, (char*)pgm_read_word(&(hstring_table[hRange])));  //
  hScale = chrBuff;                                                   // Escala Horizontal

  //
  strcpy_P(chrBuff, (char*)pgm_read_word(&(vstring_table[vRange])));  //
  vScale = chrBuff;                                                   // Escala Vertical

  switch (vRange) {              //
    case 0: {                    // Auto50V
        //        rangeMax = 1023;
        //        rangeMin = 0;
        att10x = 1;              //
        break;
      }
    case 1: {                    // Auto 5V
        //        rangeMax = 1023;
        //        rangeMin = 0;
        att10x = 0;              //
        break;
      }
    case 2: {                    // 50V
        rangeMax = 50 / lsb50V;  //
        rangeMaxDisp = 5000;     // 100
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 1;              //
        break;
      }
    case 3: {                    // 20V
        rangeMax = 20 / lsb50V;  //
        rangeMaxDisp = 2000;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 1;              //
        break;
      }
    case 4: {                    // 10V
        rangeMax = 10 / lsb50V;  //
        rangeMaxDisp = 1000;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 1;              //
        break;
      }
    case 5: {                    // 5V
        rangeMax = 5 / lsb5V;    //
        rangeMaxDisp = 500;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              //
        break;
      }
    case 6: {                    // 2V
        rangeMax = 2 / lsb5V;    //
        rangeMaxDisp = 200;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              //
        break;
      }
    case 7: {                    // 1V
        rangeMax = 1 / lsb5V;    //
        rangeMaxDisp = 100;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              //
        break;
      }
    case 8: {                    // 0.5V
        rangeMax = 0.5 / lsb5V;  //
        rangeMaxDisp = 50;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              //
        break;
      }
    case 9: {                    // 0.5V
        rangeMax = 0.2 / lsb5V;  //
        rangeMaxDisp = 20;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              //
        break;
      }
  }
}

void writeCommonImage() {     //
  display.clearDisplay();                   // (0.4ms)
  display.setTextColor(WHITE);              //
  display.setCursor(86, 0);                 // Iniciar en la parte superior izquierda
  display.println(F("av    V"));            // 1
  display.drawFastVLine(26, 9, 55, WHITE);  //
  display.drawFastVLine(127, 9, 55, WHITE); //

  display.drawFastHLine(24, 9, 7, WHITE);   // Max
  display.drawFastHLine(24, 36, 2, WHITE);  //
  display.drawFastHLine(24, 63, 7, WHITE);  //

  display.drawFastHLine(51, 9, 3, WHITE);   // Max
  display.drawFastHLine(51, 63, 3, WHITE);  //

  display.drawFastHLine(76, 9, 3, WHITE);   // Max
  display.drawFastHLine(76, 63, 3, WHITE);  //

  display.drawFastHLine(101, 9, 3, WHITE);  // Max
  display.drawFastHLine(101, 63, 3, WHITE); //

  display.drawFastHLine(123, 9, 5, WHITE);  // Max
  display.drawFastHLine(123, 63, 5, WHITE); //

  for (int x = 26; x <= 128; x += 5) {
    display.drawFastHLine(x, 36, 2, WHITE); // ()
  }
  for (int x = (127 - 25); x > 30; x -= 25) {
    for (int y = 10; y < 63; y += 5) {
      display.drawFastVLine(x, y, 2, WHITE); // 3
    }
  }
}

void readWave() {                            //
  if (att10x == 1) {                         // 1/10
    pinMode(12, OUTPUT);                     //
    digitalWrite(12, LOW);                   // LOW
  } else {                                   //
    pinMode(12, INPUT);                      // Hi-z
  }

  switch (hRange) {                          //

    case 0: {                                // 50ms
        timeExec = 400 + 50;                 // (ms) EEPROM
        ADCSRA = ADCSRA & 0xf8;              // 3
        ADCSRA = ADCSRA | 0x07;              // 128 (arduino
        for (int i = 0; i < REC_LENGTH; i++) {     // 200
          waveBuff[i] = analogRead(0);       // 112s
          delayMicroseconds(1888);           //
        }
        break;
      }

    case 1: {                                // 20ms
        timeExec = 160 + 50;                 // (ms) EEPROM
        ADCSRA = ADCSRA & 0xf8;              // 3
        ADCSRA = ADCSRA | 0x07;              // 128 (arduino
        for (int i = 0; i < REC_LENGTH; i++) {     // 200
          waveBuff[i] = analogRead(0);       // 112s
          delayMicroseconds(688);            //
        }
        break;
      }

    case 2: {                                // 10 ms
        timeExec = 80 + 50;                  // (ms) EEPROM
        ADCSRA = ADCSRA & 0xf8;              // 3
        ADCSRA = ADCSRA | 0x07;              // 128 (arduino
        for (int i = 0; i < REC_LENGTH; i++) {     // 200
          waveBuff[i] = analogRead(0);       // 112s
          delayMicroseconds(288);            //
        }
        break;
      }

    case 3: {                                // 5 ms
        timeExec = 40 + 50;                  // (ms) EEPROM
        ADCSRA = ADCSRA & 0xf8;              // 3
        ADCSRA = ADCSRA | 0x07;              // 128 (arduino
        for (int i = 0; i < REC_LENGTH; i++) {     // 200
          waveBuff[i] = analogRead(0);       // 112s
          delayMicroseconds(88);             //
        }
        break;
      }

    case 4: {                                // 2 ms
        timeExec = 16 + 50;                  // (ms) EEPROM
        ADCSRA = ADCSRA & 0xf8;              // 3
        ADCSRA = ADCSRA | 0x06;              // 64 (0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
        for (int i = 0; i < REC_LENGTH; i++) {     // 200
          waveBuff[i] = analogRead(0);       // 56s
          delayMicroseconds(24);             //
        }
        break;
      }

    case 5: {                                // 1 ms
        timeExec = 8 + 50;                   // (ms) EEPROM
        ADCSRA = ADCSRA & 0xf8;              // 3
        ADCSRA = ADCSRA | 0x05;              // 16 (0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
        for (int i = 0; i < REC_LENGTH; i++) {     // 200
          waveBuff[i] = analogRead(0);       // 28s
          delayMicroseconds(12);             //
        }
        break;
      }

    case 6: {                                // 500us
        timeExec = 4 + 50;                   // (ms) EEPROM
        ADCSRA = ADCSRA & 0xf8;              // 3
        ADCSRA = ADCSRA | 0x04;              // 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
        for (int i = 0; i < REC_LENGTH; i++) {     // 200
          waveBuff[i] = analogRead(0);       // 16s
          delayMicroseconds(4);              //
          // 1.875snop 110.0625s @16MHz)
          asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
          asm("nop"); asm("nop"); asm("nop");
        }
        break;
      }

    case 7: {                                // 200us
        timeExec = 2 + 50;                   // (ms) EEPROM
        ADCSRA = ADCSRA & 0xf8;              // 3
        ADCSRA = ADCSRA | 0x02;              // :4(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
        for (int i = 0; i < REC_LENGTH; i++) {
          waveBuff[i] = analogRead(0);       // 6s
          // 1.875snop 110.0625s @16MHz)
          asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
          asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
          asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        }
        break;
      }
  }
}

void dataAnalize() {                   //
  int d;
  long sum = 0;

  //
  dataMin = 1023;                         //
  dataMax = 0;                            //
  for (int i = 0; i < REC_LENGTH; i++) {  //
    d = waveBuff[i];
    sum = sum + d;
    if (d < dataMin) {                    //
      dataMin = d;
    }
    if (d > dataMax) {                    //
      dataMax = d;
    }
  }

  //
  dataAve = (sum + 10) / 20;               // 10

  // max,min
  if (vRange <= 1) {                       // Auto1
    rangeMin = dataMin - 20;               // -20
    rangeMin = (rangeMin / 10) * 10;       // 10
    if (rangeMin < 0) {
      rangeMin = 0;                        // 0
    }
    rangeMax = dataMax + 20;               // +20
    rangeMax = ((rangeMax / 10) + 1) * 10; // 10
    if (rangeMax > 1020) {
      rangeMax = 1023;                     // 10201023
    }

    if (att10x == 1) {                            //
      rangeMaxDisp = 100 * (rangeMax * lsb50V);   // ADC
      rangeMinDisp = 100 * (rangeMin * lsb50V);   //
    } else {                                      //
      rangeMaxDisp = 100 * (rangeMax * lsb5V);
      rangeMinDisp = 100 * (rangeMin * lsb5V);
    }
  } else {                                   //
    //
  }

  //
  for (trigP = ((REC_LENGTH / 2) - 51); trigP < ((REC_LENGTH / 2) + 50); trigP++) { //
    if (trigD == 0) {                        // 0
      if ((waveBuff[trigP - 1] < (dataMax + dataMin) / 2) && (waveBuff[trigP] >= (dataMax + dataMin) / 2)) {
        break;                              //
      }
    } else {                                // 0
      if ((waveBuff[trigP - 1] > (dataMax + dataMin) / 2) && (waveBuff[trigP] <= (dataMax + dataMin) / 2)) {
        break;
      }                                    //
    }
  }
  trigSync = true;
  if (trigP >= ((REC_LENGTH / 2) + 50)) {  //
    trigP = (REC_LENGTH / 2);
    trigSync = false;                      // Sin sincronismo (Unsync)
  }
}


void dispHold() {                            // Congelar pantalla (Hold)
  display.fillRect(32, 12, 24, 8, BLACK);    // 4
  display.setCursor(32, 12);
  display.print(F("Hold"));                  // (Hold)
  display.display();                         //
}

void dispInf() {                             //
  float voltage;
  //
  display.setCursor(2, 0);                   //
  display.print(vScale);                     //
  if (scopeP == 0) {                         //
    display.drawFastHLine(0, 7, 27, WHITE);  //
    display.drawFastVLine(0, 5, 2, WHITE);
    display.drawFastVLine(26, 5, 2, WHITE);
  }

  //
  display.setCursor(34, 0);                  //
  display.print(hScale);                     // Divisor escalar de tiempo(time/div)
  if (scopeP == 1) {                         //
    display.drawFastHLine(32, 7, 33, WHITE); //
    display.drawFastVLine(32, 5, 2, WHITE);
    display.drawFastVLine(64, 5, 2, WHITE);
  }

  //
  display.setCursor(75, 0);                  //
  if (trigD == 0) {
    display.print(char(0x18));               //
  } else {
    display.print(char(0x19));               //              
  }
  if (scopeP == 2) {      //
    display.drawFastHLine(71, 7, 13, WHITE); //
    display.drawFastVLine(71, 5, 2, WHITE);
    display.drawFastVLine(83, 5, 2, WHITE);
  }

  //
  if (att10x == 1) {                         // 10
    voltage = dataAve * lsb50V / 10.0;       // 50V
  } else {
    voltage = dataAve * lsb5V / 10.0;        // 5V
  }
  dtostrf(voltage, 4, 2, chrBuff);           // x.xx
  display.setCursor(98, 0);                  //
  display.print(chrBuff);                    //
  //  display.print(saveTimer);                  //

  //
  voltage = rangeMaxDisp / 100.0;            // Max
  if (vRange == 1 || vRange > 4) {           // 5VAuto5V
    dtostrf(voltage, 4, 2, chrBuff);         //  *.**
  } else {                                   //
    dtostrf(voltage, 4, 1, chrBuff);         // **.*
  }
  display.setCursor(0, 9);
  display.print(chrBuff);                    // Max

  voltage = (rangeMaxDisp + rangeMinDisp) / 200.0; //
  if (vRange == 1 || vRange > 4) {           // 5VAuto5V
    dtostrf(voltage, 4, 2, chrBuff);         // 2
  } else {                                   //
    dtostrf(voltage, 4, 1, chrBuff);         // 1
  }
  display.setCursor(0, 33);
  display.print(chrBuff);                    //

  voltage = rangeMinDisp / 100.0;            // Min
  if (vRange == 1 || vRange > 4) {           // 5VAuto5V
    dtostrf(voltage, 4, 2, chrBuff);         // 2
  } else {
    dtostrf(voltage, 4, 1, chrBuff);         // 1
  }
  display.setCursor(0, 57);
  display.print(chrBuff);                    // Min

  //
  if (trigSync == false) {                   //
    display.setCursor(60, 55);               //
    display.print(F("Unsync"));              // Unsync
  }
}

void plotData() {                    //
  long y1, y2;
  for (int x = 0; x <= 98; x++) {
    y1 = map(waveBuff[x + trigP - 50], rangeMin, rangeMax, 63, 9); //
    y1 = constrain(y1, 9, 63);                                     //
    y2 = map(waveBuff[x + trigP - 49], rangeMin, rangeMax, 63, 9); //
    y2 = constrain(y2, 9, 63);                                     //
    display.drawLine(x + 27, y1, x + 28, y2, WHITE);               //
  }
}

void saveEEPROM() {                    // EEPROM
  if (paraChanged == true) {           //
    saveTimer = saveTimer - timeExec;  //
    if (saveTimer < 0) {               //
      paraChanged = false;             //
      EEPROM.write(0, vRange);        //
      EEPROM.write(1, hRange);
      EEPROM.write(2, trigD);
      EEPROM.write(3, scopeP);
    }
  }
}

void loadEEPROM() {                // EEPROM
  int x;
  x = EEPROM.read(0);             // vRange
  if ((x < 0) || (x > 9)) {        // 0-9
    x = 3;                         //
  }
  vRange = x;

  x = EEPROM.read(1);             // hRange
  if ((x < 0) || (x > 7)) {        // 0-9
    x = 3;                         //
  }
  hRange = x;
  x = EEPROM.read(2);             // trigD
  if ((x < 0) || (x > 1)) {        // 0-9
    x = 1;                         //
  }
  trigD = x;
  x = EEPROM.read(3);             // scopeP
  if ((x < 0) || (x > 2)) {        // 0-9
    x = 1;                         //
  }
  scopeP = x;
}

void pin2IRQ() {                   // Pin2(int0)
  //pin8,9,10,11Pin2
  //

  int x;                           //
  x = PINB;                        // B

  if ( (x & 0x07) != 0x07) {       // 3High
    saveTimer = 5000;              // EEPROM(ms
    paraChanged = true;            // ON
  }

  if ((x & 0x01) == 0) {
    scopeP++;
    if (scopeP > 2) {
      scopeP = 0;
    }
  }

  if ((x & 0x02) == 0) {           // UP
    if (scopeP == 0) {             //
      vRange++;
      if (vRange > 9) {
        vRange = 9;
      }
    }
    if (scopeP == 1) {             //
      hRange++;
      if (hRange > 7) {
        hRange = 7;
      }
    }
    if (scopeP == 2) {             //
      trigD = 0;                   //
    }
  }

  if ((x & 0x04) == 0) {           // DOWN
    if (scopeP == 0) {             //
      vRange--;
      if (vRange < 0) {
        vRange = 0;
      }
    }
    if (scopeP == 1) {             //
      hRange--;
      if (hRange < 0) {
        hRange = 0;
      }
    }
    if (scopeP == 2) {             //
      trigD = 1;                   //
    }
  }

  if ((x & 0x08) == 0) {           // HOLD
    hold = ! hold;                 //
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void startScreen() {                 //
  display.clearDisplay();
  display.setTextSize(1);            // 2
  display.setTextColor(WHITE);       //
  display.setCursor(5, 4);         //(10, 25);
  display.println(F("Voltimetro-Ind-OSC"));   //
  display.drawFastHLine(3, 2, 120, WHITE);
  display.drawFastHLine(3, 12, 120, WHITE);
  display.drawFastHLine(3, 62, 120, WHITE);
 
  display.drawFastVLine(3, 3, 60, WHITE);
  display.drawFastVLine(123, 3, 60, WHITE);
  display.setCursor(5, 20);         //
  display.println(F("Desing by:"));
 
  display.setCursor(5, 29);         //
  display.println(F("Torres Alejandro M."));

  display.setTextSize(2);            // 2
  display.setTextColor(WHITE);       //
  display.setCursor(5, 46);         //
  display.println(F("  BETA V1  "));
  display.display();                 //
  delay(1500);
  display.clearDisplay();
  display.setTextSize(1);            //
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////