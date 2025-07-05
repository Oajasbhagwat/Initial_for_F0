extern unsigned long timr3, timr4;  // declared in LCD_Touch.c++
#include "DEV_Config.h"
#include "LCD_Driver.h"
#include "LCD_GUI.h"
#include "LCD_Touch.h"
#include <SPI.h>
#include <SD.h>
const int chipSelect = 53;
     File dataFile;

void setup() {
  const int Tch_Screen_SD_Cs = 53;  //  we have used SDC_CS_PIN=53 (line no.71) should it be 5, (i.e. D5)(for Touch Screen LCD)
  digitalWrite(42, HIGH);           // this will ensure that flip-flop u1-A(4013) remains on & +7.5 V remains on. if we wish to turn off DSU, we should make D42=0
  Serial1.begin(9600, SERIAL_8N1);  // for GPS module ,9/August/2022
  Serial2.begin(4800, SERIAL_8N2);  // Now,(5/Sept/2022)receives Resistance,Batt etc.on DSU2_Rx2 But transmits any data to SimCrm1_Tx2(Earlier,for receiving data from SimCrm1 9/August/2022
  System_Init();
  pinMode(SDC_CS_PIN, OUTPUT);  // this is pin 53
  pinMode(22, INPUT_PULLUP);
  digitalWrite(22, HIGH);
  pinMode(23, INPUT_PULLUP);
  digitalWrite(23, HIGH);  //D22,D23 & D26 are 'input' pins is permanent.
  pinMode(26, INPUT_PULLUP);
  pinMode(40, INPUT_PULLUP); pinMode(28, INPUT_PULLUP);pinMode(29, INPUT_PULLUP);  //
  pinMode(24, OUTPUT);
  digitalWrite(24, HIGH);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);  // D24, D25 are ouputs,initially '0' become '1'at U7,8255 pins
  pinMode(42, OUTPUT);
  digitalWrite(42, HIGH);  // to keep power to Arduino on
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);  // D27 is made '0' for ~200 mSec. to turn A1 card on
  void Recv_Serial2(void);  // prototype definition
 Serial.begin(57600, SERIAL_8N1);  //for Serial monitor (LapTop)
  Serial.println("examining DSU data");
  LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;  //SCAN_DIR_DFT = D2U_L2R
  LCD_Init(Lcd_ScanDir, 200);
  TP_Dialog();
  del1();
  InitTimr();
  interrupts();       // This is moved to A4_Init enable all global interrup
  lcd1.begin(20, 4);  //  20, 4): 20 chars.(0~19)x 4 lines(0~3)
  lcd1.setCursor(0, 0);
  lcd1.print("Anvic systems ");
  Serial.println("SR11----");
  lcd1.setCursor(0, 1);
  lcd1.print("+++++ CRM Auto - D +++Test 68+++");
  delay(2000);
  lcd1.setCursor(0, 1);  //
 Serial1.begin(9600);
Serial.println("SR12----");
  const int SDC_CS_PIN = 53;
  Serial.println("SR13---");
       SD_ok=1;lcd1.clear();lcd1.setCursor(0, 2);
        lcd1.setCursor(5, 3);  lcd1.print(" ...SD OK.... ");
        delay(2000);
  DDRC = 0xFF;  // port C is output port. Not needed !
  pinMode(26, INPUT_PULLUP);  // this pin goes Low when Measure switch is pressed
  A4_Init();  // ( one-time initialization
  pinMode(otpin0, OUTPUT);
  pinMode(otpin1, OUTPUT);  // out (3,2,1,0) 1110
  pinMode(otpin2, OUTPUT);
  pinMode(otpin3, OUTPUT);
  pinMode(Kbin0, INPUT_PULLUP);
  pinMode(Kbin1, INPUT_PULLUP);
  pinMode(Kbin2, INPUT_PULLUP);
  pinMode(Kbin3, INPUT_PULLUP);
  //SD Card from below
   Serial.begin(9600);
  pinMode(53, OUTPUT);
  SD.begin();
   while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  // pinMode(10, OUTPUT);
  // digitalWrite(10, HIGH);
  if (!SD.begin()) {
    Serial.println("Initialization failed!");
    lcd1.setCursor(0, 1);
    lcd1.print("Initialization failed 68");
    delay(2000);
    return;
  }
  Serial.println("Initialization sucessful!");
  dataFile = SD.open("data68.txt", FILE_WRITE);
   lcd1.setCursor(0, 1);
    lcd1.print("Initialization sucessful 68");
    delay(2000);

   dataFile.print("test 68");
  dataFile.close();
  Serial.println("test data saved 68");
    lcd1.print("data saving sucessful 68");
    delay(2000);

     Serial.begin(57600, SERIAL_8N1);  //for Serial monitor (LapTop)
    pinMode(53, HIGH);
}  // end of 'setup()
void loop() {
   i11++;
  if (i11 >= 100000) {
    i11 = 0;
    i13++;
    i14 = i13 % 10;
  }
  if (timr4 > 200 && timr4 <= 400) Get_GPS();  // timr4 goes from 0~ 40 Sec. {  Kbkaz=0; }    //  Read & show GPS timr4<=5 Sec. only
  Recv_Serial2();     //    //  void receive data from A1-card
  Kb_Action();  // show all 16 keys, & take appropriatae actions
  Led += 1;
}  // end of 'loop'
void InitTimr(void) {}
