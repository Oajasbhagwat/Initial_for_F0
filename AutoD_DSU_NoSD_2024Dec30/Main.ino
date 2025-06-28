extern unsigned long timr3, timr4;  // declared in LCD_Touch.c++
                                    // ------------------ --------------initialization  14/Feb/2022  -----------------------------------------------
                                    //  --- all initialization done here
                                    // --------------------------------------------------------------------------------------------------------
#include "DEV_Config.h"
#include "LCD_Driver.h"
#include "LCD_GUI.h"
#include "LCD_Touch.h"
void setup() {
  const int Tch_Screen_SD_Cs = 53;  //  we have used SDC_CS_PIN=53 (line no.71) should it be 5, (i.e. D5)(for Touch Screen LCD)
  digitalWrite(42, HIGH);           // this will ensure that flip-flop u1-A(4013) remains on & +7.5 V remains on. if we wish to turn off DSU, we should make D42=0
  Serial1.begin(9600, SERIAL_8N1);  // for GPS module ,9/August/2022
  //-------------------------------------------------------
  // GPSSer.begin(9600) ; // actually this is same as Serial1.begin(9600) (GPS channel)
  Serial2.begin(4800, SERIAL_8N2);  // Now,(5/Sept/2022)receives Resistance,Batt etc.on DSU2_Rx2 But transmits any data to SimCrm1_Tx2(Earlier,for receiving data from SimCrm1 9/August/2022
  System_Init();
  pinMode(SDC_CS_PIN, OUTPUT);  // this is pin 53
  pinMode(22, INPUT_PULLUP);
  digitalWrite(22, HIGH);
  pinMode(23, INPUT_PULLUP);
  digitalWrite(23, HIGH);  //D22,D23 & D26 are 'input' pins is permanent.
  pinMode(26, INPUT_PULLUP);
  pinMode(40, INPUT_PULLUP); pinMode(28, INPUT_PULLUP);pinMode(29, INPUT_PULLUP);  //
      //D28 is Err7 (Amplifier saturated)(when 0) D29 
  // D22-Data,D23-A1Req,(D26=0 can be used to detect that ' A1 card is being turned on,D40=1 means A1 is turned on are defined as input pins
  pinMode(24, OUTPUT);
  digitalWrite(24, HIGH);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);  // D24, D25 are ouputs,initially '0' become '1'at U7,8255 pins
                           //----- added 3/dec/2021 ----------------------------------------
  pinMode(42, OUTPUT);
  digitalWrite(42, HIGH);  // to keep power to Arduino on
                           //..............................................................
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);  // D27 is made '0' for ~200 mSec. to turn A1 card on
        /*
  pinMode(Kbin0, INPUT_PULLUP);    //repeated at Line 144
  pinMode(Kbin1, INPUT_PULLUP);
  pinMode(Kbin2, INPUT_PULLUP);
  pinMode(Kbin3, INPUT_PULLUP);
         */
  void Recv_Serial2(void);  // prototype definition

  //pins 22,23 are now input pins. other PortA pins PrA[2~7) (D24~D27) (6 pins) remain as output pins
  //  D21 is (ISR pin)

  Serial.begin(57600, SERIAL_8N1);  //for Serial monitor (LapTop)

  Serial.println("examining DSU data");
  //Serial.println("Init...");

  LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;  //SCAN_DIR_DFT = D2U_L2R
  LCD_Init(Lcd_ScanDir, 200);

  //Serial.println("Init Touch Pad...");   // following functions are declared in lcd_touch.h
  //TP_Init( Lcd_ScanDir );
  //TP_GetAdFac();
  TP_Dialog();
  GUI_DisString_EN(4, 5, "...Auto_DSU8..~5Jan'23  ", &Font12, Colr[4], BLUE);         // show Title of the project
  GUI_DisNum(90, 50, i13 + 1, &Font12, Colr[6], BLUE);                                //
  GUI_DrawRectangle(3, 4, 3 + (8 * 36), 4 + 14, Colr[3], DRAW_EMPTY, DOT_PIXEL_DFT);  // draw Rect. around above letters
                                                                                      // ------Defined already earlier (Serial2.begin(9600);  // baud rate for GPS is 9600 )
  // background colour '5' is 'Tan'
  //void A4_ckt1();  // draw a line
  del1();
  InitTimr();
  //byte RevBits (byte Num);
  // ......defined in LCD_Touch.h ------void wrt_8255(  byte x,  byte Ad);
  interrupts();       // This is moved to A4_Init enable all global interrup
  lcd1.begin(20, 4);  //  20, 4): 20 chars.(0~19)x 4 lines(0~3)
                      // Print a message to the LCD.

  // print the number of seconds since reset:
  lcd1.setCursor(0, 0);
  lcd1.print("Anvic systems ");
  Serial.println("SR11----");
  lcd1.setCursor(0, 1);
  lcd1.print("+++++ CRM Auto - D +++Test 2.24+++");
  lcd1.setCursor(0, 1);  //
 Serial1.begin(9600);
  //  vr2= 6.28;
  //Serial.println("Drawing..."); Serial.print (tb2); Serial.write("  "); Serial.print (vr2);
  //
  //A4_ckt1();
  // ---- commented on 9/August/2022,Tuesday,  ~ 11 am --------
  // attachInterrupt(digitalPinToInterrupt(21),ISR_clk_pin,FALLING); // clk pin (D21) 'clock' signal from transmitter 'SimCrm1'
  //------............................................------------
  // D27 of 'SimCrm1'is connected to pin 21 of 'DSU' ( 'Receiver')
  //  --------------------------------write a file on SD (16 GB )card --------
Serial.println("SR12----");


  //START: Initialise SD card       ****temp modification begin
  const int SDC_CS_PIN = 53;
     
     //SD_ok = 1;  // SD is supressed 
         /*
  while (!SD.begin(SDC_CS_PIN)) {
    Serial.println("waiting for SD.begin ??????");
    lcd1.setCursor(0, 2);
    lcd1.print("-- waiting for SD????????");
  }           // while(1);()  SDC_CS_PIN is D53) if SD_ok  remains 0,it means that SD card failed ]]]
              */
  
  Serial.println("SR13---");
      // SD_ok=1 means SD card is ok ]]]
     /*
  if (SD_ok == 1) {
    GUI_DisString_EN(25, 20, "..SD Ok..", &Font12, Colr[7], BLUE);
    lcd1.setCursor(0, 2);
    lcd1.print(" SD --Ok-- ");
  }  //  sky blue background. show message on 20x4 too
  else {
    GUI_DisString_EN(25, 20, "..SD failed..", &Font16, Colr[7], RED);
    lcd1.setCursor(0, 2);
    lcd1.print("xx SD  Failed xx");
  }  
         */
        //  /*
       SD_ok=1;lcd1.clear();lcd1.setCursor(0, 2);  lcd1.print("...SD forced  ");
        lcd1.setCursor(5, 3);  lcd1.print("  to be OK.... ");
          //  */ 
          /*
  if(!SD.begin(SDC_CS_PIN) ) 
      { Serial.println("..SD failed...");lcd1.clear();lcd1.setCursor(0, 2);  lcd1.print("xxxxx SD failed");}
  
  else   { Serial.println("SD   ------OK-----..."); SD_ok==1;
    lcd1.clear();lcd1.setCursor(0, 2);  lcd1.print("=======SD OK========"); }
     // --------- write a file on SD card --cancelled------------------------------------------------

            */

  //   ------------------------------- for testing crm--500 , A2 card ------
  //DDRA = 0xFF ;    // port A is output port
  // pinMode(28, INPUT_PULLUP) ; digitalWrite(28,1);// both pins D28,D29 PrA[6,7] defined as input pins with a Pullup resistor
  // pinMode (29, INPUT_PULLUP) ;  digitalWrite(29,1); // other PortA pins PrA[0~5) (D22~D27) (6 pins) remain as output pins
  // Err7= pin D28(port PrA[6] (0 means A2 card, Amplif. output saturated, Ierr= pin D29(port PrA[7](0 means Ie current(A4 card) not flowing
  DDRC = 0xFF;  // port C is output port. Not needed !

  //-----------------------------------------------------------------
        //Serial.write("  A4_Init  ");
        //Serial.write("    loop() begins ");
  //InitTimr();

  pinMode(26, INPUT_PULLUP);  // this pin goes Low when Measure switch is pressed
                              //  ------------------------------------------------------------------------------added 9/Jan/'22----------

  //  ................................................................................................
  A4_Init();  // ( one-time initialization
     //Serial.println("SR14+++");
  pinMode(otpin0, OUTPUT);
  pinMode(otpin1, OUTPUT);  // out (3,2,1,0) 1110
  pinMode(otpin2, OUTPUT);
  pinMode(otpin3, OUTPUT);

  pinMode(Kbin0, INPUT_PULLUP);
  pinMode(Kbin1, INPUT_PULLUP);
  pinMode(Kbin2, INPUT_PULLUP);
  pinMode(Kbin3, INPUT_PULLUP);
     // actually,The next 4 lin are not needed 
  
}  // end of 'setup()

// -----------------------------------------------Loop ------------------------
//  loop() begins (Do over & over again)
// ----------------------------------------------------------------------------
void loop() {
  //-----------------------presently: Serial1-GPS, Serial2-- main Auto-D instrument, Serial3- should be used for Blue-Tooth
  //  //-----------following 5-line
  //int A =1; if(A==1){digitalWrite(42,HIGH);A=A+1;}

  //  ------------- for Blue tooth communication--------------------------
  //Rx = 'C';
  //
  /*
         if(Serial1.available()>0){Rx = Serial1.read(); Serial.println(Rx);}
        if (  Rx == 'A'){digitalWrite(27, LOW);}
      if (  Rx== 'B'){ digitalWrite(27, HIGH);}
      if(Rx != 'A'){
     //}  ){}  
           */
  //,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,end of Blue tooth,,,,,,,,,,,,,,
  //volatile unsigned int k1,k2,i11,m3=0;

  extern volatile unsigned char Kbkaz;  // defined in 'Tab' -- LCD_Touch.c++
                                        //Serial.println("Drawing.... 2 ..");
                                        //----------------------------------------------Tp_DrawBoard,which detects if tp is pressed anywhere being removed 5/Jan/2023-------------------------------------
                                        // TP_DrawBoard();     // Check if Touch Screen pressed anywhere ? GUI_Show(); ,
                                        //.............................................................................................................................
                                        // -----Temporarily Get_GPS is blanked out  7/may/2022,Sat, ~4;00 pm------
                                        //void Get_GPS();    // now get_GPS is in tshow GPS--lattitude & Longitude
                                        //-------...........................-----------------------
                                        // Show_Timr5();   // this function call is only for very slow ext_clk
                                        //if(Rx == 'B' ||Rx == 'C')
                                        // A1_Connect(); // now 'A1_Connect()' calls 'Updt_RecD()', when A1 card is powered up
  //Serial.println("SR15---");
   i11++;
  if (i11 >= 100000) {
    i11 = 0;
    i13++;

    GUI_DrawRectangle(2, 80, 2 + 12 * 4, 80 + 12, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect
    GUI_DisNum(2, 80, i13, &Font12, Colr[7], BLUE);                                    //  show i13 each time i11 reaches 1000
                                                                                       //Serial.println(i13);     //show i13 on 'Screen lcd' as well as Serial monitor

    i14 = i13 % 10;
    GUI_DrawRectangle(2, 95, 2 + 12 * 4, 95 + 12, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect
    GUI_DisNum(2, 95, i14, &Font12, Colr[7], BLUE);                                    //  show i14 each time i11 reaches 1000
    //Serial.println(i14);  //
    // i14old updated
  }
  //----------------------temporarily, 'call get_GPS()' only when key 3 is pressed
  //------ temporarily stop GPS-----17/oct/2022----------
  // (not needed) if ( i14==1) Get_GPS();    i14old = i14; //  (i14old==0 &&) Read & show GPS each time i14 changes from 0--> to 1 & then update i14old
  //------------------------ now (10/sept/2022)cancelled -------
  // Updt_RecD();  // display message received from A1 card,D23,D24 Hand-shake signals & D22(data) & D21(clock) is in this 'Tab' itself
  // -------------....................---------------------------
  //-----------------------Get_GPS made unconditional, for testing: 17/oct/2022-------------------------------now Get_GPS call inserted-------22/August/20022, `1:00 pm------------------
  //----------if we want to read 'GPS' then make GPS_req = 'Tuue. then make timr4=0. After ~4 sec. make 'GPS'_req = 'False'-------
  if (timr4 > 200 && timr4 <= 400) Get_GPS();  // timr4 goes from 0~ 40 Sec. {  Kbkaz=0; }    //  Read & show GPS timr4<=5 Sec. only
                                               //......................................................................................
 // Serial.println("SR16---");

  Recv_Serial2();     //    //  void receive data from A1-card
  // .......................5:45 pm --again uncommented-------------
  //check_Auto_D() ; // now (7/Sept/2022) replaced by 'Recv_Serial2'
  //A1_Power() ;    //show A1card powered on status
//Serial.println("SR17---");

  Kb_Action();  // show all 16 keys, & take appropriatae actions
//Serial.println("SR18---");

  // -------------------------------------------
  //void Updt_DigInpLvls(); // show levels (0/1) of D22,D23, D21

  //............................................
  //check_Keyboard(); // shoe keycode if a key is pressed
  //---------------- --------  'update every second' disabled temporarily  ---29/may/2019--------
  //Updt_Displ();     // Frq+, count is written (if it has changed from last value. Similarly for Frq-
  //-------------------------------------------------------------------------------------------------
  //if (A1PowN==0)   Show_Kbpins();  // Show_Kbpins should be allowed only when A1 card is not powered on
  //   --------------  for testing Port A used for testing crm-500 A2 card --------------
  //void Check_Data(void);  // on rising edge clk signal on D23,place 0/1 at j2 th posn in 'Recvd Byte'
  //Test_Port();
  Led += 1;


  //delay(50);
}  // end of 'loop'
   //.......................................................................end of loop................
// ---------------------------------------------------------------------
// get GPS data
//  - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - -
     /*
      void Get_GPS()
 {  int cr;  volatile float lat,lon;// a carry flag for adding 5 hrs 30 min. to Greenwitch time
    float   ltg2, lsec2,lx2,frdg2,frmin2; // Of these only lat,lon have been used presently
   float  frdg0,frdg1, lx0,lx1, frmin0,frmin1, lsec0,lsec1, ltg0,ltg1;
   int ldg2,lmin2, ldg0,ldg1, lmin0,lmin1; // Of these, ldg2,lmin2 are not used presently
   int LnNo=0,tLn=0, Yr; byte Mn,Dte, Hr,mint,Scnd, DeciSec,  Cr ; 
   unsigned long f_age=0;
           /*
    while (Serial2.available() )
     {                            // {1
      if (gps.encode (Serial2.read () ) )
      {                                   //{2
      gps.f_get_position (&lat, &lon);                        // 
       Serial.print(lat);Serial.println(lon);  // print on Serial Monitor
       dtostrf(lat, 10,6,st2) ; dtostrf(lon, 10,6,st4) ;  //  convert float to strings
       GUI_DrawRectangle(50, 60, 50+140, 60+40, WHITE, DRAW_FULL, DOT_PIXEL_DFT);// Erase rect.
       GUI_DisString_EN(50,60, &st2[0], &Font16, WHITE,BLUE);  GUI_DisString_EN(50,80, &st4[0], &Font16, WHITE,BLUE);   // print lat (st2) & lon(st4)
     // date  and time
     gps.crack_datetime( &Yr,&Mn, &Dte, &Hr,&mint,&Scnd, &DeciSec, &f_age);  // get date & time
     mint+=30;  if ( mint>=60) {mint-=60; Cr=1; } else Cr=0; Hr+= 5+Cr;
    Serial.print(Dte);Serial.print(" "); Serial.print(Mn);Serial.print(" "); Serial.print(Yr);Serial.print("--"); // date
    Serial.print(Hr);Serial.print(":"); Serial.print(mint);Serial.print(":"); Serial.print(Scnd);Serial.println(" "); // time & NL
    LnNo++; tLn++; if (tLn>=5) {tLn=0; Serial.println(" ");Serial.print(LnNo);Serial.print(" ");}
    
    }   // }2                       // }1
    
  }  // }1
 } // }end of function Get_GPS 
            */
//...................................end of GPS data...........................
      //  */
/******************************************************* 
     initialization of Timer
 *******************************************************/
void InitTimr(void) {
  //  ----------------Timer-5 ( for Frq+ )-------------
  TCNT5 = 0;  // Timer count (16-bit) <-- 0
  TCCR5A = 0;
  TCCR5B = 0;                                         // Initializz to 0
  TCCR5B |= (1 << CS52) | (1 << CS51) | (1 << CS50);  // bits[2,1,0]=111 Select external clock (rising edge  )
                                                      // no interrpts enabled for timer 5
  TCCR5B |= (1 << WGM52);                             // bits [3,2,1,0] = 0100 means 'select CTC mode ( 16-bit Timer/counter ). thus timer value will be 0 ~ FFFFh
  OCR5A = 62500;                                      //(         presently 'Compare match A interrupt is not being used)
   TIMSK5 = 02;
                                                      //    ----- the counting is enabled now -----
  //   --------------Timer-0 (for Frq- )-------------------
  TCNT0 = 0;  // Timer count (8-bit) <-- 0
  TCCR0A = 0;
  TCCR0B = 0;                                         // Initializz to 0
  TCCR0A |= (1 << WGM01);                             // turn on CTC (Clear timer0 on conpare natch) mode
  TCCR0B |= (1 << CS02) | (1 << CS01) | (1 << CS00);  // bits[2,1,0]=111 Select external clock (rising edge  )
                                                      // no interrpts enabled for timer 0
                                                      //    ----- the counting is enabled now -----
  OCR0A = 250;                                        // 'Compare Regiser ='250' (0~249) ( it was 20 earlier )
  TIFR0 = 0;                                          // initialze
  TIFR0 |= (1 << OCF0A);                              // clear any pending interrupts
                                                      //  ----- interrupt
  TIMSK0 = 02;                                        // bit[1] = 1 means, 'timer- compare match A' interrupt is enabled
                                                      // If bit[0]=1, it means ' enable interrupt on 'overflow' (when TCNT0 goes from FF to 00)
  //   ------------- Tiner-3 ( for 10 milli-Second interrupt ) --------
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  TCCR3B |= (1 << WGM32) | (1 << CS32);  //|(1<<CS30);   // WGM3{3~0]= 0100b (=4) means CTC(Clear timer on Compare) mode,
                                         // (Clock Select)CS[2~0]=100 means Prescalar=256.  Internal ck3 = 16 Mhz/ 256 i.e. 62.5 kHz
                                         //   --- Temporary:-- ( interrupt disabled --8/nov/2021-----
  TIMSK3 |= (1 << OCIE3A);               // Compare Match A interrupt is enabled (it is understood that TIMSK3 is originally 0)
  // .........................................................
  OCR3A = 625;  // This will generate an interrupt every 10 mSec ( for 62,500 pulses per Sec.
  TCNT3 = 0;
  TIFR3 |= (1 << OCF3A);  // clear any pending interrupts
  TIMSK3 = 02;  // Compare Match A interrupt is enabled 
}
//  .....................................................end of initialization of Timers
