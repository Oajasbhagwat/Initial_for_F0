#include "LCD_Touch.h"  //---------AutoD_DSU8------------------
#include "Debug.h"
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <TinyGPS.h>
#include <TinyGPS++.h>     //#include <var_const.h>
#include <avr/pgmspace.h>  // <avr/pgmspace.h> is needed to define constants in 'Program memory' 8/march/2023
#include <arduino.h>
#include <stdlib.h>
#include <SD.h>
File myF;     //myF is a FILE object
TinyGPS gps;  // gps is a  TinyGPS object Baud rate for GPS chip is 9600
extern LCD_DIS sLCD_DIS;
static TP_DEV sTP_DEV;
static TP_DRAW sTP_Draw;
#define wsl 15                                                          // no.  of charrs received from wiighing machine
const int rs = 28, en = 30, d4 = 32, d5 = 34, d6 = 36, d7 = 38;         //
extern LiquidCrystal lcd1;                                              //(rs, en, d4, d5, d6, d7); //lcd1 is not being treated as Global object !
const unsigned int PLp[] PROGMEM = { 1, 2, 3, 5, 10, 13, 15, 20, 23 };  // not used, (numbers in PROG memory)
const unsigned int PLt[] PROGMEM = { 15, 20, 25, 30, 40, 50, 60, 80, 100, 100, 120, 150, 200, 250, 250, 300, 400, 500, 600, 800, 1000, 1000, 1200, 1500, 1800,
                                     2000, 2000, 2500, 3000, 3000, 3500, 4000, 4500, 5000, 5000, 5500, 6000, 6500 };  // 0~37 (total 38 integers),10* actual values
const unsigned int Plt[] PROGMEM = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 20, 20, 20, 20, 20, 50, 50, 50, 50, 50, 50, 50, 100, 100, 100, 100, 100, 200, 200, 200, 400, 400, 400, 400, 400, 500,
                                     500, 500, 500 };                                                                                                     // 0~37 (total 38 integers),10* actual values
const unsigned int Wennat[] PROGMEM = { 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300, 320, 340, 360, 380, 400, 440, 480, 520 };  // 0~22,23 values: 2~40 m. chang= 2m., 44~52. cgange by 4m. at each step
const unsigned int Dipat[] PROGMEM = { 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5 };                                       // 0~24, 25 values. like L
const unsigned int Dipnat[] PROGMEM = { 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6 };                                      // 0~24,25 values, like l
const char* const RangeSw[] PROGMEM = { "Bat", "mOhm", "ohm", "kOhm", "mV", "V" };                                                                        // Range switch
volatile unsigned int tEA ;
volatile byte nby8 = 0, nby7, old7, old8, IeNo;  // IeNo -- Sr. no. of current (0~5
volatile byte bk1 = 0x08, bk2 = 0x19, APrC, bk3, bk4, bk5, bk6, change = 0, change2 = 0, PrADt, PrCDt, Cap = 0;
volatile byte bk7[] = { 0X05, 0X0D, 0x19, 0x3F, 0x7D, 0xFA };  // bk7[0~5] are (bk[i]*0.08 --0.4,1.04,2,5.04,10,20 mAmp
volatile unsigned int drn = 0, n1 = 10, n2 = 15, n3, n4 = 5, n5, n6 = 10, n7 = 0, n8, n9 = 1, n10 = 2, n11 = 3, n12, n13 = 1, n14 = 0, n15, n16, n17, n18, n19 = 0, n20 = 0, n21, n22, n23, n24, n25, updt_flg = 0;
volatile unsigned int m1 = 0, m2 = 0, m3, m4, m5, m6 = 0, m7, m8, m9 = 1, m10 = 1, m11, m12 = 1, m13 = 0, m14, m15, k1, k2, k3, k4 = 0, k5, k6 = 0, im1, im2 = 20, im3, im4, im5;                              // some integer values
volatile unsigned int Surv_meth = 2, SpScr = 2, WaL;                                                                                                                                                           // =1 means -Schlumberger, ==2 means -Wenner, ==3 means -Dipole-Dipole ,SpScr- split screen
volatile POINT x1, y1, x2 = 20, x3 = 20, x4 = 20, x5 = 70, x6 = 80, x7, x8, x9, x10 = 100, y2 = 30, y3, y4, y5, y6 = 60, y7, y8, y10 = 120, xr1, yr1, quit, dx1, dx2, dx3, dx4, dx5, dy1, dy2, dy3, dy4, dy5;  // dy-some distance
volatile POINT xr2 = 5, xr3 = 45, xr4 = 20, xr5, yr2 = 50, yr3 = 200, yr4 = 170, yr5;
volatile POINT x12, x13, x14, x15, y12, y13, y14, y15, xi1[6] = { 300, 300, 300, 300, 300, 280 }, yi1[6] = { 50, 70, 90, 110, 130, 160 }, xw1 = 10, yw1 = 190, xv1 = 60, yv1 = 80, xv2 = 60, yv2 = 100;  // x11 defined in 1 st program group
volatile unsigned int vts[5] = { 0, 0, 0, 0, 0 }, vtstot = 0, j2a, j3a;
volatile unsigned char ch1 = 0x41, ch2 = 0x61, ch3, ch4 = 'j', ch5, ch6, ch7 = 'd', ch8, ch9, ch10, ch11, j, j1 = 0, j2 = 0, j3 = 0, j4, j5, j6, j7, j8, j9, j10, chr, TchScrch;  // ch1='A' & ch2='a', ch4=107 (97+10 ('k') )
volatile unsigned char j2old = 0, j3old = 0, j4old = 0, j5old = 0, j6old = 0, j2new = 0, j3new = 0, j4new = 0, j5new = 0, j6new = 0;
volatile unsigned char dimR;                       // dimR='milli'/blank/'kilo'
volatile unsigned char chs1 = 0, chs2, Kbkaz = 0;  //chs1-- old state of touch pad, chs2-- present state, Kbkaz='a'~'z'means some key is pressed. Kbkaz=0 means action has been taken
volatile unsigned char sgsy[8] = { '+', '+', '-', '-', '-', '-', '+', '+' };
volatile byte PrA, PrC;                    // data to be written into Arduino ports A & C
volatile byte SPrA = 04, SPrB = 00, SPrC;  // data to be wrtten into 'Slave' Ports (of 8255 on A4-D1 card) A,B,C , V5 =V5Low
volatile unsigned int wlk1[] = { 1, 2, 4, 0x08, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000 };
volatile float vr1 = 49.37, vr2, vr3, vr4, vr5, Vinp, KNp = 5000.0, KNm = 5000.0, Voffs = +0.0, SPVolt;  //27; ( for Gain=5.0 ),
volatile float IeMag[] = { 0.4, 1.0, 2.00, 5.0, 10.0, 20.0 }, Ieval = 2.0, wgt1, wgt2, icnA1;  // Ieval= 2.0 mAmp
volatile char st1[wsl] = { "+000085.12  g" }, st2[wsl], st3[wsl], st4[wsl] = { "" }, st5[wsl], st6[wsl], st7[wsl], st8[wsl], st9[wsl], st10[wsl];
volatile char Blnk[10] = { "         " }, Fpr = 'Q';  // 'Q' means it is in 'Test' mode 10 blanks,Fpr=0 or 'F',or 'G'....'P' (11 nos.)
const char FName[] = { "SUV9.txt" };
volatile char FName2[10] = { "Srv3.csv" }, StrName1[20], StrName2[20], StrName3[20];
volatile unsigned int Noffp = 100, Noffn = 0, Npp = 500, Npm = 0;  //(say, Npp= 50, Noffset+,- Np =,- (for Gain.5 -- Noffp =155
const int SerD = 22, SClk = 23, Lat = 24, SyncP = 25, Wr2 = 27;    // Serial  data out -D22,Serial clock pin=D23,LatchPin=D24. SyncP for OscScope
volatile byte A2_Cntrl, A4_Dt, A4_Cntrl, A4_KAv, A4_Rl, A4_DAC;
const String inStr = "499.98", Str3, Str5 = "Vp1p2", Str6 = "Resist", Str7, Str11 = "Sur4.csv";
volatile String Str10, Str12, Str13, Str14, Str15 = ("Sur4.csv"), Str16("#"), Str17, Str18, Str19, Str20;                                              // volatile Strings, "Suv" -- Survey
volatile unsigned int Tsat = 0, NtRn1 = 5, NtRn2 = 5, ht1 = 13, EAd = 0, EAd1 = 0, EAd2, EAd3, EAd4, EAd5, EAd6, EAd7, EAd8, EAd9, EAd10, EAd11;  //Colr[];
volatile float Resist[20], yfct = 0.4, Vp1p2, Res, Resx, Resm, fNsig2, BattV, tRes1, fltLv, fltlv, tRohm;                   // yfct=100/Npls[0] (should be Npls[1]
volatile float tRes2[5];                                                                                                    //   tRes2[70];
volatile signed int Npls[5], Ndots[5], Ycoord[5], NdtxA = 160, Nsig = 20, Nsig2, Nbck = 150, Nbckdt = 100, Slp = 1, IeNo2;  // Slp - slope =1 means Vp1p2 changes by Slp every Sec.
extern volatile byte SD_ok, sh_sg = 0;                                                                                      //(rs, en, d4, d5, d6, d7);
volatile byte in_Byte, last_lvl = 0, pres_lvl = 0, start_sending = 0, Recv = 0, First_Pls = 0, Ignr_pulses = 1, A1ReqO = 1, A1ReqN = 0, A1Powold = 0, A1PowN = 0, A1PSw, IeStat;
volatile byte Ack1, Actkn1 = 0, Actkn2 = 0;  // Acknoledge signal = D24 output. WHen this is '0', Auto-D starts sending data to DSU
volatile byte Recv_Buff1[100], RcBf_R1[100], GPS_rdng = 0;
volatile unsigned long F_cnt, Resint, ln8;                                                                                  //no. of pulses in 4 Sec, Resint- 'resistance' in int. form
volatile unsigned int icn[] = { 1000, 500, 250, 100, 50, 20, 10 }, CurrMag[] = { 5, 10, 20, 50, 100, 250, 500 }, RdNo = 0;  // 'multipliers' for 5,10,20, 50, 100,250,500 mA
volatile unsigned int Cycls[] = { 1, 4, 16, 64 }, PrCycl_No;                                                                // PrCycl_No-- present cycle no. viz. 1~4/16/64
volatile unsigned int IeMag2[] = { 0, 1, 2, 5, 10, 15, 20, 30, 40, 50, 70, 100, 200, 250, 300, 400, 500 };                  // for AutoC,nos-- 0,1~16 no. 0 is --null
volatile unsigned long Ntv[20], Ntvtot, Ltm2, Ltm1, tint, timr1, timr2 = 0, timr3, timr4, timr5, timr6, timr7, timr8, timr3old = 0, timr5old, timr6old, tmcn, tmcn5 = 0, timc1 = 0;  // Ltm1/2 & tint used for measuring time interval in milliSec.
volatile float Lv[] = { 1.5, 3, 5, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 230, 250, 270,
                        290, 300, 320, 340, 360, 380, 400, 420, 440, 460, 480, 500 };  //0~47 (total-48 float values)
volatile unsigned int Lt[] = { 15, 20, 25, 30, 40, 50, 60, 80, 100, 100, 120, 150, 200, 250, 250, 300, 400, 500, 600, 800, 1000, 1000, 1200, 1500, 1800,
                               2000, 2000, 2500, 3000, 3000, 3500, 4000, 4500, 5000, 5000, 5500, 6000, 6500 };  // 0~37 (total 38 integers),10* actual values
volatile unsigned int lt[] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 20, 20, 20, 20, 20, 50, 50, 50, 50, 50, 50, 50, 100, 100, 100, 100, 100, 200, 200, 200, 400, 400, 400, 400, 400, 500,
                               500, 500, 500 };       // 0~37 (total 38 integers),10* actual values
volatile unsigned int wrLlSD = 2;                     // if weLlSD==2,do not write L,l (float)  or a (float on SD
volatile float lv[] = { 0.5, 2, 5, 10, 15, 20, 30 };  // 0~ 6 (total-7)
volatile float Rest[5], Rho[5], Kv[5], Gain[4] = { 5, 0.5, 0.05, 0.005 }, tRes, tRho, MRdNmb[5], MLv[5], Mlv[5], MKv[5], MRes[5], MRho[5];        // ;  // Gain -- 4 values
volatile float Kvt;                                                                                                                               // a K-value
volatile byte Sccd[4] = { 0x01, 0x02, 0x04, 0x08 }, keyBf0 = 0, freezeSP = 0;                                                                     // Scan code to be changed every 10 mSec.
volatile unsigned int TotRd = 0, StRdNo = 1, Rds = 0, RdLNo = 0, StNos[] = { 1, 7, 13, 19, 25, 31, 37, 43, 49 }, Srv_No = 2, Survey_No_Stat = 0;  //('StRdNo' not used, tot readings,starting read. no.reading sets,starting no.s
volatile unsigned int trow, Llmax = 20, LlTbEd = 1, RdgEd = 0, SrvEd = 0;
volatile unsigned int LNo[80], lNo[4] = { 0, 1, 2, 3 }, ChPts[3] = { 3, 8, 13 }, SNo, LCNo[100], lCNo[100], RdNo1, MRdNo, MRdN1 = 0, Show_Alt = 1, ShSpcRd = 0, ShRdg = 1, CNomax;  //ShSpcN=1 means show spacings
volatile unsigned int tlim2, tlim3, tlim4, tlim5, tlim6, tlim7, tlim8, Lnlth = 0, LRdSr = 0, RdSr, IntSz, FltSz;                                                                    // Lnlth-- no. of char.s of type '0'~'9'/','/'.'. LRdSn- Last Reading Srl. No.(0~N)
volatile unsigned int SpcN, LSpcN, tm2, tm3, tm4, tm4old, TstE2p = 0, tm5, tm6, tm7, tm8, Ltm3, Ltm4, Ltm5;                                                                         // Spacing no, Last Spac. tm5,6 used in Timer5 Interrupt used
volatile unsigned int LSpcN2, LRdSr2, alph_st = 0, Norm_st = 0, StRecrds = 5, MSpcN, MLlNo, MLvt, Mlvt, tMLvt, tMlvt, chbfr = 0;                                                    //StRecrds-no. of records stored in  EAd6~EAd7 region. ; 3                         // LSpcN2,LRdSr2-last Reading & spacing nos., MSpcN-spacing no.as stored 'Readings list'.chbfe==0 means change by 1 m.,else 0.1 m.
volatile unsigned int xw2 = 70, yw2 = 50, nw2 = 1, noByRc;                                                                                                                          // no. of Bytes Recvd.
volatile unsigned int LCNomax = 100, lCNomax = 8, Lint1, Lint2, Lint3, lint1, lint2, lint3, Ldig1, ldig1, LlpSz, LNomax = 38, amax, Dip_a, Dip_n;                                   //Dipoe_Dipole metods' 'a' & 'n'LCNomax will get redefined
volatile unsigned int xn1, timr7_flag;                                                                                                                                              // used for printing 4 values of Resistance in 4-cycle mode
volatile unsigned int tn1 = 0, tn2, tn3, t_transf = 0, no_rdgs = 0, kpr = 0, kpr_key_2=1,F_kpr;                                                                                                 //tn1=0~14 (15 nos.),15~20(6 nos.),21~35 (15 nos.)& for 2nd,3rd & 4th cycles --(36~50)(51~65),(66~80),no. of readings
volatile float fact1, fact2, fact3, fact4, fact5 = 0, tenf[] = { 1, 10, 100 };  // different factors of Resistance 'Resm'
volatile unsigned int StRd[] = { 1, 2, 3, 4, 5 }, StL[] = { 15, 30, 50, 100, 150 }, StRd1, StL1, StL2, NRec = 5;
volatile float StRho[] = { 30.53, 25.41, 17.62, 20.28, 21.73 }, StRho1;  // Rho values
#define Kbin0 37                                                         // 4 return lines
#define Kbin1 39
#define Kbin2 41
#define Kbin3 43
#define otpin0 29  // scan output code on 4 pins
#define otpin1 31
#define otpin2 33
#define otpin3 35
//  define key-codes -------------------
#define k_1   0x11  // '1'
#define k_4   0x21  // '4'
#define k_7   0x41  // '7'
#define k_dot 0x81  // '.'
#define k_2 0x12    // '2'
#define k_5 0x22    // '5'
#define k_8 0x42    // '8'
#define k_zr 0x82   // '0'
#define k_3 0x14    // '3'
#define k_6 0x24    // '6'
#define k_9 0x44    // '9'
#define k_F 0x84    // 'F'
#define k_pr 0x18                                                                                  // "Prev"
#define k_nx 0x28                                                                                  // "Next"
#define k_cl 0x48                                                                                  //"Clear"
#define k_sv 0x88                                                                                  //"Save"
volatile int ldg2, lmin2, ldg0 = 0, ldg1 = 0, lmin0 = 0, lmin1 = 0, ldg0L, ldg1L, lmin0L, lmin1L;  //  ( ldg2,lmin2  not used presently)
//  for KeyBoard :. . . LKSt
volatile byte Kbout[4], sccdV[4] = { 0x0E, 0x0D, 0x0B, 0x07 }, RetL = 0, RetL2 = 0x41;
volatile struct {
  byte KSt;
  byte j;
  byte DbD[4];
  byte LKSt;
  byte DefKSt[2];
} Cl[4];  //
volatile byte RtLD, Curr_Sw, Range_Sw, Cycl_Sw = 2, Meas_Sw;  // Cycl_Sw=2 means 4-cycle mode  rtLD is read from kkbin0~3
volatile byte tick1[8] = { 0, 1, 2, 0x14, 8, 0, 0 }, tick2[8] = { 0, 1, 3, 0x16, 0x1c, 8, 0 };
volatile float Altit, lat, lon, ltg2, lsec2, lx2, frdg2, frmin2;  // Of these only lat,lon have been used presently
volatile float err0, err1, Rlsec0, Rlsec1;                        // err=(present)lsec0- (Reference ) Rlsec0. fAltit-Altitude(float)
volatile float frdg0, frdg1, lx0, lx1, frmin0, frmin1, lsec0, lsec1, ltg0, ltg1;
volatile unsigned int LnNo = 0, tLn = 0, Yr, YrL, EYr[4];
volatile byte Mn, Dte, Hr, mint, Scnd, EMn[4], EDte[4], EHr[4], Emint[4], EScnd[4], DeciSec, Cr, DteL, MnL, HrL, mintL, ScndL;  // E- extra
volatile unsigned long LAltit, f_age = 0, sz1, sz2, tsz1, tsz2;                                                                 // sz1,sz2 for file size, tsz1,tsz2-- total sizes
volatile unsigned int i1, i2, i3, i4, i5, EdSpc_No=2;  // 'Edit_Spacing_Srl. No.
volatile float latitude, longitude;  // from
/*******************************************************************************
  function:
        Paint the Delete key and paint color choose area
*******************************************************************************/
void TP_Dialog(void) {
}
// --    draw keyboard  a~z or A~Z
void Dr_Kb()  // draw keyBoard
{}
// ................................end of drw KeyBoard ,,,,,,,,,,,,,,,,,,,
void A4_Init() {
  InitTimr();    // Timers 3,5,0 initialized
  interrupts();  //enable all global interrupts
  j2 = 0;
  j3 = 0;
  in_Byte = 0;  //j2- no. of bits,j3 - no.of bytes, D22 logic level is shifted into 'in_byte'
  A1ReqN = digitalRead(23);
  A1ReqO = A1ReqN;  // initialization -----no longer used after Sept 2022--------------------
  tlim2 = 800 / 4;
  tlim3 = (tlim2 + 1) / 4;
  tlim4 = 3 * tlim3;  // tlim2,3,4 = 200,50,150  used in Timer3 10 mSec interrupt
  IntSz = 2;
  FltSz = 4;  //IntSz = sizeof(int); FltSz = sizeof (float);
  EAd1 = 0;
  EAd2 = 40;
  EAd3 = 120;
  EAd4 = 160;
  EAd5 = 460;
  EAd6 = 760;
  EAd7 = 1060;
  EAd8 = 2060;
  EAd9 = 2280;
  EAd10 = 2480;  // EAd1 ,2,3,4,5,6,7,8 redefined 6/March/2023, Monday
  Srv_No = 8;
  LSpcN = 7;
  LRdSr = 9;  //these values should get redefined when the statements LSpcN2<-- E2prom[tEA] etc are executed (2~4 lines from here)
  tEA = EAd1;
  EEPROM.get(tEA, Srv_No);  // Survey no. gets updated in mode F4
  //---------------------------Both (LRdSr2 & LSpcN2) become 0, when new survey is opened------------------
  tEA = EAd1 + (3 * IntSz);
  EEPROM.get(tEA, LRdSr2);  //  Read  integer 3 as LRdSr2 earlier: tEA=EAd1+(1*IntSz) ; Last Reading Sr. No & l. spac. no.
  tEA = EAd1 + (4 * IntSz);
  EEPROM.get(tEA, LSpcN2);  //  Read 4th integer as LSPcN2  Earlier: tEA=EAd1+(2*IntSz) ; Last Spac. no. (both get updated after each reading
  tEA = EAd1 + (5 * IntSz);
  EEPROM.get(tEA, StRecrds);  //no. of records stored i  EAd6~EAd7 region
  tEA = EAd1 + (6 * IntSz);
  EEPROM.get(tEA, Surv_meth);  //  Survey method - 1/2/3 -Schlum/Wenn/DipoleDipole
  tEA = EAd1 + (7 * 2);
  EEPROM.get(tEA, Survey_No_Stat);          // 0-- means No Survey is open presently, 1- means 'Srv_No' is the latest survey
  Str11 = "Srv" + String(Srv_No) + ".csv";  // e.g. Srv5.csvStr11="Srv"+String(Srv_No) + ".csv";// e.g. Srv5.csv
  A1_Power();  // this defines: A1PowN. A1PowN==1mens A1 card is powered On
  // --------------------calculation of about float 100 Lv[100] & lv[100] values---19/july/2022 --------------------------------------------------
  for (j2 = 0; j2 <= LNomax - 1; j2++) {  //LNomax = 38 ,presently
    LNo[j2] = j2;
  }  // end of j2 0~,++loop
  j2 = 0;
  j4 = 0;
  j5 = 0;
  dy1 = 0;
  dx1 = 0;
  for (j3 = 0; j3 <= LNomax - 1; j3++) {  //note:LNomax=38 (because, only 38 values have been defined (as of 13/Nov/2022, Sunday
    j5 = j3 + j2;
    LCNo[j5] = LNo[j3];
    lCNo[j5] = j2;
    if (j3 > 19) {
      dy1 = 35;
      dx1 = 23;
    }
    vr3 = lv[j2] * 18.0;
    if ((Lv[LNo[j3]] < vr3) && (vr3 <= Lv[LNo[j3 + 1]])) {  // one 'chpts' (change point detected)
      j2++;
      j4++;
      j5 = j3 + j2;
      LCNo[j5] = LNo[j3];
      lCNo[j5] = j2;  // ( j5 has changed, j2 (which was 0 originally,has changed, so j5=j3+j2 changes too
    }  // end of 'j5 has changed'
  }    // end of j3 loop
  LCNomax = j5 + 1;
  lCNomax = j2 + 1;  // now,LCNomax overrides the value declared in variable list. if LNomax=47, then LCNomax= ~47+8=55
  //  ............................end of study of L_l_Table ......................
  //  -------------------calculate all K values ---Scalck(float,float)defined on L 874----------
  for (j5 = 0; j5 <= LCNomax - 1; j5++) Kv[j5] = ScalcK(Lv[LCNo[j5]], lv[lCNo[j5]]);  // example:  LCNo= 0,1,2,3, 3,4,5 lCNo= 0,0,0,0, 1,1,1
  //  .................................end of calculate all K values.........................
  // ..................................End of 19/july/2022 .part...............................
  //  -------------------calculate all K values ---Scalck(float,float)defined on L 874----------
  for (j5 = 0; j5 <= 22; j5++) Kv[j5] = ScalcK(Lv[LCNo[j5]], lv[lCNo[j5]]);  // example:  LCNo= 0,1,2,3, 3,4,5 lCNo= 0,0,0,0, 1,1,1
  // -------- ----------------------------Define Survey file e.g. Srv15.csv ---------------------------------------------
  del1();
  IntSz = 2;
  FltSz = 4;
  for (m7 = 0; m7 <= 8; m7++) FName2[m7] = Str11[m7];
  //lcd1.setCursor(0, 2);lcd1.print("FNme");lcd1.print(Str11); // copy const string 'Str11' into const 'char' array
  //-------show new names ----------------------------------------------
  del1();
  //..................................................end of 'define Survey file...............................
  //---------------------------------write  into 'SD' . ---- Then read back & show it----------------------------
  xv1 = 60;
  yv1 = 80;  // xv2=60,yv2=100 -initially
  del1();
  // Define some parameters
  Npls[1] = Nbck + Nsig;
  Npls[2] = Nbck - Nsig;
  Npls[3] = Nbck - Nsig;
  Npls[4] = Nbck + Nsig;
  yfct = (float)(70.0 / (float)Npls[1]);
  j2 = 0;  // bit position in received byte
  if (digitalRead(22) == HIGH) {
    pres_lvl = 1;  // -----Should be done only when +D is on--------------------
    last_lvl = pres_lvl;
  } else {
    pres_lvl = 0;
    last_lvl = pres_lvl;
  }
  Recv = 0;  // ignore received data
  Erase2();
  j3 = 0;  // j3= no. of bytes receied on 'Serial1' initialized to 0
  kpr = 2;
  Fpr = 'Q';
  entry_fnc_Q();  // defined at power-On: 'Test' mode
  n16 = 0;
  E2prom_Lltbl(n16);  // actually,both Sclumberger & Wennersets are copied into EEPROM n16=0 means , it starts from EAd5 + 0
  lcd1.createChar(1, tick1);
  lcd1.createChar(2, tick2);
}
void show_some(){}
void GPS_waiting(){}
//----------------------------begin E2prom_put----------------
void E2prom_put() {
  tEA = EAd9; for (i5=0; i5>=15; i5++) {EEPROM.put(tEA,RcBf_R1[i5] ) ; tEA++; }
       ln8 = (RcBf_R1[6] * 0x100) + RcBf_R1[5];
        BattV = (float)ln8 / 100.0;
      EEPROM.put(tEA,BattV ) ;  //16 th numeri a float (4 bytes)
}
void E2prom_get() {
  tEA = EAd9; for (i5=0; i5>=15; i5++) {EEPROM.get(tEA,RcBf_R1[i5] ) ; tEA++; } // read off 16 bytes of received data
      // read15 bytes from E2prom 
    EEPROM.get(tEA,BattV ) ;  
}
 void Show_Recv_bytes() {
   for (i5=0; i5>=14; i5++) Serial.write(RcBf_R1[i5]) ;
 }
void printDir(File dir, unsigned int ntb) { 
}
void E2prom_Lltbl(unsigned int n1) {
  unsigned int i1;
  for (i1 = 0; i1 <= 37; i1++) {
    tEA = EAd4 + i1 * 4;
    EEPROM.put(tEA, pgm_read_word_near(PLt + i1));  // RAM to EEPROM, EAd4=160
    tEA += 2;
    EEPROM.put(tEA, pgm_read_word_near(Plt + i1));
  }  //EAd4=160,38 sets
  for (i1 = 0; i1 <= 22; i1++) {
    tEA = EAd5 + i1 * 2;
    EEPROM.put(tEA, pgm_read_word_near(Wennat + i1));
  }  // EAd5=460,PROGMEM to E2PROM  EAd5=460,23 sets
  for (i1 = 0; i1 <= 24; i1++) {
    tEA = EAd9 + i1 * 4;
    EEPROM.put(tEA, pgm_read_word_near(Dipat + i1));  // EAd9=2280, 2 integers(4 bytes) per record
    tEA += 2;
    EEPROM.put(tEA, pgm_read_word_near(Dipnat + i1));
  }  //EAd9=2280,25 sets
}
void Updt_DigInpLvls() {  // vr2 = strtof(st1);  // just for testing 'strtof'
}//-----------------------------calculate Batt. Voltage--(7/Sep/2022---------------------------------------------------------------
void calc_Batt() {
  BattV = (float)ln8 / 100.0;
  dtostrf(BattV, 5, 2, st1);                                                         // Batt Volt e.g. 12.83, RcBf_R1[7] expected to be 0
                                                                                     //-------------- do not clear entire lcd ----------------------------
  lcd1.setCursor(0, 0);
  if (Fpr == 'Q') lcd1.print("F0");
  if (Fpr == 'H') lcd1.print("F2");
  if (Fpr == 'J') lcd1.print("F4");
  lcd1.setCursor(13, 0);
  lcd1.print("BE=");
  lcd1.print(BattV, 2);  // now,'Sigma',1lcd1.clear();show at (13,0) B=12.68
  delay(2000);
  lcd1.setCursor(0, 3);
  lcd1.print("                   ");  // erase line-3 (press measure)
                                      //-----------------------show switch positions, only if RcBf_R1==3,i.e.Rannge switch pos. is on 'Bat'---------------------
  Curr_Sw = RcBf_R1[2];
  Range_Sw = RcBf_R1[3];
  Cycl_Sw = RcBf_R1[4];
  if (RcBf_R1[3] == 1)  // Range Switch on 'Batt' posn.
  {
    lcd1.setCursor(0, 3);
    lcd1.print("Cur Sw:");
    if (Curr_Sw <= 7) { lcd1.print("Auto pres F0"); }  // curr_Sw: line-3
    else
      lcd1.print("Error");
    lcd1.setCursor(0, 1);
    lcd1.print("Range Sw:");
    if (Range_Sw == 1) lcd1.print("Bat      ");  //Line-1: Range switch
    if (Range_Sw >= 2 && Range_Sw <= 4) lcd1.print("Resist");
    if (Range_Sw == 5 || Range_Sw == 6) lcd1.print("S.P.");
    if (Range_Sw >= 7) lcd1.print("Error");
    lcd1.setCursor(0, 2);
    lcd1.print("Cycl Sw:");
    if (Cycl_Sw <= 4) {
      lcd1.print(Cycls[Cycl_Sw - 1]);
      lcd1.print(" cycl");
    } else lcd1.print("Error");  //
  }
}
void curr_Status() {
  Serial.println("curr_Status_Start");
  if (RcBf_R1[17] == 1) IeStat = 0;
  else IeStat = 1;  // IeStat =0, means 'current not flowing
  if (IeStat == 1) {  // Current flowing ok
    IeNo2 = RcBf_R1[18];
    if (Fpr != 'Q' && Range_Sw != 1) Show_LlK();               // 'show L,l K on line 1,' is blocked , if in 'Test' mode or if in Batt posn.
    lcd1.setCursor(13, 0);
    lcd1.print("I=");
    lcd1.setCursor(15, 0);
    lcd1.print("     ");
    lcd1.setCursor(15, 0);
    lcd1.print(IeMag2[IeNo2]);
    lcd1.print("mA");  // first erase 15~19,then current magnitude
    lcd1.setCursor(0, 0);
    if (Fpr == 'Q') lcd1.print("F0");
  }  //
  else {                                                                                        // IeStat==0,means 'No current'
    lcd1.setCursor(8, 0);
    lcd1.print("-No Curr--- ");
    lcd1.setCursor(3, 1);
    lcd1.print("mode");
    lcd1.setCursor(0, 3);
    lcd1.print("-check connections-");
    tn1 = 0;
    tn2 = 0;
    t_transf = 0;  // So DSU knows that Auto_D will now turn off
  }
  Serial.println("curr_Status_End");
}
void Show_LlK(void) {}
void calc_Res() {
  Serial.println("calc_Res_Start");
  vr2 = (float)RcBf_R1[31] + (float)RcBf_R1[32] * 256.0 + (float)RcBf_R1[33] * 65536.0;  //
  ln8 = ((long)RcBf_R1[25] * (long)0x10000) + (long)RcBf_R1[24] * (long)0x100 + (long)RcBf_R1[23];
  icnA1 = ((float)ln8 / 256.0) * tenf[RcBf_R1[26] - 1];  //Gain[0~3]-- 5/0.5/0.05/0.005
  Resx = (vr2 * icnA1 * (5.0 / Gain[RcBf_R1[28]]));      // Resx= (pulse count over 4 Sec.)*multiplier*(5/actual gain)
  Resm = Resx / 1000.0;  // Resm --Resistance in milli-ohm on dividing 'Resx'by 1000, we get value in milliOhm
  fact1 = (float)ln8 / 256.0;
  fact2 = tenf[Recv_Buff1[5] - 1];
  fact3 = 5.0 / Gain[Recv_Buff1[7]];
  fact4 = 4.9883 / 100;
  dtostrf(fact4, 8, 5, st1);
  if (Surv_meth == 1){} 
  if (Surv_meth == 2){}
  if (Surv_meth == 3){}
  Serial.print("|F_cnt|");
  Serial.print(vr2);
  Serial.print("| ");
  Serial.print(RcBf_R1[31]);
  Serial.print(" ");
  Serial.print(RcBf_R1[32]);
  Serial.print(" ");
  Serial.print(RcBf_R1[33]);
  Serial.print(" ");
  Serial.print(" |icnA1|- ");
  Serial.print(icnA1);
  Serial.print(" exp-");
  Serial.print(RcBf_R1[26] - 1);
  Serial.print(" ");
  Serial.print("Gain No ");
  Serial.print(RcBf_R1[28]);
  Serial.print(" ");
  if (Resm <= 1200.0) {
    dimR = 'm';
    Rest[RdNo] = Resm;
    tRes = Resm;                 // Resistance  in milliOhm
    tRho = Kvt * (Resm / 1000);  // (Resm/1000) is in 'Ohm'
  }                              //GUI_DrawRectangle(270, y7 + 40, 270 + 70 , y7 + 53 , Colr[7], DRAW_FULL, DOT_PIXEL_DFT); // erase rect for value
  else {
    tRes = Resm / 1000;  //now tRes is in 'ohm'
    tRho = Kvt * tRes;   // tRho is in ohm-meter
    if (tRes <= 1200.0)  // if tres<=1200 ohms
    {
      dimR = ' ';    //  now tRes is in 'ohm'
      tRohm = tRes;  //tRohm will be anything from 0.0001 to 10,000 ohm
    } else {
      tRes /= 1000.0;
      dimR = 'k';  //now Res is in kohm
    }
  }
  if ((Fpr != 'Q') && ((Cycl_Sw == 1) || ((Cycl_Sw == 2) && PrCycl_No == 4))) {
    if ((Surv_meth == 1) || (Surv_meth == 2))  // Schlumberger or Wenner-- store 8 bytes
    {}
    if (Surv_meth == 3){}
  } 
  dtostrf(tRes, 9, 2, st1);  //
  dtostrf(tRho, 9, 2, st1);  //
  // For testing:--LSpcN++not done; update Last Spacing No
  if (Cycl_Sw == 1)                                                      // 1-cycle
  {
    lcd1.setCursor(0, 1);
    lcd1.print("                   ");
    lcd1.setCursor(19, 1);
    lcd1.print(" ");  // erase entire line-1 & ch. (19,1)
    lcd1.setCursor(8, 1);
    lcd1.print("R=");
    lcd1.print(tRes, 2);
    if (dimR == 'm') lcd1.print("m");
    if (dimR == ' ') lcd1.print(" ");
    if (dimR == 'k') lcd1.print("k");
    lcd1.write(0xF4);  // lcd1.setCursor(4, 0);F4h is 'ohm'
    if (Fpr != 'Q') {  //
    } else {
      lcd1.setCursor(0, 2);
      lcd1.print("-Res. not stored- ");
    }
    lcd1.setCursor(0, 3);
    lcd1.print(" ");
    lcd1.setCursor(0, 0);
    if (Fpr == 'Q') lcd1.print("F0");
    if (Fpr == 'H') lcd1.print("F2");
    if (Fpr == 'J') lcd1.print("F4");
    if (Fpr != 'Q') {
      lcd1.setCursor(0, 3);
      lcd1.print("press 6 for next");  // message for next operation
      lcd1.setCursor(12, 0);
      lcd1.print("       ");
      lcd1.setCursor(19, 0);
      lcd1.print(" ");
    }  // erase chars. 12~19 & ch. (19,0)
    else {
      lcd1.setCursor(0, 3);
      lcd1.print(" ");
    }                  // 'Test mode
    if (Fpr != 'Q') {  // i.e. F2, Survey mode
      if (Surv_meth == 1)  // Schlumberger
      {
        if (Ldig1 == 0) {        }  // L integer, at (12,0)
        else {}  // L-Float at (12,0)
      }
      if (Surv_meth == 2)  // Wenner method
      {
        if (Ldig1 == 0) {}  // L integer, at (12,0)
        else {}  // L-Float at (12,0)
      }
    }
  }
  if (Cycl_Sw == 2 || Cycl_Sw == 3 || Cycl_Sw == 4)  // 4/16/64 cycls mode
  {
    lcd1.setCursor(12, 0);
    lcd1.print("       ");
    lcd1.setCursor(19, 0);
    lcd1.print(" ");  // erase chars. 12~19 & ch. (19,0)
    if (Fpr != 'Q') {
      //------------------------------------------no printing of 'L' in 'test' mode----(4-cycle mode---------------------------------------
      if (Ldig1 == 0) {  //L=--- at top right corner
        lcd1.setCursor(13, 0);
        lcd1.print("L=");
        lcd1.print(Lint3);
      }  // L integer, at (12,0)
      else {
        dtostrf(fltLv, 7, 1, st1);
        lcd1.setCursor(13, 0);
        lcd1.print("L=");
        lcd1.print(fltLv, 1);
      }  // L-Float at (12,0)
    }
    //- - - - - - - - Cycl-1- - - - - - - - - - -- - - - - - - - - --  - - - - - - - - - -
    if (PrCycl_No == 1) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(19, 1);
      lcd1.print(" ");  // erase entire line-1 & ch. (19,1)
      lcd1.setCursor(3, 1);
      lcd1.print("R(");
      lcd1.print(PrCycl_No);
      lcd1.print("/4)=");
      lcd1.print(tRes, 2);  //erase 0~2
      if (dimR == 'm') lcd1.print("m");
      if (dimR == ' ') lcd1.print(" ");
      if (dimR == 'k') lcd1.print("k");
      lcd1.write(0xF4);  //  F4h is 'ohm'
    }                    //
    if (PrCycl_No == 2) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(19, 1);
      lcd1.print(" ");  // erase entire line-1 & ch. (19,1)
      lcd1.setCursor(3, 1);
      lcd1.print("R(");
      lcd1.print(PrCycl_No);
      lcd1.print("/4)=");
      lcd1.print(tRes, 2);  //erase 0~2
      if (dimR == 'm') lcd1.print("m");
      if (dimR == ' ') lcd1.print(" ");
      if (dimR == 'k') lcd1.print("k");
      lcd1.write(0xF4);  //  F4h is 'ohm'
    }                    //
    if (PrCycl_No == 3) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(19, 1);
      lcd1.print(" ");  // erase entire line-1 & ch. (19,1)
      lcd1.setCursor(3, 1);
      lcd1.print("R(");
      lcd1.print(PrCycl_No);
      lcd1.print("/4)=");
      lcd1.print(tRes, 2);  //erase 0~2
      if (dimR == 'm') lcd1.print("m");
      if (dimR == ' ') lcd1.print(" ");
      if (dimR == 'k') lcd1.print("k");
      lcd1.write(0xF4);  //  F4h is 'ohm'
    }                    //
    if (PrCycl_No == 4)  // ----------last of 4 cycles------------------------
    {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(19, 1);
      lcd1.print(" ");  // erase entire line-1  & ch. (19,1)
      lcd1.setCursor(3, 1);
      lcd1.print("R(");
      lcd1.print(PrCycl_No);
      lcd1.print("/4)=");
      lcd1.print(tRes, 2);  //erase 0~2
      if (dimR == 'm') lcd1.print("m");
      if (dimR == ' ') lcd1.print(" ");
      if (dimR == 'k') lcd1.print("k");
      lcd1.write(0xF4);  //  9Ah is 'ohm'
     if (Fpr != 'Q') {
        lcd1.setCursor(0, 2);
        lcd1.write(0xE6);
        lcd1.print("=");
        lcd1.print(tRho, 2);
        lcd1.print("");
        lcd1.write(0xF4);
        lcd1.print("m");  //4th line, E6 means Rho
        lcd1.print("-stored-");
      } else {
        lcd1.setCursor(0, 2);
        lcd1.print("-Res. not stored- ");
      }  // 'F0', 'Test' mode
      lcd1.setCursor(0, 3);
      lcd1.print("press 6 for next");  // message for next operation
    }                                  //
  }  // end of 'if '4-Cycle mode'
  if (Cycl_Sw == 1 || (PrCycl_No == 4 && (Cycl_Sw == 2 || Cycl_Sw == 3 || Cycl_Sw == 4))) {
    if (Fpr != 'Q')  // {writing into 'SD' not to be done when Fpr==0, Test mode)
    {}               // end of writing into 'SD''
    Serial.println("calc_Res_End");
  }  // end of writing into 'SD'
}  //..........................................end of 'calc_Res'............................................
void Recv_Serial2() {  //j3a=0 to be done at initialization time          xv1=60 & yv1==80 initially
  while (Serial2.available() > 0) {  // data expected from Serial-2 channel (Serial-1 is for GPS module) as of 8/August/2022)
    ch10 = Serial2.read();           //Recv_Buff1[j3a] = ch10; j3a++;
    if (xv1 >= 400) {
      yv1 += 16;
      xv1 = 0;
    }
    Serial.print(ch10) ;
    RcBf_R1[tn1] = ch10;
    if (t_transf == 1) {
      Recv_Buff1[tn2] = ch10;  // copy ch10 into Recv_Buff1 (15 bytes)
      tn2++;
    }
    n15 = ch10 & 0x0F;
    if (n15 <= 9) ch6 = 0x30 + n15;
    else ch6 = 0x41 + (n15 - 10);                            // calculate 1st nibble (lower nibble only) (later try st3=String (n14,HEX);)
    n15 = (ch10 & 0xF0) >> 4;
    if (n15 <= 9) ch8 = 0x30 + n15;
    else ch8 = 0x41 + (n15 - 10);                        //calculate 2nd nibble (ch8)
    if (RcBf_R1[tn1] == 0xFF && RcBf_R1[tn1 - 1] == 0xFF) {
      xv1 += 20;
      xv1 += 20;
      xv1 += 20;
      xv1 = 60;
      yv1 += 12;
      tn1++;  // // show resistance
      if (tn1 == 15) { 
        E2prom_put() ; // store RcBf_R1 [0~15 byttes in EEPROM]
        calc_Batt();
        if (Range_Sw != 1 && Fpr != 'Q');
      }  // do not call Show_LlK ,if in Batt position,, or in 'Test'mode. show 15 bytes, expected to be 01,0Fh,.....upto FFh,FFh
      if (tn1 == 21) curr_Status();
      if (tn1 == 21) {
        tn2 = 0;
        tn3 = 0;
        t_transf = 1;
      }  // tn2 will go from 0 to 14 (15 bytes)
      if (Cycl_Sw == 1 && (tn1 == 35 || tn1 == 36)) {
        xn1 = 0;
        PrCycl_No = 0;
        calc_Res();
        tn1 = 0;
      }  //  && means 1-cycle only. after 'calc_Res' make tn1=0;  // end of 'if Cycl_Sw==1/2/3/4
    }    //end of 'if 2 bytes=FFh,FFh
    else {
      xv1 += 20;
      tn1++;
    }  // if (t_transf==1) tn2++;  at Ln ~ 1475
  }  // end of 'if Serial2 available
}
void Show_ResistData() {
}
float WcalcK(float av) {}
float ScalcK(float Lv, float lv) {}
float DipcalcK(unsigned int a, unsigned int n) {}
void Erase1(void) {}
void Erase2(void) {}
void get_Hex(byte x) {}
void show_ByRcvd() {}
void Updt_RecD(void) {}
void A1_Power() {}
void check_Keyboard() {}
void Show_Eprom2(unsigned int L, unsigned int l) {}
void Show_Eprom3(unsigned int L, unsigned int l) {}
void Kb_Action(){}
void Reject_k() {}
void entry_fnc_J(){}
void entry_fnc_J1(){}
void entry_fnc_J2(){}
void fnc_J2(){}
void fnc_J3(){}
void fnc_J4(){}              // make survey no = 30,(say)
void stat_mod_J5(){}
void stat_mod_j6(){}
void stat_mod_J7(){}
void stat_mod_J8(){}
void entry_fnc_I(){}
void fnc_I2(){}
void fnc_I3(){}
void fnc_I4(){}
void fnc_I5(){} 
void fnc_I6(){}
void fnc_I7(){}
void fnc_I8(){}
void fnc_I9(){}
void fnc_I10(){}
void entry_fnc_G() {}
void fnc_G2() {}
void fnc_G3() {}
void fnc_G4() {}
void entry_fnc_H() {}
void fnc_H2() {} //.....................................end of fnc_H2......................
void fnc_H3() {} 
void fnc_H4() {}
void fnc_H6() {}
void fnc_H9(){}
void fnc_H_Prv(){}
void fnc_H_Nxt() {}
void entry_fnc_K(){}
void fnc_K2() {}
void fnc_K3() {}  //---
void entry_fnc_Q() {
  unsigned int i1;  //show Resistance  only. No L,l,K,nor 'Rho'
  tn1 = 0;
  tn2 = 0;
  t_transf = 0;  // this means no. of chars. received at Serial2=0. This may help
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F0");
  lcd1.print(" Test mode");
  lcd1.setCursor(0, 1);
  lcd1.print("");
  lcd1.setCursor(5, 2);
  lcd1.print("          ");  // set cursor at (char. 0,line-2)
  lcd1.setCursor(0, 3);
  lcd1.print("--press Measur(9) ");
}
void fnc_Q2()  //---- when key_9 is pressed
{
  digitalWrite(27, LOW);
  timr7 = 0;
  timr7_flag = 1;  //  D27<--0  timr7 starts with '0'
}
void fnc_Q1(){}
void L_Init1(void) {}
void L_Scr_Schlum(void){}
void L_Scr_Wenn(void){}
void L_Scr_Dip(void){}
void Show_LlK3(unsigned int n1, unsigned int nL, unsigned int nl) {}
void Show_LlK2(unsigned int n1) {}
void Screen_1(){}
void Screen_2(){}
void Screen_3(){}
void Screen_4(){}
void Alpha_1(){}
void Alpha_2(){}
void Alpha_3(){}
void Normal_1(){}
void Wr_A2A4(){}
void Wr2_pulse(){}
void Test_Port(void) {}  //  ------ end of function Test_Port  ------------------------
void Show_Timr5(void) {}
byte Get_key() {}
ISR(TIMER3_COMPA_vect){}
void Show_Spc(int Sp) {}
ISR(TIMER5_COMPA_vect){}
ISR(TIMER0_COMPA_vect){}
void Updt_Displ(void) {}
void Show_wt(char* st2) {}
void A4_D1_DAC(byte byt1){}
byte RevBits(byte Num) {}
void del1() {}
void wrt_Pr(void) {}
void Get_GPS2(void) {}
void Get_GPS(void) {}
void TP_DrawBoard(void) {}  
