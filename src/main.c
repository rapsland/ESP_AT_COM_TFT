/**************************************************************************************************/
/* Project DEYE Inverter [this:ESP32]--UART-->[ESP32 AT-System]--WIFI-->[Deye 600/800 Inverter]   */
/*                             ||
/*                            -SPI-     
/*                             ||           
/*                        [TFT_DISPLAY]                                                           */
/* Version 0.5.3 */
/* Can establish communication with DEYE Inverter */
/* Prerequisites:
/* 1 x ESP32-S or WROOM (used: ESP32-D0WD-V3 (revision v3.0))*/
/* 1x ESP32 with AT-Firmware 
   1x TFT (240x320)
   1x WIFI 
   1x Deye SUN300/600/800-G3-EU230 */
/* Read:                                   | Read_CMD | RegAdd | NumOfReg | CRC16 |
                                           | ---------|--------|----------|-------|  
          - Active Power Regulation  (APR) |  0103    |  0028  |   0001   |  0402 |
          - Radiator Temperature           |  0103    |  005A  |   0001   |  A419 |  
          - Current Power                  |  0103    |  0056  |   0001   |  641A |
          - Daily Yield                    |  0103    |  003C  |   0001   |  4406 |
          - Total Yield                    |  0103    |  003F  |   0002   |  F407 |
v0.5.3:
  - Added ESP32 Logo

v0.5.1:
- Added APR Write function:
  o Testing toggle between 2% and 100%.          
  o using addtitional button to force update
- Added Current meas HW comm SW but not integrated.
- Added Version info in display
- Added Runtime
TODO: 
- Documentation (comments, AT-Command Structure, ESP32 AT-CMDs, Register adresses) (prio1)
- Error notification if the inverter is offline (prio2) -Done: tbd use ping method later
- Refine function and print structure (prio2) 
- Add other Button inputs (prio2) 
- Move WIFI information to external header (prio3)
- update wifi state, notify if connection not possible (prio3)
- Add DAte and Time to inform latest update.
- Check date and update (not needed if current measurement is implemented)
- CRC16 Modbus calculation (prio2)
Next Steps:
- Add Current Measurement
- Add adapt APR according to Current (power) measurement
ToBeTested:
- Currentpower, yield range (was not possible yet, no sun!!)

## Structure of commands

### Read commands

```
DATAGRAM	:= ATCMD + MODBUSLEN + SEPERATOR + MODBUSMSG + MODBUSCRC + NEWLINE
ATCMD		:= AT+INVDATA=
MODBUSLEN	:= len(MODBUSMSG + MODBUSCRC)
MODBUSCRC	:= crc(MODBUSMSG)
SEPERATOR	:= ,
MODBUSMSG	:= SLAVE + FCODE + STARTADDR + REGSIZE
SLAVE		:= 01
FCODE		:= 03
STARTADDR	:= FFFF
REGSIZE		:= 0001
VALUELEN   	:= len(VALUE)
VALUE		:= 0000
NEWLINE     := \n
```
### Write commands
```
DATAGRAM	:= ATCMD + MODBUSLEN + SEPERATOR + MODBUSMSG + MODBUSCRC + NEWLINE
ATCMD		:= AT+INVDATA=
MODBUSLEN	:= len(MODBUSMSG + MODBUSCRC)
MODBUSCRC	:= crc(MODBUSMSG)
SEPERATOR	:= ,
MODBUSMSG	:= SLAVE + FCODE + STARTADDR + REGSIZE + VALUELEN + VALUE
SLAVE		:= 01
FCODE		:= 10
STARTADDR	:= FFFF
REGSIZE		:= 0001
VALUELEN   	:= len(VALUE)
VALUE		:= 0000
NEWLINE     := \n
```

*/

/**************************************************************************************************/

#include <TFT_eSPI.h>       // TFT Display Library
#include <WiFi.h>           // WiFi Library
#include <HardwareSerial.h> // HW UART Library
#include "wifi_credentials.h"
#include "ESP32Logo.h"
// #include <PZEM004Tv30.h>    //Power measurement library



/* info FONT2 is 16 pixels 
        FONT4 is 26 pixels
/*
 * There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
 * 
 * U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
 * U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
 * U2UXD is unused and can be used for your projects.
 * 
*/
  /* AT-UART COMM PORTS */
  #define RXD2 16 //UART RX PIN
  #define TXD2 17 //UART TX PIN
  HardwareSerial SerialPort(2); // use UART2
  HardwareSerial PzemSerialPort(1);
/* Current Meas with PZEM, UART DEF  START-*/
  // #define PZEM_RX_PIN 12
  // #define PZEM_TX_PIN 13

  // #define PZEM_SERIAL SerialPort(1); //use UART1
  // PZEM004Tv30 pzem(PzemSerialPort, PZEM_RX_PIN, PZEM_TX_PIN);
/* Current Meas with PZEM, UART DEF  END-*/

  #define BOOTPIN 0 //used for restarting REgister REad
  #define BUTTON_D12 12
  #define DISPLAY_LED_PIN 27
  #define DISPLAY_ON_TIME 60000000 //microseconds: 60s
  #define ONE_SEC_IRQ 1000000 //1 second

  #define SERIAL_INFO_EN 1 // 1: serial monitor output enable
  #if SERIAL_INFO_EN == 0
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
  #define debuglnDec(x,y) Serial.println(x,y)
  #else
  #define debug(x) 
  #define debugln(x)
  #define debuglnDec(x,y)
  #endif

  TFT_eSPI tft = TFT_eSPI();

  String SW_Version = "0.5.3";

  String ATCommand;
  String IncomingStr;

  String ATCMD;
  int UdpReadATState = 0;
  int InvalidRegisterVal = 0;

  int init_b;
  int seconds = 0;
  int minutes = 0;
  int hours = 0;
  int DispTimeout = 60;
  int xpow = 0;
  int currentpower = 0;
  int currentAPR = 0;
  float xpow_f = 0.0;
  int testcounter = 0;
  int ButtonState0 = 0;
  int ButtonState12 = 0;
  int statechange = 0;

  int DisplayTimer = 1;
  long GlobalTimer = 1;

  char *endptr;
  char input[] = "00";

  

hw_timer_t *My_timer = NULL;

void IRAM_ATTR onTimer()
{
  // printf("T0 Interrupt");
  DisplayTimer++;
  GlobalTimer++;
  if(GlobalTimer%60 == 0)
  {
    minutes++;
  }
  // timerAlarmDisable(My_timer);
}


// void PZEMsetup() {
//     /* Debugging serial */
//     // SerialPZEM.begin(115200);
//     PzemSerialPort.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
// }

void UARTSetup()
{
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  // Serial.begin(115200);
  // //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  // Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  // Serial.println("Serial Txd is on pin: "+String(TX));
  // Serial.println("Serial Rxd is on pin: "+String(RX));

  SerialPort.begin(115200, SERIAL_8N1, 16, 17); 
}

void UartloopTest() 
{ //Choose Serial1 or Serial2 as required
  //SerialPort.println("AT+GMR");
  while(Serial.available())
  {
    ATCommand = Serial.readString();
    SerialPort.println(ATCommand);
    delay(50);
  }
  while (SerialPort.available())
  {
    Serial.println(SerialPort.readString());
  }
  ATCommand ="";

}

/*convert hexstring to Decimal (used)*/
long hexStringToDecimal(String hexString) {
  long decimalValue = 0;
  int hexLength = hexString.length();
  
  for (int i = 0; i < hexLength; i++) {
    char c = hexString.charAt(i);
    int digitValue;
    
    if (c >= '0' && c <= '9') {
      digitValue = c - '0';
    } else if (c >= 'A' && c <= 'F') {
      digitValue = 10 + (c - 'A');
    } else if (c >= 'a' && c <= 'f') {
      digitValue = 10 + (c - 'a');
    } else {
      // Handle invalid characters if necessary
      continue;
    }
    
    decimalValue = (decimalValue * 16) + digitValue;
  }
  
  return decimalValue;
}

void Uartloop() 
{ 
  delay(100);
  while (SerialPort.available())
  {

    debugln("---------------------------");
    IncomingStr = SerialPort.readString();

    debugln(UdpReadATState);
    debugln("\n");

    if(UdpReadATState == 8)
    {
      if(SERIAL_INFO_EN)
      {
      debugln("+++++++++++++++++++++++++++");
      debugln("A. Power Regulation:");
      }
      String value = IncomingStr.substring(67,69);
      xpow = hexStringToDecimal(value);
      currentAPR = xpow;
      debuglnDec(xpow, DEC);
      // TFT Print --start------------
      tft.fillRect(155, 37, 80, 15, TFT_BLACK);
      // tft.drawString("A. Power Regulation :",10,40,2);
      tft.drawString("%",190,37,2);
      tft.drawString(String(xpow),155,37,2);
      // TFT Print -- end ------------
      //-Check again if Inverter is Online 
      if(xpow < 1)
      {
        InvalidRegisterVal++;
        if(InvalidRegisterVal<2)
        {
          UdpReadATState = 0;
          delay(200);
        }else
        {
          //inverter is probably offline
           tft.setTextColor(TFT_RED);
           tft.drawString("Inverter is Offline!!",10,165,4);
           tft.drawString("(Try again later or check daylight)",10,185,2);
           tft.setTextColor(TFT_WHITE);
           delay(200);
        }
      }else
      {
        InvalidRegisterVal = 0;
        tft.fillRect(0, 165, 240, 45, TFT_BLACK);
      }
      //
      if(SERIAL_INFO_EN)
      {
      debugln("-+-----------+-----------+-");
      debugln("\n");
      debugln(value);
      }
    }else if(UdpReadATState == 14)
    {
      if(SERIAL_INFO_EN)
      {      
      debugln("+++++++++++++++++++++++++++");
      debugln("Radiator Temp:");
      }
      String value1 = IncomingStr.substring(64,66);
      String value2 = IncomingStr.substring(67,69);
      String value = value1+value2;

      xpow = hexStringToDecimal(value);
      xpow = xpow - 1000;
      xpow_f = (float)xpow/100;
      debugln(String(xpow_f));

      // TFT Print --start------------
      tft.fillRect(155, 58, 80, 17, TFT_BLACK);
      tft.drawString(String(xpow_f),155,60,2);
      // tft.drawString("Radiator Temp :",10,60,2);//do not refresh everytime
      tft.drawString("`",205,60,2);
      tft.drawString("C",210,60,2);
      // TFT Print -- end-------------
      if(SERIAL_INFO_EN)
      {
      debugln("-+-----------+-----------+-");
      debugln("\n");
      debugln(value);
      }
    }else if(UdpReadATState == 16)
    {
            if(SERIAL_INFO_EN)
      {
      debugln("+++++++++++++++++++++++++++");
      debugln("Current Power:");
      }
      String value1 = IncomingStr.substring(64,66);
      String value2 = IncomingStr.substring(67,69);
      String value = value1+value2;
      // xpow = value.toInt();
      xpow = hexStringToDecimal(value);
      currentpower = xpow/10; 
      debugln(String(currentpower));
      // TFT Print --start------------
      tft.fillRect(155, 75, 80, 18, TFT_BLACK);
      // tft.drawString("Current Power :",10,80,2);//do not refresh everytime
      tft.drawString(String(currentpower),155,80,2);
      tft.drawString("W",205,80,2);
      // TFT Print -- end-------------
      if(SERIAL_INFO_EN)
      {
      debugln("-+-----------+-----------+-");
      debugln("\n");
      debugln(value);      
      }
    }else if(UdpReadATState == 20)
    {
      if(SERIAL_INFO_EN)
      {
      debugln("+++++++++++++++++++++++++++");
      debugln("Yield Today:");
      }
      String value1 = IncomingStr.substring(64,66);
      String value2 = IncomingStr.substring(67,69);
      String value = value1+value2;
      if(SERIAL_INFO_EN)
      {      
      debugln(value1);
      debugln("\n");
      debugln(value2);
      }
      // xpow = value.toInt();
      xpow = hexStringToDecimal(value);
      xpow_f = (float)xpow/10;
      debuglnDec(xpow, DEC);
      // TFT Print --start------------
      // tft.drawString("Yield Today :",10,100,2);//do not refresh everytime
      tft.fillRect(155, 100, 80, 18, TFT_BLACK);
      tft.drawString(String(xpow_f),155,100,2);
      tft.drawString("Kwh",205,100,2);
      // TFT Print -- end-------------
            if(SERIAL_INFO_EN)
      {
      debugln("-+-----------+-----------+-");
      debugln("\n");
      debugln(value);      
      }
    }else if(UdpReadATState == 18)
    {
      if(SERIAL_INFO_EN)
      {
      debugln("+++++++++++++++++++++++++++");
      debugln("Yield Total:");
      }
      String value1 = IncomingStr.substring(64,66);
      String value2 = IncomingStr.substring(67,69);
      String value = value1+value2;
      if(SERIAL_INFO_EN)
      {      
      debugln(value1);
      debugln("\n");
      debugln(value2);
      }
      xpow = hexStringToDecimal(value);
      xpow_f = (float)xpow/10;
      debuglnDec(xpow, DEC);
      // TFT Print --start------------
      // tft.drawString("Yield Total :",10,120,2); //do not refresh everytime
      tft.fillRect(155, 125, 80, 15, TFT_BLACK);
      tft.drawString(String(xpow_f),155,127,2);
      tft.drawString("Kwh",210,127,2);
      // TFT Print -- end-------------
      if(SERIAL_INFO_EN)
      {      
      debugln("-+-----------+-----------+-");
      debugln("\n");
      debugln(value);      
      }
    }else if(UdpReadATState == 22)
    {
      if(SERIAL_INFO_EN)
      {
      debugln("+++++++++++++++++++++++++++");
      debugln("Ping Time:");
      }
      String value = IncomingStr.substring(6,9);
      if(SERIAL_INFO_EN)
      {      
      debugln(value);
      debugln("\n");
      }
      // TFT Print --start------------
      tft.fillRect(28, 10, 100, 16, TFT_BLACK);
      tft.setTextColor(TFT_WHITE);
      tft.drawString("PingTime :",5,10,2); //do not refresh everytime
      tft.drawString(String(value),80,10,2);
      tft.drawString("ms",115,10,2);
      // TFT Print -- end-------------
      if(SERIAL_INFO_EN)
      {      
      debugln("-+-----------+-----------+-");
      debugln("\n");
      debugln(value);      
      }
      delay(200);
    } else if(UdpReadATState == 30)
    {         
      debugln(IncomingStr);
      debugln("\n");
      debugln("###########################");
     
    }
    else
    {
      if(SERIAL_INFO_EN)
      {      
      debugln(IncomingStr);
      debugln("\n");
      debugln("###########################");
      }
    }
  }
  ATCommand ="";

}

void UartProcess()
{
  switch(UdpReadATState)
  {
  case 0:
    SerialPort.println("ATE0"); //echo off
    delay(100);
    ATCMD = "AT+CIPSTART=\"UDP\",\"192.168.178.39\",48899";
    SerialPort.println(ATCMD);
    UdpReadATState = 1;
    break;
  case 1:
    delay(20);
    ATCMD = "AT+CIPSTATUS";
    SerialPort.println(ATCMD);
    UdpReadATState = 2;
    break;
  case 2:    
    delay(20);
    ATCMD = "AT+CIPSEND=19";
    SerialPort.println(ATCMD);
    UdpReadATState = 3;
  break;
  case 3:  
    delay(20);
    ATCMD = "WIFIKIT-214028-READ";
    SerialPort.println(ATCMD);
    UdpReadATState = 4;
  break;
  case 4:
    delay(20);
    ATCMD = "AT+CIPSEND=3";
    SerialPort.println(ATCMD);
    UdpReadATState = 5;    
  break;
  case 5:
    delay(20);
    ATCMD = "+ok";
    SerialPort.println(ATCMD);
    UdpReadATState = 6;    
  break;
    case 6:
    delay(20);
    ATCMD = "AT+CIPSEND=30";
    SerialPort.println(ATCMD);
    UdpReadATState = 7;    
  break;
  case 7:
    delay(50);
    ATCMD = "AT+INVDATA=8,0103002800010402"; //Active Power Regulation
    SerialPort.println(ATCMD);
    UdpReadATState = 8;    
  break;
  case 8:
    delay(50);
    ATCMD = "AT+CIPSEND=30";
    SerialPort.println(ATCMD);
    UdpReadATState = 13;    
  break;
  case 13:
    delay(50);//Radiator Temp
    ATCMD = "AT+INVDATA=8,0103005A0001A419";
    SerialPort.println(ATCMD);
    UdpReadATState = 14;    
  break;
    case 14:
    delay(50);
    ATCMD = "AT+CIPSEND=30";
    SerialPort.println(ATCMD);
    UdpReadATState = 15;    
  break;
  case 15:
    delay(50);//Current Power
    ATCMD = "AT+INVDATA=8,010300560001641A"; 
    SerialPort.println(ATCMD);
    UdpReadATState = 16;    
  break;
    case 16:
    delay(50);
    ATCMD = "AT+CIPSEND=30";
    SerialPort.println(ATCMD);
    UdpReadATState = 17;    
  break;  
    case 17:
    delay(50);//Total Yield Today
    ATCMD = "AT+INVDATA=8,0103003F0002F407";
    SerialPort.println(ATCMD);
    UdpReadATState = 18;    
  break;
//Yield Today
    case 18:
    delay(50);
    ATCMD = "AT+CIPSEND=30";
    SerialPort.println(ATCMD);
    UdpReadATState = 19;    
  break;  
    case 19:
    delay(50);//Yield Today
    ATCMD = "AT+INVDATA=8,0103003C00014406";
    SerialPort.println(ATCMD);
    UdpReadATState++;    //20
  break;
  case 20:
    delay(50);
    ATCMD = "AT+PING=\"192.168.178.39\""; //PING Deye
    SerialPort.println(ATCMD);
    UdpReadATState++; 
  break;
  default:
    delay(20);
    //Serial.print("idle\n");
  break; 
  }
  delay(100);
  Uartloop(); 
}

void wifisetup(void)
{
    delay(500);
    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    //connect to your local wi-fi network
    WiFi.begin(ssid, password);
    //check wi-fi is connected to wi-fi network
    Serial.begin(115200);
    while(WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }

      Serial.println("");
      Serial.println("WiFi connected..!\n");
      tft.setTextColor(TFT_GREEN);
      tft.drawString("WiFi connected..!",10,13);
      Serial.print("Got IP: ");
      Serial.println(WiFi.localIP());
      tft.setTextColor(TFT_WHITE);
      // tft.drawString("Got IP: ",25,30);
      tft.drawString(String("Got IP: "+WiFi.localIP()),25,30);

    delay(50);
    //Wifi-SEtup end
}

void WriteActivePower()
{
    UdpReadATState = 30; //out of read range 

    delay(200); 
    ATCMD = "AT+CIPSEND=19";
    SerialPort.println(ATCMD);
    Uartloop();

    delay(200);
    ATCMD = "WIFIKIT-214028-READ";
    SerialPort.println(ATCMD);
    Uartloop();

    delay(200);
    ATCMD = "AT+CIPSEND=3";
    SerialPort.println(ATCMD);
    Uartloop();

    delay(200);
    ATCMD = "+ok";
    SerialPort.println(ATCMD);
    Uartloop();
    delay(200);

   //don't do it if no significant energy harvesting is possible
    
    if( (statechange == 0) )
    { //Toggle to 100%
      statechange = 1;    
      if( (currentpower>80) && (currentAPR > 2) )
      {
        delay(50);
        ATCMD = "AT+CIPSEND=37";
        SerialPort.println(ATCMD);
      
        Uartloop();

        delay(1000);
        ATCMD = "AT+INVDATA=11,01100028000102000221b9"; // 2%
        //011000280001020008A1BE // 8%
        //01100028000102001961B2 // 25%
        //01100028000102003221AD // 50%
        //01100028000102003CA069 // 60%
        SerialPort.println(ATCMD);
        Uartloop();
        delay(1000);//Yield Today
      }
      UdpReadATState = 21; //End of register write
    }else if(statechange == 1)
    { //Toggle to 100%
      statechange = 0;
      if(currentpower>25 && currentAPR < 10)
      {
        delay(50);
        ATCMD = "AT+CIPSEND=37";
        SerialPort.println(ATCMD);
      
        Uartloop();

        delay(1000);
        ATCMD = "AT+INVDATA=11,011000280001020064A193"; //100%
        //011000280001020050A044 // 80%
        //011000280001020070A19C //112%
        SerialPort.println(ATCMD);
        Uartloop();
        delay(1000);
      }
      UdpReadATState = 21; //End of register write
    }
}

void CheckButtonStates()
{
  ButtonState0 = digitalRead(BOOTPIN);
  ButtonState12 = digitalRead(BUTTON_D12);

  if(ButtonState12 == HIGH)
  {
    DisplayTimer = 0;
    // UdpReadATState = 20;
    // WriteActivePower();
  }else
  {
    //  digitalWrite(27, HIGH);
  }

  if(ButtonState0 == LOW)
  {
    printf("Restart Register Scan");
    printf("\n");
    testcounter++;
    printf("Nr: %d ",testcounter);
    UdpReadATState = 0; //rescan all
    DisplayTimer = 0;
  }else
  {
    //nothing
  }
  if(UdpReadATState < 21)
  {
     tft.setTextColor(TFT_RED);
     tft.fillRect(154, 10, 55, 20, TFT_BLACK); //erase Updating..
     tft.fillCircle(227, 15, 6, TFT_WHITE); //orange circle
     tft.fillCircle(227, 15, 5, TFT_BLACK); //erase circle
     tft.drawString("Updating",155,10,2);
     if(testcounter%2 == 0)
     {
      tft.fillCircle(227, 15, 3, TFT_RED); //orange circle
     }else
     {
      tft.fillCircle(227, 15, 5, TFT_BLACK); //erase circle
     }
     tft.setTextColor(TFT_WHITE);
     testcounter++;
  }else if(UdpReadATState == 21)
  {
    tft.setTextColor(TFT_GREEN);
    tft.fillCircle(227, 15, 3, TFT_GREEN); //green circle
    tft.fillRect(154, 10, 60, 20, TFT_BLACK); //erase Updating..
    tft.drawString("Updated",155,10,2);
    tft.setTextColor(TFT_WHITE);
    tft.drawFastVLine(145,32,113,TFT_RED); // A black vertical line starting from (x, y)(240,320)
    UdpReadATState++;
  }else if(UdpReadATState == 30)
  {
     tft.setTextColor(TFT_ORANGE);
     tft.fillRect(154, 10, 55, 20, TFT_BLACK); //erase Updating..
     tft.fillCircle(227, 15, 6, TFT_WHITE); //orange circle
     tft.fillCircle(227, 15, 5, TFT_BLACK); //erase circle
     tft.drawString("Register!",155,10,2);
     if(testcounter%2 == 0)
     {
      tft.fillCircle(227, 15, 3, TFT_ORANGE); //orange circle
     }else
     {
      tft.fillCircle(227, 15, 5, TFT_BLACK); //erase circle
     }
     tft.setTextColor(TFT_WHITE);
     testcounter++;     
  }
}

void TimerSetup()
{
  pinMode(DISPLAY_LED_PIN, OUTPUT);
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, ONE_SEC_IRQ, true);
  timerAlarmEnable(My_timer); //Just Enable
}

void CheckDisplayTimer()
{
  // printf("CheckDispTimer");
  // printf("\n");
  // printf("DisplayTimer: %d \n",DisplayTimer);
  // printf("GlobalTimer: %d \n",GlobalTimer);
  if(DisplayTimer >= 60 )
  {
    digitalWrite(DISPLAY_LED_PIN, false);
  }else
  {
    digitalWrite(DISPLAY_LED_PIN, true);
  }
  if(GlobalTimer%600 == 0)
  {
    digitalWrite(DISPLAY_LED_PIN, true);
    DisplayTimer = 0; 
    UdpReadATState = 6; //rescan   
  }
  if(GlobalTimer%900 == 0) //15 minutes
  {
    //Toggle APR Value between 100%-16%
    // WriteActivePower();
  }
}

 
void coreTask1( void * pvParameters )
{
 
    // String taskMessage = "Task 1: Checking Button States, core: ";
    // taskMessage = taskMessage + xPortGetCoreID();
 
    while(true){
      //  Serial.println(taskMessage);
      CheckButtonStates();
      vTaskDelay(200);
    }
    
}

void coreTask2( void * pvParameters )
{
 
    // String taskMessage = "Task 2: Check Timeout, core: ";
    // taskMessage = taskMessage + xPortGetCoreID();
 
    while(true){
        // Serial.println(taskMessage);
        CheckDisplayTimer();
        vTaskDelay(1000);
    }
}


void TaskSetup()
{
  xTaskCreatePinnedToCore(
                    coreTask1,   /* Function to implement the task */
                    "coreTask1", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    1);  /* Core where the task should run */
 
  Serial.println("Task created...");

vTaskDelay(100);

  xTaskCreatePinnedToCore(
                    coreTask2,   /* Function to implement the task */
                    "coreTask2", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    1);  /* Core where the task should run */

}

// void PZEMloop() {
         
//     Serial.print("Custom Address:");
//     Serial.println(pzem.readAddress(), HEX);

//     // Read the data from the sensor
//     float voltage = pzem.voltage();
//     float current = pzem.current();
//     float power = pzem.power();
//     float energy = pzem.energy();
//     float frequency = pzem.frequency();
//     float pf = pzem.pf();

//     // Check if the data is valid
//     if(isnan(voltage)){
//         Serial.println("Error reading voltage");
//     } else if (isnan(current)) {
//         Serial.println("Error reading current");
//     } else if (isnan(power)) {
//         Serial.println("Error reading power");
//     } else if (isnan(energy)) {
//         Serial.println("Error reading energy");
//     } else if (isnan(frequency)) {
//         Serial.println("Error reading frequency");
//     } else if (isnan(pf)) {
//         Serial.println("Error reading power factor");
//     } else {

//         // Print the values to the Serial console
//         Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
//         Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
//         Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
//         Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
//         Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
//         Serial.print("PF: ");           Serial.println(pf);
//     }

//     Serial.println();
//     delay(2000);
// }

void SetupSplash()
{
  tft.init();
  tft.fillScreen(TFT_BLACK);

  // Draw the icons, Horizontal - 2 bytes per pixel 565: 0xabcd
  tft.pushImage(65, 200, 102, 101, ESP32Logo);
  delay(500);
}

void SetupTable()
{
    // tft.init();
    tft.setRotation(0);
    // tft.fillScreen(TFT_BLACK);
      //Box start
    tft.drawFastHLine(0,30,240,TFT_RED); //A black horizontal line starting from (0, 120)
    tft.drawFastHLine(0,145,240,TFT_RED); //A black horizontal line starting from (0, 120)
    tft.drawFastVLine(238,1,145,TFT_RED); // A black vertical line starting from (x, y)
    tft.drawFastVLine(1,1,145,TFT_RED); // A black vertical line starting from (x, y)
    //Box end
    // Table for init values ---Start---
     tft.drawFastHLine(0,2,240,TFT_RED); //A black horizontal line starting from (x, y)
     tft.drawFastVLine(145,2,143,TFT_RED); // A black vertical line starting from (x, y)(240,320,length)
     tft.drawFastVLine(215,2,30,TFT_RED); // A black vertical line starting from (x, y)
    // Table for init values ---End----

// initial values and Text ---Start-------
      tft.setTextColor(TFT_BLUE);
      tft.drawString("A. Power Regulation",10,35,2);
      tft.setTextColor(TFT_WHITE);
      tft.drawFastHLine(2,56,240,TFT_RED); //A black horizontal line starting from (x, y)
      tft.drawString("Radiator Temp ",10,60,2);
      tft.drawString("Current Power ",10,80,2);
      tft.drawString("Yield Today  ",10,97,2);
      tft.drawFastHLine(2,119,240,TFT_RED); //A black horizontal line starting from (x, y)
      tft.drawString("Yield Total  ",10,125,2);
      tft.drawString("SW_Ver_",165,310,1);
      tft.drawString(String(SW_Version),210,310,1);

      init_b = 1; //flag for table generation
}

void setup() {
  SetupSplash();
  // pinMode(27, OUTPUT); //in timersetup
  Serial.begin(115200);
  // tft.init();
  // tft.setRotation(0);
  // tft.fillScreen(TFT_BLACK);
  // SetupTable();
// initial values and Text ---End-------

  //draw lines
  // wifisetup();
  // tft.drawFastHLine(0,lastline-10,320,TFT_LIGHTGREY); //A black horizontal line starting from (0, 120)
  // tft.drawFastVLine(180,0,lastline-10,TFT_LIGHTGREY); // A black vertical line starting from (160, 0)
  //UART Setup
  TimerSetup();
  //PZEMsetup();
  UARTSetup();
  TaskSetup();
}

void loop() 
{

  if(init_b == 0)
  {
    delay(2000);
    SetupTable();
  }else
  {

  }

 UartProcess();

//CheckButtonStates(); //no IRQ so do it often

 UartloopTest() ;

//  CheckButtonStates(); //no IRQ so do it often

//  CheckDisplayTimer();

      tft.setCursor(5,152);
      // tft.setTextSize(1);
      tft.fillRect(60, 152, 120, 10, TFT_BLACK);
      DispTimeout = 60 - DisplayTimer;
      tft.print(String("Display Timeout in ") + String(DispTimeout,DEC) + String("s") );

      if( (GlobalTimer%60)==0  )
      {
          if(minutes>=60)
          {
            hours++;
            minutes = 0;
          }
          // Clock---Start
        tft.drawString(String("Runtime : "),5,310,1);
        //hours
        tft.fillRect(60, 310, 80, 20, TFT_BLACK);
        tft.drawString(String(hours),62,310,1); 
        //:
        tft.drawString(String(":"),85,310,1); 
        //minute
        tft.drawString(String(minutes),95,310,1);    
      }
   
      // Serial.println(DisplayTimer);
      // Serial.println("\n");
      // Serial.println(GlobalTimer);
      // Serial.println("\n");
      //PZEMloop();

}