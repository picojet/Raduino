//-----------------------------------------------------------------------------------------------------------------------------------------------
//  v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v
//      v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v
//-----------------------------------------------------------------------------------------------------------------------------------------------
//
// 				RadUino v0.1
//
//-----------------------------------------------------------------------------------------------------------------------------------------------
//  v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v
//      v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v
//-----------------------------------------------------------------------------------------------------------------------------------------------

// Radar speed (via serial interface) to 'Trigger' frequency
// Implemented on Arduino Leonardo (ProMicro) compatible board - see: https://www.sparkfun.com/products/12640
// Board has Atmel ATmega32U4 microcontroller
// Host/PC interface uses default 'Serial' interface (via on-chip USB interface)
// Radar interface uses 'Serail1' interface (via external RS232 tranceiver)
// Trigger signal generated using 'Timer3' (via external RS422/485 driver)

// Note: After power-up, the Arduino Leonardo executes its 'boot-loader', after 8-seconds it then jups into its 'application'

// default Host/PC 'Serial' configuration is 9600,n,8,1
// default Radar 'Serial1' configuration is 9600,n,8,1

// Control/Status can be sent to/from the Radar via the Host/PC interface
// Control/status can be sent to/from the Radar-to-Trigger algorithm via the Host/PC interface
// All the Radar variables can be set and queried via the Host/PC interface
// The Host/PC interface can be used to supports the following commands;
// any command prefixed "0:" is sent to the processing in the Arduino
//  - any responses/debug/status from the Arduion are prefixed with "0:" before display
// and command prefixed "1:" is sent to the Radar (Serial1)
//  - any responses from the Radar (Serial1) are prefixed with "1:" before display
 
// 1:xxxxxxxx   -command argument 'xxxxxxxx' sent to Radar 'Serial1' interface
//              -all replies from the Radar 'Serial1' interface are displayed
 
// 0:TT         -return all trigger settings
// 0:TM 0       -set radar trigger mode (uses received radarSpeed message)
// 0:TM 1       -set manual trigger mode (uses manualDuty and manualPeriod)
// 0:TC 0       -continuous trigger
// 0:TC 1       -discontinuous trigger (trigger stops when radarSpeed < radarMinSpeed)(or no radarSpeed message for 1s)?
// 0:TP nnnn    -set manualPeriod to nnnn uS (valid values 1 - 65535)
// 0:TD nnnn    -set manualDuty to nnnn (range 0 - 1023)

// 0:SS         -return all show setings
// 0:SD n       -show Radar-to-Trigger debugging (0 = off / 1 = on)
// 0:SR n       -show received Radar-Speed message (0 = off / 1 = on)

// 0:RR         -return all Radar Speed settings
// 0:RC nn.nn   -set Radar-to-Trigger-Coefficient
// 0:RX nn.nn   -set radarMinSpeed (m/s)
// 0:RY nn.nn   -set radarMaxSpeed (m/s)

// 0:ER         -load defaults from EPROM
// 0:EW         -save current values into EPROM
// 0:EF         -load factory defaults and save to EPROM

//-----------------------------------------------------------------------------------------------------------------------------------------------
//include libraries
//
  #include <EEPROM.h>
  #include "TimerThree.h"

//-----------------------------------------------------------------------------------------------------------------------------------------------
//board led's
// the RX_LED has a defined Arduino pin
// the TX_LED was not so lucky, we'll need to use pre-defined macros (TXLED1, TXLED0) to control that.
// TX_LED is not tied to a normally controlled pin so a macro is needed to control the LED
// (We could use the same macros for the RX LED too -- RXLED1, and RXLED0.)
//
	char   			    RXLED 					      = 17;                                                                       

//-----------------------------------------------------------------------------------------------------------------------------------------------
//timer3 control
//
	char   			    timer3_outputPin   		= 5;
//	unsigned long	  timer3_defaultPeriod  = 20000;
//	unsigned int   	timer3_defaultDuty    = 512;

//-----------------------------------------------------------------------------------------------------------------------------------------------
//serial input buffers/strings
//
	String 			    ser_1_rxString 			= "";
	String 			    ser_0_rxString 			= "";
  
//----------------------------------------------------------------------------------------------------------------------
//command processing/parsing
//
  int             cmndLength;
  char            cmndMajorChar;
  char            cmndMinorChar;
  String          cmndParamStr;
  unsigned long   cmndParamLong;
  float           cmndParamFloat;
  
//----------------------------------------------------------------------------------------------------------------------
//trigger control
//
//	char           	trigManualMode			= 0;
//	unsigned long  	trigManualPeriod  	= 20000;
//	unsigned int    trigManualDuty    	= 512;

//----------------------------------------------------------------------------------------------------------------------
//show control
//
	boolean       	showRadarSpeed  		= 0;
	boolean       	showTrigPeriod  		= 0;

//----------------------------------------------------------------------------------------------------------------------
//radar speed processing
//
	float         	radarSpeed      		= 0.0;
//	float         	radarMinSpeed   		= 1.0;
//	float         	radarMaxSpeed   		= 4.0;
//	float         	radarCoeff     			= 5000.0;
	unsigned long 	radarPeriod      		= 0;
//	unsigned int    radarDuty        		= 512;

//----------------------------------------------------------------------------------------------------------------------------------------------
//miscellaneous
//
  bool  			    usbSerialOK 			= false;
	unsigned long 	loop_count 				= 0;                                                                 //

//-----------------------------------------------------------------------------------------------------------------------------------------------
//
	struct radarObject 
	  {
    int    thisObjectSize;                                             //
    unsigned int    thisObjectVersion;                                          //
    unsigned int    thisObjectCheckSum;                                         //
    unsigned int    thisDebugMask;                                              //
    byte            trigManualMode;                                             //0=manual, 1=automatic
	  unsigned long 	trigManualPeriod;                                           //manual trigger period
	  unsigned long 	trigManualDuty;                                             //manual trigger duty
    byte          	radarSpeedMode;                                             //0=continuous, 1=
	  float				    radarMinSpeed;                                              //minimum allowable vehicle speed
	  float				    radarMaxSpeed;                                              //maximum allowable vehicle speed
	  float				    radarCoeff;                                                 //radar speed coefficient...
    unsigned long   radarTrigDuty;                                              //automatic trigger duty
	  char            settingName[80];                                            //setting name
	  };

//defaults
//
  radarObject		defaultSettings = 
	  {
    sizeof(radarObject),        //thisObjectSize;
    1,                          //thisObjectVersion;
    0xFFFF,                     //thisObjectCheckSum;
    0x0000,                     //thisDebugMask;
	  0,						              //trigManualMode;
	  20000,					            //trigManualPeriod;
    512,                        //trigManualDuty;
    0,                          //radarSpeedMode;
	  1.0,						            //radarMinSpeed;
	  4.0,						            //radarMaxSpeed;
	  5000.0,					            //radarCoeff;
    512,                        //radarTrigDuty;
	  "Factory Defaults"          //settingName[80];
	  };

//working values
//
  radarObject		  workingValues;
  byte            workingSource;

//-----------------------------------------------------------------------------------------------------------------------------------------------
//  v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v
//      v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v
//-----------------------------------------------------------------------------------------------------------------------------------------------
//
  void setup()
    {
    int workingObjectSize;
    pinMode(RXLED, OUTPUT);                                                     //set RX_LED as output, TX_LED set as output behind the scenes?
    digitalWrite(RXLED, 1);                                                     //

    workingValues = defaultSettings;

    if (EEPROM.read(0) ==  0xAA) 
      {
      EEPROM.get(2, workingValues);        
      workingSource   = 1;

      if (workingValues.thisObjectSize != sizeof(radarObject))
        {
        workingValues = defaultSettings;
        EEPROM.write(0, 0xAA);         
        EEPROM.put(2, workingValues);
        workingSource   = 3;
        }
      }
    else
      {
      EEPROM.write(0, 0xAA);         
      EEPROM.put(2, workingValues);
      workingSource   = 2;
      }

    pinMode(timer3_outputPin, OUTPUT);											                    //

    Timer3.initialize(workingValues.trigManualPeriod);                            		    //initialize timer3, and set default period
    Timer3.pwm(timer3_outputPin, workingValues.trigManualDuty);                   		    //setup timer3.pwm output on pin, and set default duty cycle
    
    Serial.begin(9600);                                                         //this pipes to the serial monitor
    Serial1.begin(9600);                                                        //this is the UART, pipes to sensors attached to board
   
    Serial1.println("RadUino Radar Sensor Connection");                           //print banner to the Radar (Serial1) ??

    digitalWrite(RXLED, 0);                                                     //
    }
           
//-----------------------------------------------------------------------------------------------------------------------------------------------
//  v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v
//      v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v
//
  void loop()
    {
    if (usbSerialOK == false) 
      {
      if (Serial) 
        {
        usbSerialOK = true;
        Serial.println(F("0:RadUino Host Control Connection"));                              //print "Hello World" to the local Serial Monitor
        Serial1.println(F("Host connection established!!"));                               //debug

        switch (workingSource)
          {
          case 1 :
            Serial.println(F("0:workingValues restored from EEPROM!"));                    //
            break;
          case 2 :
            Serial.println(F("0:workingValues initialised to EEPROM!"));                   //
            break;
          case 3 :
            Serial.println(F("0:workingValues initialised to EEPROM! (new size)"));                   //
            break;
          }
          Serial.print(F("0:settingName : "));                                             //
          Serial.println(workingValues.settingName);                                    //
  //        serial0_workingValues();

          Serial.println(F("0:for help enter --> 0:HH"));                                             //
        }
      }
    
    check_serial_0_rx();
    check_serial_1_rx();

    loop_count 	= loop_count + 1;                                                        //increment loop counter
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//handler procedures for Serial.print/println access
//  void Serial0_print() {}
//  void Serial0_println() {}
//  void Serial1_print() {}
//  void Serial1_println() {}

//-----------------------------------------------------------------------------------------------------------------------------------------------
  void check_serial_0_rx()
    {
    if (Serial.available())                                                            //check to see if at least one character is available
      {
      int inChar = Serial.read();                                                         // if so read char
      if (inChar != '\n')
        {
        if (inChar != '\r')
          {
          ser_0_rxString += (char)inChar; 
          }
        }
      else
        {
  //      Serial.println(ser_0_rxString);
  //      ser_0_rxString.toUpperCase();
  //      Serial.println(ser_0_rxString);
        process_serial_0_rx();
        ser_0_rxString = ""; 
        }
      }
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
  void check_serial_1_rx()
    {
    if (Serial1.available())                                                            //check to see if at least one character is available
      {
      char inChar = Serial1.read();                                                         // if so read char
      if (inChar != '\n')
        {
        if (inChar != '\r')
          {
          ser_1_rxString += inChar; 
          }
        }
      else
        {
        process_serial_1_rx();
        ser_1_rxString = ""; 
        }
      }
    }

//----------------------------------------------------------------------------------------------------------------------
//3456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
//  v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v
//      v       v       v       v       v       v       v       v       v       v       v       v       v       v       v
//----------------------------------------------------------------------------------------------------------------------
//  position  = 0123456789        
//              0:H
//              0:TT
//              0:TM 1
//              0:TP 1234
//  length    = 123456789        
//----------------------------------------------------------------------------------------------------------------------
//
  void process_serial_0_rx()
    {
    ser_0_rxString.toUpperCase();                                     //convert to all upercase

    cmndLength      = ser_0_rxString.length();                        //get command string length
    cmndMajorChar       = '-';                                            //clear global variables
    cmndMinorChar       = '-';
    cmndParamLong   = 0;
    cmndParamFloat  = 0.0;

  //  Serial.print("0:command = ");
  //  Serial.println(ser_0_rxString);
  //  Serial.print("0:command length = ");
  //  Serial.println(cmndLength);
    
    if (cmndLength < 3)                                              //if >= 3 then we have 0:X....
      {
      Serial.println(F("0:error command too short !"));                  //if not then too short!
      return;
      }

//    Serial.print("0:charAt(1) = ");
//    Serial.print(ser_0_rxString.charAt(1));
//    Serial.println();

    if (ser_0_rxString.charAt(1) != ':')                              //is charAt(1) == ':'
      {
      Serial.println(F("0:error ':' in wrong place? !"));                //if not error
      return;
      }
    else
      {
      cmndMajorChar = ser_0_rxString.charAt(2);                           //ok so charAt(2) is majorCmnd

      if (cmndLength > 3)                                            //if length > 3
        {
        cmndMinorChar = ser_0_rxString.charAt(3);                         //then charAt(3) is minorCmnd
        }

  //    Serial.print("0:cmndMajorChar = ");
  //    Serial.print(cmndMajorChar);
  //    Serial.print(", cmndMinorChar = ");
  //    Serial.println(cmndMinorChar);

      if ((cmndLength > 5) && (ser_0_rxString.charAt(4) == ' '))     //if length > 5 and charAt(4) is space
        {
        cmndParamStr = ser_0_rxString.substring(5);                      //get command parameter..

  //      Serial.print("0:cmndParamStr = ");
  //      Serial.println(cmndParamStr);

        cmndParamLong   = cmndParamStr.toInt();                          //convert parameter to integer 
        cmndParamFloat  = cmndParamStr.toFloat();                        //convert parameter to float
        }
   
  //    Serial.print("0:cmndParamLong = ");
  //    Serial.print(cmndParamLong);
  //    Serial.print(", cmndParamFloat = ");
  //    Serial.println(cmndParamFloat);

      switch (ser_0_rxString.charAt(0))
        {
        case '0' :
          cmnd_0_handler();
          break;
      
        case '1' :
          Serial1.println(ser_0_rxString.substring(2));                                                           //  send received string
          break;
        
        default :
          Serial.println(F("0:error bad prefix id? !"));
          return;
        }
      }
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//
  void process_serial_1_rx()
    {
    String payload = "";
    int    strLength = ser_1_rxString.length();

    if (ser_1_rxString.charAt(0) == '?')
      {
      payload = ser_1_rxString.substring(2);
      radarSpeed = payload.toFloat();

      if (showRadarSpeed)
        {
        Serial.print(F("1:"));                                                                 //  send prefix char(s)
        Serial.println(ser_1_rxString);                                                           //  send received string
        }

      if (showRadarSpeed)
        {
        Serial.print(F("0:After conversion to float:"));
        Serial.println(radarSpeed);
        }

      if (radarSpeed < workingValues.radarMinSpeed)
        {
        radarSpeed = workingValues.radarMinSpeed;  
        if (showRadarSpeed)
          {
          Serial.print(F("0:warning (radarSpeed < radarMinSpeed) radarSpeed set to = "));
          Serial.println(radarSpeed);
          }
        }
      else if (radarSpeed > workingValues.radarMaxSpeed)
        {
        radarSpeed = workingValues.radarMaxSpeed;  
        if (showRadarSpeed)
          {
          Serial.print(F("0:warning (radarSpeed > radarMaxSpeed) radarSpeed set to = "));
          Serial.println(radarSpeed);
          }
        }

      radarPeriod = workingValues.radarCoeff / radarSpeed;
      if (showTrigPeriod)
        {
        Serial.print(F("0:radarCoeff = "));
        Serial.print(workingValues.radarCoeff);
        Serial.print(F(", radarPeriod = "));
        Serial.println(radarPeriod);
        }

      if (!workingValues.trigManualMode)    
        {
        Timer3.pwm(timer3_outputPin, workingValues.radarTrigDuty, radarPeriod);   
        }
      }
    else
      {
      Serial.print(F("1:"));                                                                 //  send prefix char(s)
      Serial.println(ser_1_rxString);                                                           //  send received string
      }
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//
  void cmnd_0_handler()
    {
  //  Serial.print("0:cmnd_0_handler() -> ");                                                           //  send received string

    switch (cmndMajorChar)
      {
      case 'H' :
          cmnd_help();
          break;

      case 'T' :
        cmnd_timer();                                                           //  send received string
        break;
      
      case 'R' :
        cmnd_radar();                                                           //  send received string
        break;
      
      case 'E' :
        cmnd_eeprom();                                                           //  send received string
        break;
      
      default :
        Serial.println(F("0:invalid command !!"));
        return;
      }
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//help command
//Serial.println(F("This string will be stored in flash memory"));
//
  void cmnd_help()
    {
//                      --------------------------------------------------------------------------------
	serial0_banner_line();
      Serial.println(F("0:Radar Interface HELP!!!"));                                                           //  send received string
      Serial.println(F("0:file: " __FILE__));
      Serial.println(F("0:compiled: " __DATE__ ", " __TIME__));

	serial0_banner_line();
      Serial.println(F("1:xxxx   -command argument 'xxxx' sent to Radar 'Serial1' interface"));
      Serial.println(F("1:yyyy   -all replies from the Radar 'Serial1' interface are displayed"));

	serial0_banner_line();
      Serial.println(F("0:TT     -show trigger settings"));
      Serial.println(F("0:TM 0   -trigger mode (uses received radarSpeed message)"));
      Serial.println(F("0:TM 1   -trigger mode (uses manualDuty and manualPeriod)"));
      Serial.println(F("0:TC 0   -continuous trigger"));
      Serial.println(F("0:TC 1   -stop trigger if (radarSpeed < radarMinSpeed) or no radarSpeed for >1s"));
      Serial.println(F("0:TP n   -manualPeriod uS (range 1-65535)"));
      Serial.println(F("0:TD n   -manualDuty (range 0-1023)"));

	serial0_banner_line();
      Serial.println(F("0:SS     -show debug/display setings"));
      Serial.println(F("0:SD n   -show Radar-to-Trigger math (0=off/1=on)"));
      Serial.println(F("0:SR n   -show received Radar-Speed messages (0=off/1=on)"));

	serial0_banner_line();
      Serial.println(F("0:RR     -return Radar Speed settings"));
      Serial.println(F("0:RC n.n -Radar-to-Trigger-Coefficient"));
      Serial.println(F("0:RX n.n -radarMinSpeed (m/s)"));
      Serial.println(F("0:RY n.n -radarMaxSpeed (m/s)"));

	serial0_banner_line();
      Serial.println(F("0:ES     -show current values"));
      Serial.println(F("0:ER     -load default values from EEPROM"));
      Serial.println(F("0:EW s   -save current values into EEPROM (s = optional name string)"));
      Serial.println(F("0:EF     -load factory default values and save to EEPROM"));

	serial0_banner_line();
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//
  void serial0_banner_line()
    {
    Serial.print("0:");
    for (int i=0; i < 78; i++)
      {     
      Serial.print("-");
      }
      Serial.println();
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//timer commands
//
  void cmnd_timer()
    {
    boolean trigUpdate = false;

    switch (cmndMinorChar)
      {
      case 'T' :
        break;

      case 'M' :
        workingValues.trigManualMode  = cmndParamLong & 1;
        trigUpdate      = true;    
        break;

      case 'P' :
	    if ((cmndParamLong < 65535) && (cmndParamLong > 0))
		  {
          workingValues.trigManualPeriod = cmndParamLong;              
          trigUpdate      = true;    
  	      }
        break;

      case 'D' :
	    if ((cmndParamLong < 1023) && (cmndParamLong >= 0))
		  {
          workingValues.trigManualDuty = cmndParamLong;
		  }
        break;

      default :
        Serial.println(F("0:unknown timer command? !"));             
        return;
      }
    
      if (trigUpdate)    
        {
        Timer3.pwm(timer3_outputPin, workingValues.trigManualDuty, workingValues.trigManualPeriod);    
        }

    Serial.print(F("0:Trigger Mode = "));             
    Serial.println(workingValues.trigManualMode, DEC);
    Serial.print(F("0:Trigger Period = "));             
    Serial.print(workingValues.trigManualPeriod, DEC);          
    Serial.print(F("uS = "));             
    Serial.print((1000000 / workingValues.trigManualPeriod), DEC);          
    Serial.println(F("Hz"));             
    Serial.print(F("0:Trigger Duty = "));             
    Serial.println(workingValues.trigManualDuty, DEC);          

    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//radar commands
//
  void cmnd_radar()
    {
    switch (cmndMinorChar)
      {
      case 'R' :
        break;

      case 'X' :
	    workingValues.radarMinSpeed	 = cmndParamFloat;
        break;

      case 'Y' :
	    workingValues.radarMaxSpeed	 = cmndParamFloat;
        break;

      case 'C' :
      workingValues.radarCoeff     = cmndParamFloat;
        break;

      default :
        Serial.println("0:unknown radar command? !");             
        return;
      }
    
    Serial.print(F("0:radarMinSpeed = "));
    Serial.println(workingValues.radarMinSpeed);
    Serial.print(F("0:radarMaxSpeed = "));
    Serial.println(workingValues.radarMaxSpeed);
    Serial.print(F("0:radarCoeff = "));
    Serial.println(workingValues.radarCoeff);
    }
    
//-----------------------------------------------------------------------------------------------------------------------------------------------
//show commands
//
  void cmnd_show()
    {
    switch (cmndMinorChar)
      {
      case 'D' :
      showTrigPeriod   = cmndParamLong & 1;
        break;

      case 'R' :
      showRadarSpeed   = cmndParamLong & 1;
        break;

      default :
        Serial.println(F("0:unknown radar command? !"));             
        return;
      }
    
    Serial.print(F("0:showTrigPeriod = "));
    Serial.println(showTrigPeriod);
    Serial.print(F("0:showRadarSpeed = "));
    Serial.println(showRadarSpeed);
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//eeprom commands
//
  void cmnd_eeprom()
    {
    switch (cmndMinorChar)
      {
      case 'S' :
        break;

      case 'R' :
        if (EEPROM.read(0) ==  0xAA) 
          {
          EEPROM.get(2, workingValues);        
          }
        else
          {
          EEPROM.write(0, 0xAA);         
          EEPROM.put(2, workingValues);
          }
        break;

      case 'W' :
        if (cmndParamStr.length() > 0)
          {
          cmndParamStr.toCharArray(workingValues.settingName, 80);
          }
        EEPROM.write(0, 0xAA);         
        EEPROM.put(2, workingValues);
        break;

      case 'F' :
        workingValues = defaultSettings;
        EEPROM.write(0, 0xAA);         
        EEPROM.put(2, workingValues);
        break;

      default :
        Serial.println(F("0:unknown EEPROM command? !"));             
        return;
      }

    serial0_workingValues();
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//
  void serial0_workingValues()
    {
    Serial.print(F("0:thisObjectSize      = "));
    Serial.println(workingValues.thisObjectSize);
    Serial.print(F("0:thisObjectVersion   = "));
    Serial.println(workingValues.thisObjectVersion);
    Serial.print(F("0:thisObjectCheckSum  = "));
    Serial.println(workingValues.thisObjectCheckSum, HEX);
    Serial.print(F("0:trigManualMode      = "));
    Serial.println(workingValues.trigManualMode);
    Serial.print(F("0:trigManualPeriod    = "));
    Serial.println(workingValues.trigManualPeriod);
    Serial.print(F("0:trigManualDuty      = "));
    Serial.println(workingValues.trigManualDuty);
    Serial.print(F("0:radarSpeedMode      = "));
    Serial.println(workingValues.radarSpeedMode);
    Serial.print(F("0:radarMinSpeed       = "));
    Serial.println(workingValues.radarMinSpeed);
    Serial.print(F("0:radarMaxSpeed       = "));
    Serial.println(workingValues.radarMaxSpeed);
    Serial.print(F("0:radarCoeff          = "));
    Serial.println(workingValues.radarCoeff);
    Serial.print(F("0:radarTrigDuty       = "));
    Serial.println(workingValues.radarTrigDuty);
    Serial.print(F("0:settingName[80]     = "));
    Serial.println(workingValues.settingName);
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//end of file
//-----------------------------------------------------------------------------------------------------------------------------------------------
//dump of useful bits of code...
//-----------------------------------------------------------------------------------------------------------------------------------------------
// The string in Flash
//Serial.print( F("Compiled: ");
//Serial.print( F(__DATE__));
//Serial.print( F(", "));
//Serial.print( F(__TIME__));
//Serial.print( F(", "));
//Serial.println( F(__VERSION__));
//-----------------------------------------------------------------------------------------------------------------------------------------------

//------------------------ TimerThree library help ------------------------
//Basic Usage
//  The timer is configured to repetitively measure a period of time, in microseconds. At the end of each period, an interrupt function can be run. The PWM pins can also be configured to switch during a portion of the period. 
//  If using TimerThree, replace "Timer1" with "Timer3". 
//Configuration
//  Timer1.initialize(microseconds); 
//    Begin using the timer. This function must be called first. "microseconds" is the period of time the timer takes. 
//  Timer1.setPeriod(microseconds); 
//    Set a new period after the library is already initialized. 
//Run Control
//  Timer1.start(); 
//    Start the timer, beginning a new period. 
//  Timer1.stop(); 
//    Stop the timer. 
//  Timer1.restart(); 
//    Restart the timer, from the beginning of a new period. 
//  Timer1.resume(); 
//    Resume running a stopped timer. A new period is not begun. 
//PWM Signal Output
//  Timer1.pwm(pin, duty); 
//    Configure one of the timer's PWM pins. "duty" is from 0 to 1023, where 0 makes the pin always LOW and 1023 makes the pin always HIGH. 
//  Timer1.setPwmDuty(pin, duty); 
//    Set a new PWM, without reconfiguring the pin. This is slightly faster than pwm(), but pwm() must be used at least once to configure the pin. 
//  Timer1.disablePwm(pin); 
//    Stop using PWM on a pin. The pin reverts to being controlled by digitalWrite(). 
//Interrupt Function
//  Timer1.attachInterrupt(function); 
//    Run a function each time the timer period finishes. The function is run as an interrupt, so special care is needed to share any variables beteen the interrupt function and your main program. 
//  Timer1.detachInterrupt(); 
//    Disable the interrupt, so the function no longer runs. 
