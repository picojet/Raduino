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
  #include "TimerThree.h"

//-----------------------------------------------------------------------------------------------------------------------------------------------
//board led's
// the RX_LED has a defined Arduino pin
// the TX_LED was not so lucky, we'll need to use pre-defined macros (TXLED1, TXLED0) to control that.
// TX_LED is not tied to a normally controlled pin so a macro is needed to control the LED
// (We could use the same macros for the RX LED too -- RXLED1, and RXLED0.)
//
	char   			RXLED 					= 17;                                                                       

//-----------------------------------------------------------------------------------------------------------------------------------------------
//timer3 control
//
	char   			timer3_outputPin   		= 5;
	unsigned long	timer3_defaultPeriod  	= 20000;
	unsigned int   	timer3_defaultDuty    	= 512;

//-----------------------------------------------------------------------------------------------------------------------------------------------
//serial input buffers/strings
//
	String 			ser_1_rxString 			= "";
	String 			ser_0_rxString 			= "";
  
//----------------------------------------------------------------------------------------------------------------------
//command processing/parsing
//
    int             cmndLength;
    char            cmndMajorChar;
    char            cmndMinorChar;
    unsigned long   cmndParamLong;
    float           cmndParamFloat;
  
//----------------------------------------------------------------------------------------------------------------------
//trigger control
//
	char           	trigManualMode			= 0;
	unsigned long  	trigManualPeriod  		= 20000;
	unsigned int    trigManualDuty    		= 512;

//----------------------------------------------------------------------------------------------------------------------
//show control
//
	boolean       	showRadarSpeed  		= 0;
	boolean       	showTrigPeriod  		= 0;

//----------------------------------------------------------------------------------------------------------------------
//radar speed processing
//
	float         	radarSpeed      		= 0.0;
	float         	radarMinSpeed   		= 1.0;
	float         	radarMaxSpeed   		= 4.0;
	float         	radarCoeff     			= 5000.0;
	unsigned long 	radarPeriod      		= 0;
	unsigned int    radarDuty        		= 512;

//-----------------------------------------------------------------------------------------------------------------------------------------------
//miscellaneous
//
    bool  			usbSerialOK 			= false;
	unsigned long 	loop_count 				= 0;                                                                 //

//-----------------------------------------------------------------------------------------------------------------------------------------------
//  v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v   v
//      v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v       v
//-----------------------------------------------------------------------------------------------------------------------------------------------
//
  void setup()
    {
    pinMode(RXLED, OUTPUT);                                                     //set RX_LED as output, TX_LED set as output behind the scenes?

    digitalWrite(RXLED, 1);                                                     //

    pinMode(timer3_outputPin, OUTPUT);											//
  
    Timer3.initialize(timer3_defaultPeriod);                            		//initialize timer3, and set default period
    Timer3.pwm(timer3_outputPin, timer3_defaultDuty);                   		//setup timer3.pwm output on pin, and set default duty cycle
    
    Serial.begin(9600);                                                         //this pipes to the serial monitor
    Serial1.begin(9600);                                                        //this is the UART, pipes to sensors attached to board
   
    Serial1.println("Hello Radar Sensor Connection");                           //print banner to the Radar (Serial1) ??

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
        Serial.println("0:Hello Host Control Connection");                              //print "Hello World" to the local Serial Monitor
        Serial1.println("Host connection established!!");                               //debug
        }
      }
    
    check_serial_0_rx();
    check_serial_1_rx();

    loop_count 	= loop_count + 1;                                                        //incremnt loop counter
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
    String          cmndParamStr = "";

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
      Serial.println("0:error command too short !");                  //if not then too short!
      return;
      }

//    Serial.print("0:charAt(1) = ");
//    Serial.print(ser_0_rxString.charAt(1));
//    Serial.println();

    if (ser_0_rxString.charAt(1) != ':')                              //is charAt(1) == ':'
      {
      Serial.println("0:error ':' in wrong place? !");                //if not error
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
          Serial.println("0:error bad prefix id? !");
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
        Serial.print("1:");                                                                 //  send prefix char(s)
        Serial.println(ser_1_rxString);                                                           //  send received string
        }

      if (showRadarSpeed)
        {
        Serial.print("0:After conversion to float:");
        Serial.println(radarSpeed);
        }

      if (radarSpeed < radarMinSpeed)
        {
        radarSpeed = radarMinSpeed;  
        if (showRadarSpeed)
          {
          Serial.print("0:warning (radarSpeed < radarMinSpeed) radarSpeed set to = ");
          Serial.println(radarSpeed);
          }
        }
      else if (radarSpeed > radarMaxSpeed)
        {
        radarSpeed = radarMaxSpeed;  
        if (showRadarSpeed)
          {
          Serial.print("0:warning (radarSpeed > radarMaxSpeed) radarSpeed set to = ");
          Serial.println(radarSpeed);
          }
        }

      radarPeriod = radarCoeff / radarSpeed;
      if (showTrigPeriod)
        {
        Serial.print("0:radarCoeff = ");
        Serial.print(radarCoeff);
        Serial.print(", radarPeriod = ");
        Serial.println(radarPeriod);
        }

      if (!trigManualMode)    
        {
        Timer3.pwm(timer3_outputPin, radarDuty, radarPeriod);   
        }
      }
    else
      {
      Serial.print("1:");                                                                 //  send prefix char(s)
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
      
      default :
        Serial.println("0:invalid command !!");
        return;
      }
    }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//help command
//Serial.println(F("This string will be stored in flash memory"));
//
  void cmnd_help()
    {
//                    --------------------------------------------------------------------------------
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
      Serial.println(F("0:SR n   -show rx Radar-Speed message (0=off/1=on)"));

	serial0_banner_line();
      Serial.println(F("0:RR     -return Radar Speed settings"));
      Serial.println(F("0:RC n.n -Radar-to-Trigger-Coefficient"));
      Serial.println(F("0:RX n.n -radarMinSpeed (m/s)"));
      Serial.println(F("0:RY n.n -radarMaxSpeed (m/s)"));

	serial0_banner_line();
      Serial.println(F("0:ER     -load defaults from EPROM"));
      Serial.println(F("0:EW     -save current values into EPROM"));
      Serial.println(F("0:EF     -load factory defaults and save to EPROM"));

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
        trigManualMode  = cmndParamLong & 1;
        trigUpdate      = true;    
        break;

      case 'P' :
	    if ((cmndParamLong < 65535) && (cmndParamLong > 0))
		  {
          trigManualPeriod = cmndParamLong;              
          trigUpdate      = true;    
  	      }
        break;

      case 'D' :
	    if ((cmndParamLong < 1023) && (cmndParamLong >= 0))
		  {
          trigManualDuty = cmndParamLong;
		  }
        break;

      default :
        Serial.println("0:unknown timer command? !");             
        return;
      }
    
      if (trigUpdate)    
        {
        Timer3.pwm(timer3_outputPin, trigManualDuty, trigManualPeriod);    
        }

    Serial.print("0:Trigger Mode = ");             
    Serial.println(trigManualMode, DEC);
    Serial.print("0:Trigger Period = ");             
    Serial.print(trigManualPeriod, DEC);          
    Serial.print("uS = ");             
    Serial.print((1000000 / trigManualPeriod), DEC);          
    Serial.println("Hz");             
    Serial.print("0:Trigger Duty = ");             
    Serial.println(trigManualDuty, DEC);          

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

      case 'C' :
	    radarCoeff		 = cmndParamFloat;
        break;

      case 'X' :
	    radarMinSpeed	 = cmndParamFloat;
        break;

      case 'Y' :
	    radarMaxSpeed	 = cmndParamFloat;
        break;

      default :
        Serial.println("0:unknown radar command? !");             
        return;
      }
    
    Serial.print("0:radarCoeff = ");
    Serial.println(radarCoeff);
    Serial.print("0:radarMinSpeed = ");
    Serial.println(radarMinSpeed);
    Serial.print("0:radarMaxSpeed = ");
    Serial.println(radarMaxSpeed);
    }
    
//-----------------------------------------------------------------------------------------------------------------------------------------------
//show commands
//
  void cmnd_show()
    {
    switch (cmndMinorChar)
      {
      case 'S' :
        break;

      case 'D' :
	    showTrigPeriod	 = cmndParamLong & 1;
        break;

      case 'R' :
	    showRadarSpeed	 = cmndParamLong & 1;
        break;

      default :
        Serial.println("0:unknown radar command? !");             
        return;
      }
    
    Serial.print("0:showTrigPeriod = ");
    Serial.println(showTrigPeriod);
    Serial.print("0:showRadarSpeed = ");
    Serial.println(showRadarSpeed);
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
