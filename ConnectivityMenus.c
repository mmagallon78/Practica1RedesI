/*****************************************************************************
* Connectivity test demo Menus implementation.
* 
* Copyright (c) 2013, Freescale, Inc. All rights reserved.
*
* 
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
*****************************************************************************/

#include "ConnectivityMenus.h"

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
uint8_t * const cu8FreescaleLogo[]={  
  "\f\r\n",
  "\n\r\n\r\n\r      #\n",
  "\r     ###\n",
  "\r    ###  *\n",
  "\r     #  ***\n",
  "\r       ***  #\n",
  "\r        *  ###\n",
  "\r          ###\n",
  "\r        *  #\n",
  "\r       ***\n",
  "\r      ***  #\n",
  "\r    #  *  ###\n",
  "\r   ###   ###\n",
  "\r  ###  *  #         F R E E S C A L E\n",
  "\r   #  ***\n",
  "\r     ***            S E M I C O N D U C T O R\n",
  "\r   #  *\n",
  "\r  ###               2 0 1 3\n",
  "\r ###\n",
  "\r  #           Connectivity Test Demo\n\n",
  "\r\n -Press enter to start",
  NULL
};

uint8_t * const cu8MainMenu[]={  
  "\f\r  Connectivity Test Interface short cuts\n",
  "\r------------------------------------------\n",
  "\r -Press [t] for Tx operation\n",
  "\r -Press [r] for Rx operation\n",
  "\r -Press [q] for channel up\n",
  "\r -Press [w] for channel down\n",
  "\r -Press [a] for Power up\n",
  "\r -Press [s] for Power down\n",
  "\r -Press [z] for Crystal Trim up\n",
  "\r -Press [x] for Crystal Trim down\n",
  "\r -Press [n] to increase the Payload\n",
  "\r -Press [m] to decrease the Payload\n",
  "\r -Press [S] to send team number package\n",
  "\r -Press [R] to go directly to Continuous Reception\n",
  "\r These keys can be used all over the application to change\n",
  "\r the test parameters\n",
  "\r  ________________________________\n",
  "\r |                                |\n",
  "\r |   Select the Test to perform   |\n",
  "\r |________________________________|\n",
  "\r -Press [1] Continuous tests\n",
  "\r -Press [2] Packet Error Rate test\n",
  "\r -Press [3] Range test\n",
  "\r -Press [4] Radio registers edit\n",
  "\r -Press [5] Send Receive Packets\n\r\n",
  NULL
};

uint8_t * const cu8ShortCutsBar[]={ 
  "\f\r\n",
  "\r---------------------------------------------------\n",
  "\r   [t] Tx   [q] Ch+ [a] Pw+  [z] Trm+  [n] Pyld+\n",  
  "\r   [r] Rx   [w] Ch- [s] Pw-  [x] Trm-  [m] Pyld-\n",  
  "\r---------------------------------------------------\n\r",
  NULL
};
 
uint8_t * const cu8ContinuousTestMenu[]={ 
  "\r __________________________ \n",
  "\r|                          |\n",
  "\r|   Continuous Test Menu   |\n",
  "\r|__________________________|\n\r\n",
  "\r-Press [1] Idle\n",
  "\r-Press [2] Continuous PRBS Transmission\n",
  "\r-Press [3] Continuous Modulated Transmission\n",
  "\r-Press [4] Continuous Unmodulated Transmission\n",
  "\r-Press [5] Continuous Reception\n",
  "\r-Press [6] Continuous Energy Detect\n",
  "\r-Press [7] Continuous Scan\n",
  "\r-Press [p] Previous Menu\n\r\n",
  "\rNow Running: ",
  NULL
};

uint8_t * const cu8PerTxTestMenu[]={ 
  "\r  ____________________________ \n",
  "\r |                            |\n",
  "\r |      PER Tx Test Menu      |\n",
  "\r |____________________________|\n\r\n",
  "\r  Choose the amount of packets to send:\n",
  "\r [0] - 1     Packet     [1] - 25    Packets\n",
  "\r [2] - 100   Packets    [3] - 500   Packets\n",
  "\r [4] - 1000  Packets    [5] - 2000  Packets\n",
  "\r [6] - 5000  Packets    [7] - 10000 Packets\n",
  "\r [8] - 65535 Packets\n\r\n",
  "\r-Press [p] Previous Menu\n\r\n",
  NULL
};

uint8_t * const cu8PerRxTestMenu[]={ 
  "\r  ______________________ \n",
  "\r |                      |\n",
  "\r |   PER Rx Test Menu   |\n",
  "\r |______________________|\n\r\n",
  "\r -Press [space bar] to start/stop Receiving Packets\n",
  "\r -Press [p] Previous Menu\n\r\n",
  NULL
};

uint8_t * const cu8RangeTxTestMenu[]={ 
  "\r  ________________________ \n",
  "\r |                        |\n",
  "\r |   Range Tx Test Menu   |\n",
  "\r |________________________|\n\r\n",
  "\r -Press [space bar] to start/stop Transmiting Packets\n",
  "\r -Press [p] Previous Menu\n\r\n",
  NULL
};

uint8_t * const cu8RangeRxTestMenu[]={ 
  "\r  ________________________ \n",
  "\r |                        |\n",
  "\r |   Range Rx Test Menu   |\n",
  "\r |________________________|\n\r\n",
  "\r -Press [space bar] to start/stop Receiving Packets\n",
  "\r -Press [p] Previous Menu\n\r\n",
  NULL
};

#if defined(gMc1323xPlatform_d)
uint8_t * const cu8RadioRegistersEditMenu[]={ 
  "\r   ____________________________ \n",
  "\r  |                            |\n",
  "\r  | Radio Registers Edit Menu  |\n",
  "\r  |____________________________|\n\r\n",
  "\r  -Press [1] Override Direct Registers\n",
  "\r  -Press [2] Read Direct Registers\n",
  "\r  -Press [3] Override Indirect Registers\n",
  "\r  -Press [4] Read Indirect Registers\n",
  "\r  -Press [p] Previous Menu\n\r\n",
  NULL
};
#else
uint8_t * const cu8RadioRegistersEditMenu[]={ 
  "\r   ____________________________ \n",
  "\r  |                            |\n",
  "\r  | Radio Registers Edit Menu  |\n",
  "\r  |____________________________|\n\r\n",
  "\r  -Press [1] Override Radio SPI Registers\n",
  "\r  -Press [2] Read Radio SPI Registers\n",
  "\r  -Press [p] Previous Menu\n\r\n",
  NULL
};
#endif

uint8_t * const cu8SendReceivePacketsTx[]={ 
  "\r __________________________ \n",
  "\r|                          |\n",
  "\r|  Send Receive Packets Tx |\n",
  "\r|__________________________|\n\r\n",
  "\r-Press [1] Single Packet\n",
  "\r-Press [2] Size Variable Packet\n",
  "\r-Press [3] Messenger\n",
  "\r-Press [p] Previous Menu\n\r\n",
  "\rNow Running: ",
  NULL
};

uint8_t * const cu8SendReceivePacketsRx[]={ 
  "\r __________________________ \n",
  "\r|                          |\n",
  "\r|  Send Receive Packets Rx |\n",
  "\r|__________________________|\n\r\n",
  "\r-Press 'space' to start receiving packets\n",
  "\r-Press [p] Previous Menu\n\r\n",
  "\rNow Running: ",
  NULL
};

uint8_t * const cu8SelectTags[] ={
  " Channel select ",
  "  Power select  ",
  " Test Tx select ",
  " Test Rx select ",
  "Trim coarse tune",
  " Trim fine tune "
};

uint8_t * const cu8TxTestTags[] ={
  "     PER Tx     ",
  "    Range Tx    ",
  "   Cont. Idle   ",
  "  Cont. PRBS9   ",
  "Cont. Modulated ",
  "Cont. Unmodulatd"
};

uint8_t * const cu8RxTestTags[] ={
  "     PER Rx     ",
  "    Range Rx    ",
  "Cont. Reception ",
  "   Cont. Scan   ",
  "Cont.Energy Det."
};

uint8_t * const cu8ContinuousTestTags[] ={
  "Idle mode",
  NULL,
  "Continuous Tx Modulated",
  "Continuous Tx Unmodulated",
  "Continuous PRBS9"
};

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/

/* Place it in NON_BANKED memory */
#ifdef MEMORY_MODEL_BANKED
#pragma CODE_SEG __NEAR_SEG NON_BANKED
#else
#pragma CODE_SEG DEFAULT
#endif /* MEMORY_MODEL_BANKED */

void LedsTimerCallback(void);
   
#pragma CODE_SEG DEFAULT

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

LEDsStates_t LEDsState;

/************************************************************************************
*************************************************************************************
* Interface functions
*************************************************************************************
************************************************************************************/
/***********************************************************************************
*
* PrintMenu
*
************************************************************************************/
void PrintMenu(uint8_t * const pu8Menu[], uartPortNumber_t port)
{
  uint8_t u8Index = 0;
  while(pu8Menu[u8Index]){
    (void)Uart_BlockingStringTx(pu8Menu[u8Index], port);
    u8Index++;
  }
}

/***********************************************************************************
*
* InitLEDs
*
************************************************************************************/
void InitLEDs(void)
{
  tmrChannelConfig_t TimerConfig;   
  /*Timer configuration*/
  TimerConfig.tmrChannel = gLEDsTimmerChannel_c;
  TimerConfig.tmrChannOptMode= gTmrOutputCompare_c;
  TimerConfig.tmrPinConfig.tmrOutCompState = gTmrPinNotUsedForOutComp_c;
  TimerConfig.tmrCompareVal = gLEDsToggleTime_c;
  (void)Tmr_SetCallbackFunc(gLEDsTimmer_c, gTmrChannel0Event_c, LedsTimerCallback);
  (void)Tmr_SetChannelConfig(gLEDsTimmer_c, &TimerConfig);
  
  LEDsState = mLEDsStopped_c;
  
  (void)Tmr_Enable(gLEDsTimmer_c, gTmrClkDivBy128_c, gLEDsToggleTime_c);
}

/***********************************************************************************
*
* LEDsBlink
*
************************************************************************************/
void LEDsBlink(void)
{
   LEDsState = mLEDsBlinking_c;
}

/***********************************************************************************
*
* LEDsBlinkLED4
*
************************************************************************************/
void LEDsBlinkLED4(void)
{
   LEDsState = mLED4Blinking_c;
}

/***********************************************************************************
*
* LEDsRun
*
************************************************************************************/
void LEDsRun(void)
{
   LEDsState = mLEDsRunning_c;
}

/***********************************************************************************
*
* LEDsStop
*
************************************************************************************/
void LEDsStop(void)
{
   Led_OffAll();
   LEDsState = mLEDsStopped_c;
}

/***********************************************************************************
*
* LEDsFlashValue
*
************************************************************************************/
void LEDsFlashValue(uint8_t u8Value, uint8_t u8Time100ms)
{
  uint8_t u8Index;

  Led_OffAll();
  for(u8Index=0;u8Index<3;u8Index++) DelayMs(100);
  Led_PrintValue(u8Value);
  for(u8Index=0;u8Index<u8Time100ms;u8Index++) DelayMs(100);
  Led_OffAll();
  for(u8Index=0;u8Index<3;u8Index++) DelayMs(100);
}

/***********************************************************************************
*
* LEDs_PrintValue3LEDs
*
************************************************************************************/
void LEDs_PrintValue3LEDs(uint8_t LedHexValue)
{
  Led_Off(gLED1_c);
  Led_Off(gLED2_c);
  Led_Off(gLED3_c);
  
  if(0x01 & LedHexValue){
    (void)Led_On(gLED1_c);
  }
      
  if(0x02 & LedHexValue){
    (void)Led_On(gLED2_c);
  }
    
  if(0x04 & LedHexValue){
    (void)Led_On(gLED3_c);
  }
      
}

/***********************************************************************************
*
* LEDs_PrintEnergyLevelOnLEDs
*
************************************************************************************/
void LEDs_PrintEnergyLevelOnLEDs(uint8_t u8EnergyLevel)
{
    Led_OffAll();
	  if((u8EnergyLevel < mEnergyTrigger3_c) && (u8EnergyLevel >= mEnergyTrigger2_c)){ 
		  Led_On(gLED2_c); 
		  Led_On(gLED1_c);
	  }
	  if((u8EnergyLevel < mEnergyTrigger4_c) && (u8EnergyLevel >= mEnergyTrigger3_c)) {
		  Led_On(gLED3_c);
		  Led_On(gLED2_c);
		  Led_On(gLED1_c);
	  }
	  if(u8EnergyLevel >= mEnergyTrigger4_c){ 
		  Led_OnAll(); 
	  } 
	  else{
		  Led_On(gLED1_c); 
	  }
}


/************************************************************************************
*************************************************************************************
* private functions
*************************************************************************************
************************************************************************************/

/* Place it in NON_BANKED memory */
#ifdef MEMORY_MODEL_BANKED
#pragma CODE_SEG __NEAR_SEG NON_BANKED
#else
#pragma CODE_SEG DEFAULT
#endif /* MEMORY_MODEL_BANKED */


/************************************************************************************
* LedsTimerCallback
* 
*
*
************************************************************************************/
void LedsTimerCallback(void)
{
  static uint8_t su8Toggle; 
  uint8_t aux;
  
  su8Toggle++;

  if(mLEDsStopped_c == LEDsState){
  }else if(mLEDsBlinking_c == LEDsState){
    if(su8Toggle & 0x01){
      Led_OnAll();
    }else{
      Led_OffAll();
    } 
  }else if(mLEDsRunning_c == LEDsState){
    aux = su8Toggle & 0x03;
    if(0 == aux){Led_On(gLED1_c); Led_Off(gLED2_c);Led_Off(gLED3_c);Led_Off(gLED4_c);}
    if(1 == aux){Led_Off(gLED1_c);Led_On(gLED2_c); Led_Off(gLED3_c);Led_Off(gLED4_c);}
    if(2 == aux){Led_Off(gLED1_c);Led_Off(gLED2_c);Led_On(gLED3_c); Led_Off(gLED4_c);}
    if(3 == aux){Led_Off(gLED1_c);Led_Off(gLED2_c);Led_Off(gLED3_c);Led_On(gLED4_c);}  
  }else if(mLED4Blinking_c == LEDsState){
    Led_Toggle(gLED4_c);
  }
}
   
#pragma CODE_SEG DEFAULT
