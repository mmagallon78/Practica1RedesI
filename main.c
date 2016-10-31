/*****************************************************************************
* Connectivity test demo main file.
* 
* Copyright (c) 2012, Freescale, Inc. All rights reserved.
*
* 
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
*  The Connectivity Test Demo  is  mainly  used  to perform radio performance 
*  test in hardware for wireless implementations. It allows users to exercise 
*  all the radio functionality and to get results for some important wireless 
*  tests as Packet Error Rate and Range test.
*  
*  The main functions provided by Connectivity test are:
*  - Perform Range test.
*  - Perform Packet Error Rate test.
*  - Manage radio parameters as Channel, Power and Crystal Trim.
*  - Perform Radio Tests as Continuous Modulated TX, Continuous PRBS9 packets 
*    Tx, Un-modulated TX, Continuous Rx and Channel energy Detect.
*  - Manage to read and write radio registers by address.
*
*  The Connectivity Test demo implements two options for user interface:
*  - Serial Menus attached to a host computer.
*  - Stand alone,  using the  boards built  in peripherals  as LCD, touchpad, 
*    LEDs, Buzzer and push buttons.
*
*****************************************************************************/

#include <hidef.h>                  /*EnableInterrupts macro*/
#include "McuInit.h"                /*CPU and System Clock related functions*/
#include "derivative.h"             /*System Clock related declarations*/
#include "EmbeddedTypes.h"          /*Include special data types*/             
#include "Utilities_Interface.h"    /*Includes Generic utilities*/
#include "PLM_config.h"             /*Defines Platform functionality*/
#include "SMAC_Interface.h"         /*Include all SMAC OTA functionality*/
#include "app_config.h"             /*Defines the Application default parameters*/
#include "ConnectivityMenus.h"      /*Defines the Application menus*/
#if gTargetBoard_c == gMc1323xRcm_c || gTargetBoard_c == gMc1323xRem_c  
 #include "PhyPacketProcessor.h"    /*Includes IAR registers functions*/
#else
 #include "Radio_Interface.h"       /*Includes SPI registers functions*/ 
#endif

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void InitProject(void);
void InitSmac(void);
static void DisplaySelectedState(ConnectivityStates_t state);
static void SetNewChannel(channels_t newChannel);
static void SetNewPower(uint8_t newPower);
static void SetNewTrim(uint8_t newTrim);
static void DisplayNewTxTest(txTests_t newTxTest);
static void DisplayNewRxTest(RxTests_t newRxTest);
static void PrintTestParameters(bool_t bEraseLine);
static void ClearEvents(void);
static void SerialUIStateMachine(void);
static void ManualUIStateMachine(void);
static bool_t SerialContinuousTxRxTest(void);
static bool_t ManualContinuousIdle(void);
static bool_t ManualContinuousPrbsTransmission(void);
static bool_t ManualContinuousModulatedTransmission(void);
static bool_t ManualContinuousUnmodulatedTransmission(void);
static bool_t ManualContinuousReception(void);
static bool_t ManualContinuousScan(void);
static bool_t ManualContinuousEnergyDetect(void);
static bool_t PacketErrorRateTx(void);
static bool_t PacketErrorRateRx(void);
static void SetRadioRxOnTimeOut15ms(void);
static void SetRadioRxOnNoTimeOut(void);
static void PrintPerRxFinalLine(uint16_t u16Received, uint16_t u16Total);
static bool_t RangeTx(void);
static bool_t RangeRx(void);
static bool_t EditRegisters(void);
#if defined(gMc1323xPlatform_d)
static bool_t OverrideDirectRegisters(void);
static bool_t OverrideIndirectRegisters(void);
static bool_t ReadDirectRegisters(void);
static bool_t ReadIndirectRegisters(void);
static bool_t SendReceivePacketsTxFunc(void);
#else
static bool_t OverrideSpiRadioRegisters(void);
static bool_t ReadSpiRadioRegisters(void);
#endif

/* Place it in NON_BANKED memory */
#ifdef MEMORY_MODEL_BANKED
#pragma CODE_SEG __NEAR_SEG NON_BANKED
#else
#pragma CODE_SEG DEFAULT
#endif /* MEMORY_MODEL_BANKED */
void MLMEScanConfirm(channels_t ClearestChann);
void MLMEResetIndication(void);
void MLMEWakeConfirm(void);
void UartRxCallback(uint8_t u8UartFlags);
void ShortCutsParser(uint8_t u8UartData);
void UartTxCallback(void);
#if (TRUE == gKeyboardSupported_d) || (TRUE == gTouchpadSupported_d) || (TRUE == gKbiSupported_d)
  static void KbiCallback(kbiPressed_t pressedKey);
#endif
#if (gTargetBoard_c == gMc1323xRcm_c) || (gTargetBoard_c == gMc1323xRem_c)
  static void KeyboardCallback (keyboardButton_t keyPressed);
#endif
#if gTargetBoard_c == gMc1323xRcm_c
  static void TouchpadCallback(touchpadEvent_t * event);
#endif
void LCDCallback(lcdErrors_t lcdError);
void MCPSDataConfirm(txStatus_t TransmissionResult);
void MCPSDataIndication(rxPacket_t *gsRxPacket);

/* Place your callbacks prototypes declarations here */

#pragma CODE_SEG DEFAULT


/************************************************************************************
*************************************************************************************
* Module Constants
*************************************************************************************
************************************************************************************/

#define mTotalFinalFrames_c 25

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

static uint8_t gau8RxDataBuffer[130]; 
static uint8_t gau8TxDataBuffer[128]; 
        
        
static txPacket_t * gAppTxPacket;
static rxPacket_t * gAppRxPacket;
            
bool_t firstUSignal;

bool_t bTxDone;
bool_t bRxDone;
bool_t bScanDone;
channels_t bestChannel;

bool_t  bUartTxDone;  
uint8_t gu8UartData;
uartConfigSet_t uartSettings;
kbiConfig_t kbiConfiguration; 

operationModes_t testOpMode;
operationModes_t prevOpMode;
channels_t       testChannel;
uint8_t          testPower;
uint8_t          testTrimmValue;
uint8_t          testPayloadLen;

ConnectivityTestUIs_t selectedUI;

smacTestMode_t contTestRunning;

bool_t evSwitch1;
bool_t evSwitch2;
bool_t evSwitch3;
bool_t evSwitch4;
bool_t evDataFromUART;
bool_t evTestParameters;

bool_t shortCutsEnabled;
ConnectivityStates_t       connState;
ContinuousTxRxTestStates_t cTxRxState;
PerTxStates_t              perTxState;
PerRxStates_t              perRxState;
RangeTxStates_t            rangeTxState;
RangeRxStates_t            rangeRxState;
EditRegsStates_t    eRState; 
ODRStates_t         oDRState;
OIRStates_t         oIRState;
RDRStates_t         rDRState;
RIRStates_t         rIRState;
RangeTxStates_t            rangeTxState;
RangeRxStates_t            rangeRxState;
SendReceivePacketsTx_t		SendReceivePacketsTx;
SendReceivePacketsRx_t		SendReceivePacketsRx;

overrideRegistersStates_t     oSRRState;
readRegistersStates_t         rSRRState;

uint8_t messengerOn = 0;


uint8_t au8ScanResults[16];

txTests_t txTestIndex;
bool_t (*txTestsList[])(void) = {
  PacketErrorRateTx,
  RangeTx,
  ManualContinuousIdle,
  ManualContinuousPrbsTransmission,
  ManualContinuousModulatedTransmission,
  ManualContinuousUnmodulatedTransmission,
};

RxTests_t rxTestIndex;

bool_t (*rxTestsList[])(void) = {
  PacketErrorRateRx,
  RangeRx,
  ManualContinuousReception,
  ManualContinuousScan,
  ManualContinuousEnergyDetect  
};

/************************************************************************************
*************************************************************************************
* Main application functions
*************************************************************************************
************************************************************************************/
void main(void) 
{
  MCUInit();  
  InitProject();
  EnableInterrupts; 

  /* SMAC Initialization */
  InitSmac();
  (void)MLMEXtalAdjust(testTrimmValue);

  /*Prints the Welcome screens in the UART and LCD*/  
  PrintMenu(cu8FreescaleLogo, gDefaultUartPort_c);
  (void)Lcd_WriteStringBlocking(1,"  Connectivity");
  (void)Lcd_WriteStringBlocking(2,"   Press  SW1");
  LEDsBlink();

  /*Waits until de user selects an UI*/  
  
  while(gNoUI_c == selectedUI){ 
    if(evDataFromUART && ('\r' == gu8UartData)){
       selectedUI = gSerialUI_c;
       connState = gConnIdleState_c;
       (void)Lcd_Clear();
    }
    if(evSwitch1){
       selectedUI = gManualUI_c;
       connState = gConnSelectTest_c;      
    }
  } 

  ClearEvents();
  LEDsStop();
        
  for(;;) 
  {
 
    /*Runs the selected UI*/
    if(gSerialUI_c == selectedUI){
      SerialUIStateMachine();
    }else{
      ManualUIStateMachine();
    }
    __RESET_WATCHDOG();
  } 
}

/************************************************************************************
* InitProject
* 
* Hardware and Global data initialization
*
************************************************************************************/
void InitProject(void)
{
  Gpio_Init();
  /* UART Initialization */
#if TRUE == gUartSupported_d   
  uartSettings.baudRate = gUartDefaultBaud_c;
  uartSettings.dataBits = g8DataBits_c;
  uartSettings.parity = gUartParityNone_c;
  (void)Uart_Init();
  (void)Uart_Configure(gDefaultUartPort_c, &uartSettings);
  (void)Uart_RegisterRxCallBack(UartRxCallback, gUartRxCbCodeNewByte_c, gDefaultUartPort_c);
  (void)Uart_RegisterTxCallBack(UartTxCallback, gDefaultUartPort_c);
#endif
  (void)Tmr_Init();
  
  /* KBI Initialization */
#if gTargetBoard_c == gMc1323xRcm_c || gTargetBoard_c == gMc1323xRem_c  
#if (gKeyboardSupported_d || gTouchpadSupported_d)    
  kbiConfiguration.Control.bit.TriggerByLevel = 0;
  kbiConfiguration.Control.bit.Interrupt = 1;
  kbiConfiguration.Control.bit.Acknowledge = 1;
#if(gTouchpadSupported_d)
  kbiConfiguration.InterruptPin.Port = gSwitchColmnMask_c|gTouchpadAttnPinMask_c;
#else 
  kbiConfiguration.InterruptPin.Port = gSwitchColmnMask_c; 
#endif
  kbiConfiguration.EdgeSelect.Port = gSwitchColmnMask_c;
  (void)Kbi_Init((kbiCallback_t)KbiCallback, &kbiConfiguration, gSwitchKbiModule_c);
  (void)Keyboard_InitKeyboard(KeyboardCallback);
#endif
#else
  kbiConfiguration.Control.bit.TriggerByLevel = 0;
  kbiConfiguration.Control.bit.Interrupt = 1;
  kbiConfiguration.Control.bit.Acknowledge = 1;
  kbiConfiguration.InterruptPin.Port = gSwitchMask;
  kbiConfiguration.EdgeSelect.Port = 0;
  (void)Kbi_Init((kbiCallback_t)KbiCallback, &kbiConfiguration, gSwitchKbiModule_c);
 
#endif

  /* SPI Initialization */ 
   SPI1_Init(gSpiBaudDivisor_2_c); 
  
  /* Touchpad Initialization */ 
  #if gTargetBoard_c == gMc1323xRcm_c
  (void)IIC_Init(mIic100KhzBaudInitParameters_c);
  (void)Touchpad_DriverInit(TouchpadCallback, gGpioPortB_c, gGpioPin6Mask_c);
  #endif

  InitLEDs();
  Buzzer_Init();    
  ClearEvents();
  
   /*LCD configuration*/
  (void)Lcd_Init(LCDCallback);
  (void)Lcd_Config(TRUE,TRUE,FALSE);
  
  /*Global Data init*/
  testOpMode     = gDefaultMode_c;
  testChannel    = gDefaultChannelNumber_c;
  testPower      = gDefaultOutputPower_c;
  testTrimmValue = gDefaultCrysTrim_c;
  testPayloadLen = gDefaultPayload_c;
  selectedUI     = gNoUI_c;
  contTestRunning = gTestModeForceIdle_c;
  shortCutsEnabled = FALSE; 
  connState      = gConnInitState_c;
  cTxRxState     = gCTxRxStateInit_c;
  perTxState     = gPerTxStateInit_c;
  perRxState     = gPerRxStateInit_c;
  rangeTxState   = gRangeTxStateInit_c;
  rangeRxState   = gRangeRxStateInit_c;
  SendReceivePacketsTx = gSendReceivePacketsTxInit_c;
  SendReceivePacketsRx = gSendReceivePacketsRxInit_c;
  prevOpMode      = gDefaultMode_c;
  txTestIndex    = gPacketErrorRateTx_c;
  rxTestIndex    = gPacketErrorRateRx_c;
  firstUSignal	 = 0;

}

/**************************************************************************************/
void InitSmac(void)
{
  gAppTxPacket = (txPacket_t*)gau8TxDataBuffer;
  gAppRxPacket = (rxPacket_t*)gau8RxDataBuffer; 
  gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
  
  (void)MLMERadioInit();
  (void)MLMESetClockRate(gClko16MHz_c);
  MCU_UseExternalClock();
  
  (void)MLMESetTmrPrescale(gTimeBase250kHz_c);
  while (gErrorNoError_c != MLMESetChannelRequest(gDefaultChannelNumber_c));
  (void)MLMEPAOutputAdjust(gDefaultOutputPower_c);
  (void)MLMEFEGainAdjust(gGainOffset_c);
}

/**************************************************************************************/
void SerialUIStateMachine(void)
{
	if(evSwitch1)
	{
		gAppTxPacket->u8DataLength = 27;
	  	MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "Equipo 9, Boton Presionado",27);
	  	(void)MCPSDataRequest(gAppTxPacket);
	  	Led_Toggle(gLED1_c);
	  	DelayMs(200);
	  	Led_Toggle(gLED1_c);
	  	ClearEvents();
	}
    
    
  if((gConnSelectTest_c == connState) && evTestParameters){
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  switch(connState){
    case gConnIdleState_c:
       PrintMenu(cu8MainMenu, gDefaultUartPort_c);
       PrintTestParameters(FALSE);
       shortCutsEnabled = TRUE;           
       connState = gConnSelectTest_c;
    break;
    case gConnSelectTest_c:
     if(evDataFromUART){
       if('1' == gu8UartData){
         cTxRxState = gCTxRxStateInit_c;
         connState = gConnContinuousTxRxState_c;
       }else if('2' == gu8UartData){
         perTxState = gPerTxStateInit_c;
         perRxState = gPerRxStateInit_c;
         connState = gConnPerState_c;
       }else if('3' == gu8UartData){
         rangeTxState = gRangeTxStateInit_c;
         rangeRxState = gRangeRxStateInit_c;
         connState = gConnRangeState_c;
       }else if('4' == gu8UartData){
         eRState = gERStateInit_c;
         connState = gConnRegEditState_c;
       }else if('5' == gu8UartData){
		SendReceivePacketsTx = gSendReceivePacketsTxInit_c;
		if(testOpMode == mTxOperation_c)
			connState = gConnSendReceivePacketsTxState_c;
		else
			connState = gConnSendReceivePacketsRxState_c;
		}
       ClearEvents();
     }
    break;
    case gConnContinuousTxRxState_c:
       if(SerialContinuousTxRxTest()) {
           connState = gConnIdleState_c;
       }
    break;
    case gConnPerState_c:
       if(mTxOperation_c == testOpMode){
         if(PacketErrorRateTx()){
           connState = gConnIdleState_c;
         }
       }else{
         if(PacketErrorRateRx()){
           connState = gConnIdleState_c;
         }
       }
    break;
    case gConnRangeState_c:
       if(mTxOperation_c == testOpMode){
         if(RangeTx()){
           connState = gConnIdleState_c;
         }
       }else{
         if(RangeRx()){
           connState = gConnIdleState_c;
         }
       }
    break;
    case gConnRegEditState_c:
       if(EditRegisters()) {
           connState = gConnIdleState_c;
       }
    break;
    //////////////////////////////////////
    case gConnSendReceivePacketsTxState_c:
       if(SendReceivePacketsTxFunc()) {
           connState = gConnIdleState_c;
       }
    break;
    //////////////////////////////////////
    default:
    break;
    
  }
  if(prevOpMode != testOpMode){
    perTxState = gPerTxStateInit_c;
    perRxState = gPerRxStateInit_c;
    rangeTxState = gRangeTxStateInit_c;
    rangeRxState = gRangeRxStateInit_c;
    prevOpMode = testOpMode;
  }
}


/**************************************************************************************/
void ManualUIStateMachine(void)
{

  if(evSwitch1){
    connState ++;
    if(connState > gConnSetTrimFineState_c){
      connState = gConnSetChannelState_c;
    }
    ClearEvents();
    DisplaySelectedState(connState);
  }
  LEDs_PrintValue3LEDs(connState - gConnSetChannelState_c + 1);
    
  switch(connState){
    case gConnSelectTest_c:
        connState = gConnSetChannelState_c;
        DisplaySelectedState(connState);
    break;
    case gConnSetChannelState_c:
         if(evSwitch2){
           testChannel++;
           if(gChannel26_c < testChannel){
             testChannel = gChannel11_c;
           }
           SetNewChannel(testChannel);
           ClearEvents();
         }
         if(evSwitch3){
           testChannel--;
           if(gChannel11_c > testChannel){
             testChannel = gChannel26_c;
           }
           SetNewChannel(testChannel);
           ClearEvents();
         }
    break;
    case gConnSetPowerState_c:
         if(evSwitch2){
           testPower++;
           if(gMaxOutputPower_c < testPower){
             testPower = 0;
           }
           SetNewPower(testPower);
           ClearEvents();
         }
         if(evSwitch3){
           testPower--;
           if(0xFF == testPower){
             testPower = gMaxOutputPower_c;
           }
           SetNewPower(testPower);
           ClearEvents();
         }
    break;
    case gConnSetTxTestState_c:
         if(evSwitch2){
           txTestIndex++;
           if(gMaxTxTest_c == txTestIndex) txTestIndex = 0; 
           DisplayNewTxTest(txTestIndex);
           #if(FALSE == gLcdSupported_d)
            LEDsFlashValue(txTestIndex+1,5);              
           #endif 
           ClearEvents();
         }
         if(evSwitch3){
           txTestIndex--;
           if(0xFF == txTestIndex) txTestIndex = gMaxTxTest_c-1;               
           DisplayNewTxTest(txTestIndex);
           #if(FALSE == gLcdSupported_d)
            LEDsFlashValue(txTestIndex+1,5);  
           #endif 
           ClearEvents();
         }
         if(evSwitch4){
           perTxState = gPerTxStateInit_c;
           rangeTxState = gRangeTxStateInit_c;
           ClearEvents();
           while(!(*txTestsList[txTestIndex])());
           DisplaySelectedState(connState);
           DisplayNewTxTest(txTestIndex);
         }
    break;
    case gConnSetRxTestState_c:
         if(evSwitch2){
           rxTestIndex++;
           if(gMaxRxTest_c == rxTestIndex) rxTestIndex = 0;               
           DisplayNewRxTest(rxTestIndex);
           #if(FALSE == gLcdSupported_d)
            LEDsFlashValue(rxTestIndex+1,5);  
           #endif 
           ClearEvents();
         }
         if(evSwitch3){
           rxTestIndex--;
           if(0xFF == rxTestIndex) rxTestIndex = gMaxRxTest_c-1;               
           DisplayNewRxTest(rxTestIndex);
           #if(FALSE == gLcdSupported_d)
            LEDsFlashValue(rxTestIndex+1,5);  
           #endif 
           ClearEvents();
         }
         if(evSwitch4){
           perRxState = gPerRxStateInit_c;
           rangeRxState = gRangeRxStateInit_c;
           ClearEvents();
           while(!(*rxTestsList[rxTestIndex])());
           DisplaySelectedState(connState);
           DisplayNewRxTest(rxTestIndex);
         }
   break;
    case gConnSetTrimCoarseState_c:
         if(evSwitch2){
           testTrimmValue+=0x10;
           SetNewTrim(testTrimmValue);
           ClearEvents();
         }
         if(evSwitch3){
           testTrimmValue-=0x10;
           SetNewTrim(testTrimmValue);
           ClearEvents();
         }
    break;
    case gConnSetTrimFineState_c:
         if(evSwitch2){
           testTrimmValue++;
           SetNewTrim(testTrimmValue);
           ClearEvents();
         }
         if(evSwitch3){
           testTrimmValue--;
           SetNewTrim(testTrimmValue);
           ClearEvents();
         }

    break;
    default:
    break;
  }
  
}

/**************************************************************************************/
bool_t SerialContinuousTxRxTest(void)
{
  bool_t bBackFlag = FALSE;
  smacErrors_t smacResult;
  uint8_t u8Index, u8TempEnergyValue;
  
  if(evTestParameters){
    (void)MLMERXDisableRequest();
    (void)MLMETestMode(gTestModeForceIdle_c);
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    if(gCTxRxStateSelectTest_c == cTxRxState){
      PrintTestParameters(TRUE);
    }else{
      PrintTestParameters(FALSE);
      (void)Uart_BlockingStringTx("\r\n", gDefaultUartPort_c);      
    }
    
    if(gTestModePRBS9_c == contTestRunning){
      (void)MLMETestMode(gTestModePRBS9_c);
    }else if(gTestModeContinuousTxModulated_c == contTestRunning){
      (void)MLMETestMode(gTestModeContinuousTxModulated_c);
    }else if(gTestModeContinuousTxUnmodulated_c == contTestRunning){
      (void)MLMETestMode(gTestModeContinuousTxUnmodulated_c);
    }
    
    if(gCTxRxStateRunnigRxTest_c == cTxRxState){
      bRxDone = FALSE;
      gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
      (void)MLMERXEnableRequest(gAppRxPacket, 0);
    }
    evTestParameters = FALSE;
  }
  
  switch(cTxRxState)
  {
    case gCTxRxStateIdle_c:
         if((evDataFromUART) && ('\r' == gu8UartData))
         {
           cTxRxState = gCTxRxStateInit_c;
           ClearEvents();  
         }
    break;
    case gCTxRxStateInit_c:
         PrintMenu(cu8ShortCutsBar, gDefaultUartPort_c);
         PrintMenu(cu8ContinuousTestMenu, gDefaultUartPort_c);
         (void)MLMETestMode(contTestRunning);
         if (gTestModeContinuousTxUnmodulated_c == contTestRunning && firstUSignal==0)
         {
        	 firstUSignal=1;
        	 (void)MLMETestMode(contTestRunning);
         }
         (void)Uart_BlockingStringTx(cu8ContinuousTestTags[contTestRunning], gDefaultUartPort_c);
         (void)Uart_BlockingStringTx("\r\n\r\n", gDefaultUartPort_c);
         PrintTestParameters(FALSE);
         shortCutsEnabled = TRUE;           
         cTxRxState = gCTxRxStateSelectTest_c; 
    break;
    case gCTxRxStateSelectTest_c:
         if(evDataFromUART){
           if('1' == gu8UartData){
             contTestRunning = gTestModeForceIdle_c;              
             cTxRxState = gCTxRxStateInit_c;
           }else if('2' == gu8UartData){
             contTestRunning = gTestModePRBS9_c;               
             cTxRxState = gCTxRxStateInit_c;
           }else if('3' == gu8UartData){
             contTestRunning = gTestModeContinuousTxModulated_c;               
             cTxRxState = gCTxRxStateInit_c;
           }else if('4' == gu8UartData){
             contTestRunning = gTestModeContinuousTxUnmodulated_c;               
             cTxRxState = gCTxRxStateInit_c;
           }else if('5' == gu8UartData){
             (void)MLMETestMode(gTestModeForceIdle_c);
             contTestRunning = gTestModeForceIdle_c;
             (void)Uart_BlockingStringTx("\f\r\nPress [p] to stop receiving promiscuous packets \r\n", gDefaultUartPort_c);
             MLMESetPromiscuousMode(TRUE);
             bRxDone = FALSE;
             gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
             (void)MLMERXEnableRequest(gAppRxPacket, 0);
             cTxRxState = gCTxRxStateRunnigRxTest_c;
           }else if('6' == gu8UartData){
             (void)MLMETestMode(gTestModeForceIdle_c);
             contTestRunning = gTestModeForceIdle_c;
             (void)Uart_BlockingStringTx("\f\r\nPress [p] to stop the Continuous ED test\r\n", gDefaultUartPort_c);
             contTestRunning = gTestModeForceIdle_c;                
             cTxRxState = gCTxRxStateRunnigEdTest_c;
           }else if('7' == gu8UartData){
             (void)MLMETestMode(gTestModeForceIdle_c);
             contTestRunning = gTestModeForceIdle_c;
             (void)Uart_BlockingStringTx("\f\r\nPress [p] to stop the Continuous SCAN test\r\n", gDefaultUartPort_c);
             bScanDone = FALSE;
             smacResult = MLMEScanRequest(0xFFFF, gScanModeED_c, au8ScanResults);
             contTestRunning = gTestModeForceIdle_c;                
             cTxRxState = gCTxRxStateRunnigScanTest_c;
           }else if('p' == gu8UartData){ 
             (void)MLMETestMode(gTestModeForceIdle_c);
             bBackFlag = TRUE;
           }
           ClearEvents();
         }
    break;
    case gCTxRxStateRunnigRxTest_c:
         if(bRxDone){
           if (gAppRxPacket->rxStatus == rxSuccessStatus_c){
             (void)Uart_BlockingStringTx("New Packet: ", gDefaultUartPort_c);
             for(u8Index = 0; u8Index < gAppRxPacket->u8DataLength ; u8Index++){
               PrintByteOnHexFormatBlocking(gAppRxPacket->smacPdu.u8Data[u8Index], TRUE, gDefaultUartPort_c);
             }
             (void)Uart_BlockingStringTx("\b \r\n", gDefaultUartPort_c);
           }
           bRxDone = FALSE;
           gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
           (void)MLMERXEnableRequest(gAppRxPacket, 0);
         }
         if((evDataFromUART) && ('p' == gu8UartData)){
           (void)MLMERXDisableRequest();
           MLMESetPromiscuousMode(FALSE);
           (void)Uart_BlockingStringTx("\r\n\r\n Press [enter] to go back to the Continuous test menu ", gDefaultUartPort_c);
           cTxRxState = gCTxRxStateIdle_c;
           ClearEvents();
         }
    break;
    case gCTxRxStateRunnigEdTest_c:
         DelayMs(100);DelayMs(100);
         (void)Uart_BlockingStringTx("Energy on the Channel ", gDefaultUartPort_c);
         PrintWordOnDecimalFormatBlocking((uint16_t)testChannel, 0, FALSE, gDefaultUartPort_c);
         (void)Uart_BlockingStringTx(" : ", gDefaultUartPort_c);
         (void)MLMEEnergyDetect(&u8TempEnergyValue);
         PrintWordOnDecimalFormatBlocking((uint16_t)(u8TempEnergyValue),0,TRUE, gDefaultUartPort_c);
         (void)Uart_BlockingStringTx("dBm\r\n", gDefaultUartPort_c);
         if((evDataFromUART) && ('p' == gu8UartData)){
           (void)Uart_BlockingStringTx("\r\n\r\n Press [enter] to go back to the Continuous test menu ", gDefaultUartPort_c);
           cTxRxState = gCTxRxStateIdle_c;
           ClearEvents();
         }
    break;
    case gCTxRxStateRunnigScanTest_c:
         if(bScanDone){
           (void)Uart_BlockingStringTx("Results : ", gDefaultUartPort_c);
           for(u8Index = 0; u8Index < 16 ; u8Index++){
             PrintWordOnDecimalFormatBlocking((uint16_t)(au8ScanResults[u8Index]),0,TRUE, gDefaultUartPort_c);
             (void)Uart_BlockingStringTx(",", gDefaultUartPort_c);   
           }
           (void)Uart_BlockingStringTx("\b \r\n", gDefaultUartPort_c);
           if((evDataFromUART) && ('p' == gu8UartData)){
             (void)Uart_BlockingStringTx("\r\n\r\n Press [enter] to go back to the Continuous test menu ", gDefaultUartPort_c);
             cTxRxState = gCTxRxStateIdle_c;
             ClearEvents();
           }else{
             DelayMs(100);DelayMs(100);DelayMs(50);
             bScanDone = FALSE;
             smacResult = MLMEScanRequest(0xFFFF, gScanModeED_c, au8ScanResults);
           }
         }
    break;
    default:
    break;
  }
  return bBackFlag;
}


/**************************************************************************************/
bool_t PacketErrorRateTx(void)
{
  static uint16_t u16TotalPacketsOptions[] = {1,25,100,500,1000,2000,5000,10000,65535};
  static uint16_t u16TotalPackets;
  static uint16_t u16SentPackets;
  uint8_t  au8LcdString1[]=" PER Tx running ";
  uint8_t  au8LcdString2[]="Sending ##### Pk";

  uint8_t u8Index;
  bool_t bBackFlag = FALSE;
  
  if(evTestParameters){
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  
  switch(perTxState)
  {
    case gPerTxStateInit_c:
         if(gSerialUI_c == selectedUI){
           PrintMenu(cu8ShortCutsBar, gDefaultUartPort_c);
           PrintMenu(cu8PerTxTestMenu, gDefaultUartPort_c);
           PrintTestParameters(FALSE);
           shortCutsEnabled = TRUE;           
           perTxState = gPerTxStateSelectPacketNum_c;
         }else{
           u16TotalPackets = gManualPerNumberOfPackets_c;
           int2BCD(u16TotalPackets,&au8LcdString2[8]);
           for(u8Index=8;u8Index<13;u8Index++) au8LcdString2[u8Index] += '0';
           (void)Lcd_WriteStringBlocking(1,au8LcdString1);      
           (void)Lcd_WriteStringBlocking(2,au8LcdString2);      
           LEDsRun();
           perTxState = gPerTxStateStartTest_c;
         }
    break;
    case gPerTxStateSelectPacketNum_c:
         if(evDataFromUART){
           if((gu8UartData >= '0') && (gu8UartData <= '8')){
             u16TotalPackets = u16TotalPacketsOptions[gu8UartData - '0'];
             shortCutsEnabled = FALSE;  
             perTxState = gPerTxStateStartTest_c;
           }else if('p' == gu8UartData){ 
             bBackFlag = TRUE;
           }
           ClearEvents();
         }
    break;
    case gPerTxStateStartTest_c:
         gAppTxPacket->u8DataLength = testPayloadLen;
         u16SentPackets = 0;
         gAppTxPacket->smacPdu.u8Data[0] = (u16TotalPackets >> 8);
         gAppTxPacket->smacPdu.u8Data[1] = (uint8_t)u16TotalPackets;
         gAppTxPacket->smacPdu.u8Data[2] = ((u16SentPackets+1) >> 8);
         gAppTxPacket->smacPdu.u8Data[3] = (uint8_t)(u16SentPackets+1);
         MemoryCpy(&(gAppTxPacket->smacPdu.u8Data[4]), "SMAC PER Demo",13);
         if(17 < testPayloadLen){
           for(u8Index=17;u8Index<testPayloadLen;u8Index++){
             gAppTxPacket->smacPdu.u8Data[u8Index] = (u8Index%10)+'0';            
           }
         }
         bTxDone = FALSE;
 	  	Led_Toggle(gLED1_c);
 	  	DelayMs(200);
 	  	Led_Toggle(gLED1_c);
         (void)MCPSDataRequest(gAppTxPacket);
         u16SentPackets++;
         if(gSerialUI_c == selectedUI){
           (void)Uart_BlockingStringTx("\f\r\n Running PER Tx, Sending ", gDefaultUartPort_c);
           PrintWordOnDecimalFormatBlocking(u16TotalPackets, 0, FALSE, gDefaultUartPort_c);
           (void)Uart_BlockingStringTx(" Packets", gDefaultUartPort_c);
         }
         perTxState = gPerTxStateRunningTest_c;
    break;
    case gPerTxStateRunningTest_c:
         if(bTxDone){
           if(u16SentPackets == u16TotalPackets){
             MemoryCpy(&(gAppTxPacket->smacPdu.u8Data[4]), "DONE",4);
             gAppTxPacket->u8DataLength = 8;
             u16SentPackets = 0;
             u16TotalPackets = mTotalFinalFrames_c;
             gAppTxPacket->u8DataLength = 8;
             perTxState = gPerTxStateSendingLastFrames_c;
           }else{
             gAppTxPacket->smacPdu.u8Data[2] = ((u16SentPackets+1) >> 8);
             gAppTxPacket->smacPdu.u8Data[3] = (uint8_t)(u16SentPackets+1);
             gAppTxPacket->u8DataLength = testPayloadLen;
           }
           bTxDone = FALSE;
           DelayMs(10);
   	  	Led_Toggle(gLED1_c);
   	  	DelayMs(200);
   	  	Led_Toggle(gLED1_c);
           (void)MCPSDataRequest(gAppTxPacket);
           u16SentPackets++;
         }
    break;
    case gPerTxStateSendingLastFrames_c:
         if(bTxDone){
           bTxDone = FALSE;
           if(u16SentPackets == u16TotalPackets){
             if(gSerialUI_c == selectedUI){
               (void)Uart_BlockingStringTx("\r\n PER Tx DONE \r\n", gDefaultUartPort_c);
               (void)Uart_BlockingStringTx("\r\n\r\n Press [enter] to go back to the PER Tx test menu ", gDefaultUartPort_c);
               perTxState = gPerTxStateIdle_c;
             }else{
               Buzzer_ShortBeep();
               LEDsStop();
               ClearEvents();
               bBackFlag = TRUE;
               perTxState = gPerTxStateIdle_c;               
             }
           }else{
             gAppTxPacket->u8DataLength = 8;
             DelayMs(10);
     	  	Led_Toggle(gLED1_c);
     	  	DelayMs(200);
     	  	Led_Toggle(gLED1_c);
             (void)MCPSDataRequest(gAppTxPacket);
             u16SentPackets++;
           }
         }
    break;
    case gPerTxStateIdle_c:
         if((evDataFromUART) && ('\r' == gu8UartData))
         {
           perTxState = gPerTxStateInit_c;
           ClearEvents();
         }
    break;
   default:
    break;
  }

  return bBackFlag;
}

/**************************************************************************************/
bool_t PacketErrorRateRx(void)
{
  static uint16_t u16ReceivedPackets;
  static uint16_t u16PacketsIndex;  
  static uint16_t u16TotalPackets;
  static uint8_t  u8FinalPacketsCount;
  static uint32_t u32LQISum;
  uint8_t  u8AverageLQI, u8TempLQIvalue;
  uint8_t  au8LcdString1[]="PER running -000";
  uint8_t  au8LcdString2[]="  00000, 00000  ";
  uint8_t  u8Index;

  bool_t bBackFlag = FALSE;
  
  if(evTestParameters){
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }

  switch(perRxState)
  {
    case gPerRxStateInit_c:
         u16TotalPackets = 0;
         u16ReceivedPackets = 0;
         u16PacketsIndex = 0;
         u32LQISum = 0;
         if(gSerialUI_c == selectedUI){
           PrintMenu(cu8ShortCutsBar, gDefaultUartPort_c);
           PrintMenu(cu8PerRxTestMenu, gDefaultUartPort_c);
           PrintTestParameters(FALSE);
           shortCutsEnabled = TRUE;           
           perRxState = gPerRxWaitStartTest_c;
         }else{
           LEDsRun();
           (void)Lcd_WriteStringBlocking(1,au8LcdString1);
           (void)Lcd_WriteStringBlocking(2,au8LcdString2);
           SetRadioRxOnTimeOut15ms();
           perRxState = gPerRxStateStartTest_c;
         }
    break;
    case gPerRxWaitStartTest_c:
         if(evDataFromUART){
           if(' ' == gu8UartData){
             (void)Uart_BlockingStringTx("\f\n\rPER Test Rx Running\r\n\r\n", gDefaultUartPort_c);
             SetRadioRxOnTimeOut15ms();
             shortCutsEnabled = FALSE;  
             perRxState = gPerRxStateStartTest_c;
           }else if('p' == gu8UartData){ 
             bBackFlag = TRUE;
           }
           ClearEvents();
         }
    break;
    case gPerRxStateStartTest_c:
         if(bRxDone){
           if (gAppRxPacket->rxStatus == rxSuccessStatus_c){
             if(stringComp("SMAC PER Demo",&gAppRxPacket->smacPdu.u8Data[4],13)){
               u16TotalPackets = ((uint16_t)gAppRxPacket->smacPdu.u8Data[0] <<8) + gAppRxPacket->smacPdu.u8Data[1];
               u16PacketsIndex = ((uint16_t)gAppRxPacket->smacPdu.u8Data[2] <<8) + gAppRxPacket->smacPdu.u8Data[3];
               u16ReceivedPackets++;
               (void)MLMELinkQuality(&u8TempLQIvalue);
               u32LQISum += u8TempLQIvalue;
               u8AverageLQI = (uint8_t)(u32LQISum/u16ReceivedPackets);
               if(gSerialUI_c == selectedUI){
                 (void)Uart_BlockingStringTx("Packet ", gDefaultUartPort_c);
                 PrintWordOnDecimalFormatBlocking(u16ReceivedPackets, 1, FALSE, gDefaultUartPort_c);
                 (void)Uart_BlockingStringTx("\r\n", gDefaultUartPort_c);
               }else{
                 #if !defined(gMcs08Gt60Platform_d)
                 int2BCD((uint16_t)u8AverageLQI,&au8LcdString1[11]);
                 au8LcdString1[11] = ' ';         
                 for(u8Index=12;u8Index<16;u8Index++) au8LcdString1[u8Index] += '0';
                 (void)Lcd_WriteStringBlocking(1,au8LcdString1);
                #endif
                 int2BCD(u16ReceivedPackets,&au8LcdString2[2]);
                 int2BCD((u16PacketsIndex-u16ReceivedPackets),&au8LcdString2[9]);
                 for(u8Index=2;u8Index<7;u8Index++) au8LcdString2[u8Index] += '0';                 
                 for(u8Index=9;u8Index<14;u8Index++) au8LcdString2[u8Index] += '0';                 
                 (void)Lcd_WriteStringBlocking(2,au8LcdString2);
               }
               if(u16PacketsIndex == u16TotalPackets){
                 u8FinalPacketsCount = 0; 
                 perRxState = gPerRxStateReceivingLastFrames_c;
               }
             }else if(stringComp("DONE",&gAppRxPacket->smacPdu.u8Data[4],4)){
               u8FinalPacketsCount = 0; 
               perRxState = gPerRxStateReceivingLastFrames_c;
             }
           }else{ 
             if(u16TotalPackets){
               u16PacketsIndex++;
               if(u16PacketsIndex == u16TotalPackets){
                 u8FinalPacketsCount = 0; 
                 perRxState = gPerRxStateReceivingLastFrames_c;
               }              
             }
           }
           SetRadioRxOnTimeOut15ms();
         }
         if(evDataFromUART){
           if(' ' == gu8UartData){
             (void)MLMERXDisableRequest();
             (void)Uart_BlockingStringTx("\n\rPER Test Rx Stopped\r\n\r\n", gDefaultUartPort_c);
             PrintPerRxFinalLine(u16ReceivedPackets,u16TotalPackets);
             perRxState = gPerRxStateIdle_c;
           } 
           ClearEvents();
         }
         if(evSwitch4){
           (void)MLMERXDisableRequest();
           bBackFlag = TRUE;           
           rangeRxState = gRangeRxStateIdle_c;
           ClearEvents();
         }         
    break;
    case gPerRxStateReceivingLastFrames_c:
         if(bRxDone){
           u8FinalPacketsCount++; 
           if(mTotalFinalFrames_c == u8FinalPacketsCount){
             if(gSerialUI_c == selectedUI){
               (void)Uart_BlockingStringTx("\n\rPER Test Finished\r\n\r\n", gDefaultUartPort_c);
               PrintPerRxFinalLine(u16ReceivedPackets,u16TotalPackets);              
             }else{
               Buzzer_ShortBeep();
               LEDsStop();
               u8Index=0;
               if(u16ReceivedPackets > (u16TotalPackets/4)) u8Index |= 0x01; 
               if(u16ReceivedPackets > (u16TotalPackets/4)*2) u8Index |= 0x02; 
               if(u16ReceivedPackets > (u16TotalPackets/4)*3) u8Index |= 0x04; 
               if(u16ReceivedPackets == u16TotalPackets) u8Index |= 0x08; 
               LEDsFlashValue(u8Index,20);
               bBackFlag = TRUE;
             }
             perRxState = gPerRxStateIdle_c;
           }else{  
             SetRadioRxOnTimeOut15ms();
           }
         }
    break;
    case gPerRxStateIdle_c:
         if((evDataFromUART) && ('\r' == gu8UartData)){
           perRxState = gPerRxStateInit_c;
         }
         ClearEvents();
    break;
    default:
    break;
  }
  return bBackFlag;
}

/**************************************************************************************/
bool_t RangeTx(void)
{
  bool_t bBackFlag = FALSE;
  static uint32_t u32LQISum;
  static uint16_t u16ReceivedPackets;
  static uint16_t u16PacketsDropped;
  uint8_t  u8AverageLQI;
  uint8_t  u8CurrentLQI;
  uint8_t  au8LcdString1[]="Range Tx running";
  uint8_t  au8LcdString2[]="LQI:###,AVG:###";
  uint8_t u8Index;

  
  if(evTestParameters){
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  
  switch(rangeTxState)
  {
    case gRangeTxStateInit_c:
         u32LQISum = 0;
         u16ReceivedPackets = 0;
         u16PacketsDropped = 0;
         if(gSerialUI_c == selectedUI){
           PrintMenu(cu8ShortCutsBar, gDefaultUartPort_c);
           PrintMenu(cu8RangeTxTestMenu, gDefaultUartPort_c);
           PrintTestParameters(FALSE);
           shortCutsEnabled = TRUE;           
           rangeTxState = gRangeTxWaitStartTest_c;
         }else{
           (void)Lcd_WriteStringBlocking(1,au8LcdString1);      
           (void)Lcd_WriteStringBlocking(2,au8LcdString2);      
           rangeTxState = gRangeTxStateStartTest_c;
         }
    break;
    case gRangeTxWaitStartTest_c:
         if(evDataFromUART){
           if(' ' == gu8UartData){
             shortCutsEnabled = FALSE; 
             (void)Uart_BlockingStringTx("\f\r\nRange Test Tx Running\r\n", gDefaultUartPort_c);
             rangeTxState = gRangeTxStateStartTest_c;
           }else if('p' == gu8UartData){ 
             bBackFlag = TRUE;
           }
           ClearEvents();
         }
    break;
    case gRangeTxStateStartTest_c:
         DelayMs(100);
         DelayMs(100);
         bTxDone = FALSE;
         gAppTxPacket->u8DataLength = 16;
         gAppTxPacket->smacPdu.u8Data[0]  = 0;
         MemoryCpy(&(gAppTxPacket->smacPdu.u8Data[1]), "SMAC Range Demo",15);
 	  	Led_Toggle(gLED1_c);
 	  	DelayMs(200);
 	  	Led_Toggle(gLED1_c);
         (void)MCPSDataRequest(gAppTxPacket);
         rangeTxState = gRangeTxStateRunningTest_c;
    break;
    case gRangeTxStateRunningTest_c:
         if(bTxDone){
           SetRadioRxOnTimeOut15ms();
           rangeTxState = gRangeTxStatePrintTestResults_c;
         }
    break;
    case gRangeTxStatePrintTestResults_c:
         if(bRxDone){
           if(gAppRxPacket->rxStatus == rxSuccessStatus_c){ 
             if(stringComp("SMAC Range Demo",&gAppRxPacket->smacPdu.u8Data[1],15)){
               u8CurrentLQI = (gAppRxPacket->smacPdu.u8Data[0]); 
               u32LQISum += u8CurrentLQI;  
               u16ReceivedPackets++;
               u8AverageLQI = (uint8_t)(u32LQISum/u16ReceivedPackets);
               if(gSerialUI_c == selectedUI){
                 (void)Uart_BlockingStringTx("\r\n LQI = ", gDefaultUartPort_c);
                 PrintWordOnDecimalFormatBlocking((uint16_t)u8CurrentLQI,1,FALSE, gDefaultUartPort_c);               
               }else{
                 int2BCD((uint16_t)u8CurrentLQI,&au8LcdString2[2]);
                 au8LcdString2[2] = 'I';
                 au8LcdString2[3] = ':';                 
                 for(u8Index=4;u8Index<7;u8Index++) au8LcdString2[u8Index] += '0';
                 int2BCD((uint16_t)u8AverageLQI,&au8LcdString2[10]);
                 au8LcdString2[10] = 'G';
                 au8LcdString2[11] = ':';                  
                 for(u8Index=12;u8Index<15;u8Index++) au8LcdString2[u8Index] += '0';
                 (void)Lcd_WriteStringBlocking(2,au8LcdString2);      
                 LEDs_PrintEnergyLevelOnLEDs(u8CurrentLQI);
               }
             }else{
               SetRadioRxOnTimeOut15ms();
             }
           }else{
             if(gSerialUI_c == selectedUI){
               u16PacketsDropped++;
               (void)Uart_BlockingStringTx("\r\nPacket Dropped", gDefaultUartPort_c);
             }
           }
           if(evDataFromUART && (' ' == gu8UartData)){
             (void)Uart_BlockingStringTx("\n\r\n\rRange Test Tx Stopped\r\n\r\n", gDefaultUartPort_c);
             u8AverageLQI = (uint8_t)(u32LQISum/u16ReceivedPackets);
             (void)Uart_BlockingStringTx("Average LQI     ", gDefaultUartPort_c);
             PrintWordOnDecimalFormatBlocking((uint16_t)u8AverageLQI,1,FALSE, gDefaultUartPort_c);  
             (void)Uart_BlockingStringTx("\r\nPackets dropped ", gDefaultUartPort_c);
             PrintWordOnDecimalFormatBlocking(u16PacketsDropped,1,FALSE, gDefaultUartPort_c);  
             (void)Uart_BlockingStringTx("\r\n\r\n Press [enter] to go back to the Range Tx test menu", gDefaultUartPort_c);
             rangeTxState = gRangeTxStateIdle_c;
           }else if(evSwitch4){
             bBackFlag = TRUE;           
             rangeTxState = gRangeTxStateIdle_c;
           }else{
             rangeTxState = gRangeTxStateStartTest_c;
           }
           ClearEvents();
         }
    break;
    case gRangeTxStateIdle_c:
         if((evDataFromUART) && ('\r' == gu8UartData))
         {
           rangeTxState = gRangeTxStateInit_c;
         }
         ClearEvents();
    break;
    default:
    break;
  }
  return bBackFlag;
}

/**************************************************************************************/
bool_t RangeRx(void)
{
  bool_t bBackFlag = FALSE;
  static uint32_t u32LQISum;
  static uint16_t u16ReceivedPackets;
  uint8_t  u8AverageLQI, u8TempLQIvalue;
  uint8_t  u8CurrentLQI;
  uint8_t  au8LcdString1[]="Range Rx running";
  uint8_t  au8LcdString2[]="LQI:###,AVG:###";
  uint8_t u8Index;

  if(evTestParameters){
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }

  switch(rangeRxState)
  {
    case gRangeRxStateInit_c:
         u32LQISum = 0;
         u16ReceivedPackets = 0;
         if(gSerialUI_c == selectedUI){
           PrintMenu(cu8ShortCutsBar, gDefaultUartPort_c);
           PrintMenu(cu8RangeRxTestMenu, gDefaultUartPort_c);
           PrintTestParameters(FALSE);
           shortCutsEnabled = TRUE;           
           rangeRxState = gRangeRxWaitStartTest_c;
         }else{
           (void)Lcd_WriteStringBlocking(1,au8LcdString1);      
           (void)Lcd_WriteStringBlocking(2,au8LcdString2);               
           rangeRxState = gRangeRxStateStartTest_c;
         }
    break;
    case gRangeRxWaitStartTest_c:
         if(evDataFromUART){
           if(' ' == gu8UartData){
             shortCutsEnabled = FALSE; 
             (void)Uart_BlockingStringTx("\f\r\nRange Test Rx Running\r\n", gDefaultUartPort_c);
             rangeRxState = gRangeRxStateStartTest_c;
           }else if('p' == gu8UartData){ 
             bBackFlag = TRUE;
           }
           ClearEvents();
         }
    break;
    case gRangeRxStateStartTest_c:
         SetRadioRxOnNoTimeOut();
         rangeRxState = gRangeRxStateRunningTest_c;
    break;
    case gRangeRxStateRunningTest_c:
         if(evDataFromUART && (' ' == gu8UartData))
         {             
           (void)MLMERXDisableRequest();
           (void)Uart_BlockingStringTx("\n\r\n\rRange Test Rx Stopped\r\n\r\n", gDefaultUartPort_c);
           u8AverageLQI = (uint8_t)(u32LQISum/u16ReceivedPackets);
           (void)Uart_BlockingStringTx("Average LQI     ", gDefaultUartPort_c);
           PrintWordOnDecimalFormatBlocking((uint16_t)u8AverageLQI,1,FALSE, gDefaultUartPort_c);  
           (void)Uart_BlockingStringTx("\r\n\r\n Press [enter] to go back to the Range Rx test menu", gDefaultUartPort_c);
           rangeRxState = gRangeRxStateIdle_c;
         }
         if(evSwitch4){
           bBackFlag = TRUE;           
           rangeRxState = gRangeRxStateIdle_c;
           ClearEvents();
         }
         ClearEvents();
         if(bRxDone){
           if(gAppRxPacket->rxStatus == rxSuccessStatus_c){ 
             if(stringComp("SMAC Range Demo",&gAppRxPacket->smacPdu.u8Data[1],15)){
               bTxDone = FALSE;
			   DelayMs(2);
			   (void)MLMELinkQuality(&u8TempLQIvalue);
               gAppTxPacket->smacPdu.u8Data[0]  = u8TempLQIvalue;
               MemoryCpy(&(gAppTxPacket->smacPdu.u8Data[1]), "SMAC Range Demo",15);
               gAppTxPacket->u8DataLength = 16;
       	  	Led_Toggle(gLED1_c);
       	  	DelayMs(200);
       	  	Led_Toggle(gLED1_c);
               (void)MCPSDataRequest(gAppTxPacket);
               rangeRxState = gRangeRxStatePrintTestResults_c;
             }else{
               SetRadioRxOnNoTimeOut();
             }
           }else{
             SetRadioRxOnNoTimeOut();
           }
         }
    break;
    case gRangeRxStatePrintTestResults_c:
         if(bTxDone)
         {
           (void)MLMELinkQuality(&u8CurrentLQI); 
           u32LQISum += u8CurrentLQI;
           u16ReceivedPackets++;
           u8AverageLQI = (uint8_t)(u32LQISum/u16ReceivedPackets);
 
           if(gSerialUI_c == selectedUI){
             (void)Uart_BlockingStringTx("\r\n LQI = ", gDefaultUartPort_c);
             PrintWordOnDecimalFormatBlocking((uint16_t)u8CurrentLQI,1,FALSE, gDefaultUartPort_c);               
           }else{
             int2BCD((uint16_t)u8CurrentLQI,&au8LcdString2[2]);
             au8LcdString2[2] = 'I';
             au8LcdString2[3] = ':';               
             for(u8Index=4;u8Index<7;u8Index++) au8LcdString2[u8Index] += '0';
             int2BCD((uint16_t)u8AverageLQI,&au8LcdString2[10]);
             au8LcdString2[10] = 'G';
             au8LcdString2[11] = ':';                  
             for(u8Index=12;u8Index<15;u8Index++) au8LcdString2[u8Index] += '0';
             (void)Lcd_WriteStringBlocking(2,au8LcdString2);      
             LEDs_PrintEnergyLevelOnLEDs(u8CurrentLQI);
           }
           rangeRxState = gRangeRxStateStartTest_c;
         }
    break;
    case gRangeRxStateIdle_c:
         if((evDataFromUART) && ('\r' == gu8UartData))
         {
           rangeRxState = gRangeRxStateInit_c;
         }
         ClearEvents();
    break;
    default:
    break;
  }
  return bBackFlag;
}

#if defined(gMc1323xPlatform_d)
/**************************************************************************************/
/**************************************************************************************/
bool_t EditRegisters(void)
{
  bool_t bBackFlag = FALSE;
  if(evTestParameters){
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }

  switch(eRState)
  {
    case gERStateInit_c:
         PrintMenu(cu8ShortCutsBar, gDefaultUartPort_c);
         PrintMenu(cu8RadioRegistersEditMenu, gDefaultUartPort_c);
         PrintTestParameters(FALSE);
         shortCutsEnabled = TRUE;           
         eRState = gERWaitSelection_c;
    break;
    case gERWaitSelection_c:
         if(evDataFromUART){
           if('1' == gu8UartData){
             oDRState = gODRStateInit_c;
             eRState = gERStateOverrideDirectReg_c;
           }else if('2' == gu8UartData){
             rDRState = gRDRStateInit_c;
             eRState = gERStateReadDirectReg_c;
           }else if('3' == gu8UartData){
             oIRState = gOIRStateInit_c;
             eRState = gERStateOverrideIndirectReg_c;
           }else if('4' == gu8UartData){
             rIRState = gRIRStateInit_c;
             eRState = gERStateReadIndirectReg_c;
           }else if('p' == gu8UartData){ 
             bBackFlag = TRUE;
           }
           ClearEvents();
         }
    break;
    case gERStateOverrideDirectReg_c:
         if(OverrideDirectRegisters()) {
             eRState = gERStateInit_c;
         }    
    break;
    case gERStateOverrideIndirectReg_c:
         if(OverrideIndirectRegisters()) {
             eRState = gERStateInit_c;
         }    
    break;
    case gERStateReadDirectReg_c:
         if(ReadDirectRegisters()) {
             eRState = gERStateInit_c;
         }    
    break;
    case gERStateReadIndirectReg_c:
         if(ReadIndirectRegisters()) {
             eRState = gERStateInit_c;
         }    
    break;
    default:
    break;
  }
  return bBackFlag;
}


/**************************************************************************************/
bool_t OverrideDirectRegisters(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t au8RxString[5];
  static uint8_t u8Index;
  static uint16_t u16RegAddress;
  static uint8_t u8RegValue;
  
  
  switch(oDRState)
  {
    case gODRStateInit_c:
         (void)Uart_BlockingStringTx("\f\r\nOverride Direct Registers\r\n", gDefaultUartPort_c);
         (void)Uart_BlockingStringTx("\r\n-Press [p] Previous Menu\r\n", gDefaultUartPort_c);
         shortCutsEnabled = FALSE;   
         oDRState = gODRStateStart_c;
    break;
    case gODRStateStart_c:
         (void)Uart_BlockingStringTx("\r\n -write the Register address in Hex and [enter]: 0x", gDefaultUartPort_c);
         u8Index = 0;
         oDRState = gODRWaitForTheAddress_c; 
    break;
    case gODRWaitForTheAddress_c:
         if(evDataFromUART){
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData)){
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               oDRState = gODRStateStart_c;              
             }
           }else if((4 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             oDRState = gODRStateStart_c;
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u16RegAddress = HexString2Dec16(au8RxString);
             (void)Uart_BlockingStringTx("\r\n -write the Register value to override in Hex and [enter]: 0x", gDefaultUartPort_c);
             u8Index = 0;
             oDRState = gODRWaitForTheValue_c; 
           }
           ClearEvents();
         }
    break;
    case gODRWaitForTheValue_c:
         if(evDataFromUART){
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData)){
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               oDRState = gODRStateStart_c;              
             }
           }else if((2 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             oDRState = gODRStateStart_c;              
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u8RegValue = (uint8_t)(HexString2Dec16(au8RxString) & 0x00FF);
             *((uint8_t *)(u16RegAddress))  = u8RegValue;
             (void)Uart_BlockingStringTx("\r\n Register overridden \r\n", gDefaultUartPort_c);
             u8Index = 0;
             oDRState = gODRStateStart_c; 
           }
           ClearEvents();
         }
    break;
    default:
    break;
  }
  return bBackFlag;  
}

/**************************************************************************************/
bool_t OverrideIndirectRegisters(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t au8RxString[3];
  static uint8_t u8Index;
  static uint8_t u8RegAddress;
  static uint8_t u8RegValue;
  
  
  switch(oIRState)
  {
    case gOIRStateInit_c:
         (void)Uart_BlockingStringTx("\f\r\nOverride Indirect Registers\r\n", gDefaultUartPort_c);
         (void)Uart_BlockingStringTx("\r\n-Press [p] Previous Menu\r\n", gDefaultUartPort_c);
         shortCutsEnabled = FALSE;   
         oIRState = gOIRStateStart_c;
    break;
    case gOIRStateStart_c:
         (void)Uart_BlockingStringTx("\r\n -write the Register address in Hex and [enter]: 0x", gDefaultUartPort_c);
         u8Index = 0;
         oIRState = gOIRWaitForTheAddress_c; 
    break;
    case gOIRWaitForTheAddress_c:
         if(evDataFromUART){
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData)){
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               oIRState = gOIRStateStart_c;              
             }
           }else if((2 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             oIRState = gOIRStateStart_c;   
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u8RegAddress = (uint8_t)(HexString2Dec16(au8RxString) & 0x00FF);
             (void)Uart_BlockingStringTx("\r\n -write the Register value to override in Hex and [enter]: 0x", gDefaultUartPort_c);
             u8Index = 0;
             oIRState = gOIRWaitForTheValue_c; 
           }
           ClearEvents();
         }
    break;
    case gOIRWaitForTheValue_c:
         if(evDataFromUART)
         {
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData)){
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               oIRState = gOIRStateStart_c;              
             }
           }else if((2 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             bBackFlag = TRUE;
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u8RegValue = (uint8_t)(HexString2Dec16(au8RxString) & 0x00FF);
             IoIndirectWrite(u8RegAddress, u8RegValue);
             (void)Uart_BlockingStringTx("\r\n Register overridden \r\n", gDefaultUartPort_c);
             u8Index = 0;
             oIRState = gOIRStateStart_c; 
           }
           ClearEvents();
         }
    break;
    default:
    break;
  }
  return bBackFlag;  
}


/**************************************************************************************/
bool_t ReadDirectRegisters(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t au8RxString[5];
  static uint8_t u8Index;
  static uint16_t u16RegAddress;
  static uint8_t u8RegValue;
  
  switch(rDRState)
  {
    case gRDRStateInit_c:
         (void)Uart_BlockingStringTx("\f\r\rRead Direct Registers\r\n", gDefaultUartPort_c);
         (void)Uart_BlockingStringTx("\r\n-Press [p] Previous Menu\r\n", gDefaultUartPort_c);
         shortCutsEnabled = FALSE;   
         rDRState = gRDRStateStart_c;
    break;
    case gRDRStateStart_c:
         (void)Uart_BlockingStringTx("\r\n -write the Register address in Hex and [enter]: 0x", gDefaultUartPort_c);
         u8Index = 0;
         rDRState = gRDRWaitForTheAddress_c; 
    break;
    case gRDRWaitForTheAddress_c:
         if(evDataFromUART){
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData)){
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               rDRState = gRDRStateStart_c;              
             }
           }else if((4 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             rDRState = gRDRStateStart_c;
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u16RegAddress = HexString2Dec16(au8RxString);
             u8RegValue = *((uint8_t *)(u16RegAddress)); 
             (void)Uart_BlockingStringTx("\r\n -Register value : 0x", gDefaultUartPort_c);
             PrintByteOnHexFormatBlocking(u8RegValue,FALSE, gDefaultUartPort_c);
             (void)Uart_BlockingStringTx("\r\n", gDefaultUartPort_c);
             
             rDRState = gRDRStateStart_c; 
           }
           ClearEvents();
         }
    break;
    default:
    break;
  }
  return bBackFlag;  
}

/**************************************************************************************/
bool_t ReadIndirectRegisters(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t au8RxString[5];
  static uint8_t u8Index;
  static uint8_t u8RegAddress;
  static uint8_t u8RegValue;
  
  switch(rIRState)
  {
    case gRIRStateInit_c:
         (void)Uart_BlockingStringTx("\f\r\rRead Indirect Registers\r\n", gDefaultUartPort_c);
         (void)Uart_BlockingStringTx("\r\n-Press [p] Previous Menu\r\n", gDefaultUartPort_c);
         shortCutsEnabled = FALSE;   
         rIRState = gRIRStateStart_c;
    break;
    case gRIRStateStart_c:
         (void)Uart_BlockingStringTx("\r\n -write the Register address in Hex and [enter]: 0x", gDefaultUartPort_c);
         u8Index = 0;
         rIRState = gRIRWaitForTheAddress_c; 
    break;
    case gRIRWaitForTheAddress_c:
         if(evDataFromUART){
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData))
           {
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               rIRState = gRIRStateStart_c;              
             }
           }else if((2 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             rIRState = gRIRStateStart_c; 
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u8RegAddress = (uint8_t)(HexString2Dec16(au8RxString) & 0x00FF);
             u8RegValue = IoIndirectRead(u8RegAddress); 
             (void)Uart_BlockingStringTx("\r\n -Register value : 0x", gDefaultUartPort_c);
             PrintByteOnHexFormatBlocking(u8RegValue, FALSE, gDefaultUartPort_c);
             (void)Uart_BlockingStringTx("\r\n", gDefaultUartPort_c);
             rIRState = gRIRStateStart_c; 
           }
           ClearEvents();
         }
    break;
    default:
    break;
  }
  return bBackFlag;  
} 

bool_t SendReceivePacketsTxFunc(void)
{
  static uint16_t u16TotalPacketsOptions[] = {1,25,100,500,1000,2000,5000,10000,65535};
  static uint16_t u16TotalPackets;
  static uint16_t u16SentPackets;
  uint8_t buffer[50];
  uint8_t i;
  uint8_t  au8LcdString1[]=" PER Tx running ";
  uint8_t  au8LcdString2[]="Sending ##### Pk";

  uint8_t u8Index;
  bool_t bBackFlag = FALSE;
  
  if(evTestParameters){
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  
  switch(SendReceivePacketsTx)
  {
    case gSendReceivePacketsTxInit_c:
//         if(gSerialUI_c == selectedUI){
           PrintMenu(cu8ShortCutsBar, gDefaultUartPort_c);
           PrintMenu(cu8SendReceivePacketsTx, gDefaultUartPort_c);
           PrintTestParameters(FALSE);
           shortCutsEnabled = TRUE;  
           SendReceivePacketsTx = gSendReceivePacketsTxSelectTest_c;
//         }else{
//           u16TotalPackets = gManualPerNumberOfPackets_c;
//           int2BCD(u16TotalPackets,&au8LcdString2[8]);
//           for(u8Index=8;u8Index<13;u8Index++) au8LcdString2[u8Index] += '0';
//           (void)Lcd_WriteStringBlocking(1,au8LcdString1);      
//           (void)Lcd_WriteStringBlocking(2,au8LcdString2);      
//           LEDsRun();
//           perTxState = gPerTxStateStartTest_c;
//         }
    break;
    case gSendReceivePacketsTxSelectTest_c:
         if(evDataFromUART){
        	 if('1' == gu8UartData){
        		 SendReceivePacketsTx = gSendReceivePacketsTxSinglePacket_c;
        	 }else if('2' == gu8UartData){
				 SendReceivePacketsTx = gSendReceivePacketsTxSizeVariablePacket_c;
			 }else if('3' == gu8UartData){
				 SendReceivePacketsTx = gSendReceivePacketsTxMessenger_c;
			 }else if('p' == gu8UartData){ 
	             bBackFlag = TRUE;
	          }
           ClearEvents();
         }
    break;
    case gSendReceivePacketsTxSinglePacket_c:
    	gAppTxPacket->u8DataLength = 3;
      	MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "1",3);
	  	Led_Toggle(gLED1_c);
	  	DelayMs(200);
	  	Led_Toggle(gLED1_c);
      	(void)MCPSDataRequest(gAppTxPacket);
      	SendReceivePacketsTx = gSendReceivePacketsTxSelectTest_c;
    break;
    case gSendReceivePacketsTxSizeVariablePacket_c:
    	while(!evDataFromUART);
    	evDataFromUART = 0;
        gAppTxPacket->u8DataLength = gu8UartData + 2;
        	
        if('1' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "1 payload size:1",17);
		}else if('2' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "12 payload size:2",18);
		}else if('3' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "123 payload size:3",19);
		}else if('4' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "1234 payload size:4",20);
		}else if('5' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "12345 payload size:5",21);
		}else if('6' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "123456 payload size:6",22);
		}else if('7' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "1234567 payload size:7",23);
		}else if('8' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "12345678 payload size:8",24);
		}else if('9' == gu8UartData){
			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "123456789 payload size:9",25);
		}else if('p' == gu8UartData){ 
			SendReceivePacketsTx = gSendReceivePacketsTxSelectTest_c;
			evDataFromUART = 0;
			break;
		}
	  	Led_Toggle(gLED1_c);
	  	DelayMs(200);
	  	Led_Toggle(gLED1_c);
      	(void)MCPSDataRequest(gAppTxPacket);
      	SendReceivePacketsTx = gSendReceivePacketsTxSizeVariablePacket_c;
    break;
    case gSendReceivePacketsTxMessenger_c:
    	gAppTxPacket->u8DataLength = 50;
    	shortCutsEnabled = 0;
    	memset(buffer,0,50);
    	
    	for(i = 0; 13 != gu8UartData; i++)
    	{
    		while(!evDataFromUART);
    		evDataFromUART = 0;
    		if((13 != gu8UartData) && ('*' != gu8UartData))
    			buffer[i] = gu8UartData;
    		else if('*' == gu8UartData)
    		{
    			SendReceivePacketsTx = gSendReceivePacketsTxSelectTest_c;
    			shortCutsEnabled = 1;
    			break;
    		}
    		else
    		{
    			buffer[i] = gu8UartData;
    			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), buffer, 50);
    			(void)MCPSDataRequest(gAppTxPacket);
    		}
    	}
//    	while(!evDataFromUART);
//    	evDataFromUART = 0;
//        	gAppTxPacket->u8DataLength = gu8UartData + 2;
//    	MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "                   ",20);
//		if('1' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "1  ",3);
//		}else if('2' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "12  ",4);
//		}else if('3' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "123  ",5);
//		}else if('4' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "1234  ",6);
//		}else if('5' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "12345  ",7);
//		}else if('6' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "123456  ",8);
//		}else if('7' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "1234567  ",9);
//		}else if('8' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "12345678  ",10);
//		}else if('9' == gu8UartData){
//			MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "123456789  ",11);
//		}else if('p' == gu8UartData){ 
//			SendReceivePacketsTx = gSendReceivePacketsTxSelectTest_c;
//			evDataFromUART = 0;
//			break;
//		}
//	  	Led_Toggle(gLED1_c);
//	  	DelayMs(200);
//	  	Led_Toggle(gLED1_c);
//      	(void)MCPSDataRequest(gAppTxPacket);
//      	SendReceivePacketsTx = gSendReceivePacketsTxSizeVariablePacket_c;
    break;
//    case gSendReceivePacketsTxSelectTest_c:
//         if(evDataFromUART){
//           if((gu8UartData >= '0') && (gu8UartData <= '8')){
//             u16TotalPackets = u16TotalPacketsOptions[gu8UartData - '0'];
//             shortCutsEnabled = FALSE;  
//             perTxState = gPerTxStateStartTest_c;
//           }else if('p' == gu8UartData){ 
//             bBackFlag = TRUE;
//           }
//           ClearEvents();
//         }
//    break;
//    case gPerTxStateStartTest_c:
//         gAppTxPacket->u8DataLength = testPayloadLen;
//         u16SentPackets = 0;
//         gAppTxPacket->smacPdu.u8Data[0] = (u16TotalPackets >> 8);
//         gAppTxPacket->smacPdu.u8Data[1] = (uint8_t)u16TotalPackets;
//         gAppTxPacket->smacPdu.u8Data[2] = ((u16SentPackets+1) >> 8);
//         gAppTxPacket->smacPdu.u8Data[3] = (uint8_t)(u16SentPackets+1);
//         MemoryCpy(&(gAppTxPacket->smacPdu.u8Data[4]), "SMAC PER Demo",13);
//         if(17 < testPayloadLen){
//           for(u8Index=17;u8Index<testPayloadLen;u8Index++){
//             gAppTxPacket->smacPdu.u8Data[u8Index] = (u8Index%10)+'0';            
//           }
//         }
//         bTxDone = FALSE;
// 	  	Led_Toggle(gLED1_c);
// 	  	DelayMs(200);
// 	  	Led_Toggle(gLED1_c);
//         (void)MCPSDataRequest(gAppTxPacket);
//         u16SentPackets++;
//         if(gSerialUI_c == selectedUI){
//           (void)Uart_BlockingStringTx("\f\r\n Running PER Tx, Sending ", gDefaultUartPort_c);
//           PrintWordOnDecimalFormatBlocking(u16TotalPackets, 0, FALSE, gDefaultUartPort_c);
//           (void)Uart_BlockingStringTx(" Packets", gDefaultUartPort_c);
//         }
//         perTxState = gPerTxStateRunningTest_c;
//    break;
//    case gPerTxStateRunningTest_c:
//         if(bTxDone){
//           if(u16SentPackets == u16TotalPackets){
//             MemoryCpy(&(gAppTxPacket->smacPdu.u8Data[4]), "DONE",4);
//             gAppTxPacket->u8DataLength = 8;
//             u16SentPackets = 0;
//             u16TotalPackets = mTotalFinalFrames_c;
//             gAppTxPacket->u8DataLength = 8;
//             perTxState = gPerTxStateSendingLastFrames_c;
//           }else{
//             gAppTxPacket->smacPdu.u8Data[2] = ((u16SentPackets+1) >> 8);
//             gAppTxPacket->smacPdu.u8Data[3] = (uint8_t)(u16SentPackets+1);
//             gAppTxPacket->u8DataLength = testPayloadLen;
//           }
//           bTxDone = FALSE;
//           DelayMs(10);
//   	  	Led_Toggle(gLED1_c);
//   	  	DelayMs(200);
//   	  	Led_Toggle(gLED1_c);
//           (void)MCPSDataRequest(gAppTxPacket);
//           u16SentPackets++;
//         }
//    break;
//    case gPerTxStateSendingLastFrames_c:
//         if(bTxDone){
//           bTxDone = FALSE;
//           if(u16SentPackets == u16TotalPackets){
//             if(gSerialUI_c == selectedUI){
//               (void)Uart_BlockingStringTx("\r\n PER Tx DONE \r\n", gDefaultUartPort_c);
//               (void)Uart_BlockingStringTx("\r\n\r\n Press [enter] to go back to the PER Tx test menu ", gDefaultUartPort_c);
//               perTxState = gPerTxStateIdle_c;
//             }else{
//               Buzzer_ShortBeep();
//               LEDsStop();
//               ClearEvents();
//               bBackFlag = TRUE;
//               perTxState = gPerTxStateIdle_c;               
//             }
//           }else{
//             gAppTxPacket->u8DataLength = 8;
//             DelayMs(10);
//     	  	Led_Toggle(gLED1_c);
//     	  	DelayMs(200);
//     	  	Led_Toggle(gLED1_c);
//             (void)MCPSDataRequest(gAppTxPacket);
//             u16SentPackets++;
//           }
//         }
//    break;
//    case gPerTxStateIdle_c:
//         if((evDataFromUART) && ('\r' == gu8UartData))
//         {
//           perTxState = gPerTxStateInit_c;
//           ClearEvents();
//         }
//    break;
   default:
    break;
  }

  return bBackFlag;
}

#else
/**************************************************************************************/
bool_t EditRegisters(void)
{
  bool_t bBackFlag = FALSE;
  if(evTestParameters){
    (void)MLMESetChannelRequest(testChannel);
    (void)MLMEPAOutputAdjust(testPower);
    (void)MLMEXtalAdjust(testTrimmValue);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }

  switch(eRState)
  {
    case gERStateInit_c:
         PrintMenu(cu8ShortCutsBar, gDefaultUartPort_c);
         PrintMenu(cu8RadioRegistersEditMenu, gDefaultUartPort_c);
         PrintTestParameters(FALSE);
         shortCutsEnabled = TRUE;           
         eRState = gERWaitSelection_c;
    break;
    case gERWaitSelection_c:
         if(evDataFromUART){
           if('1' == gu8UartData){
             oSRRState = gOIRStateInit_c;
             eRState = gERStateOverrideRadioSpiReg_c;
           }else if('2' == gu8UartData){
             oSRRState = gRIRStateInit_c;
             eRState = gERStateReadRadioSpiReg_c;
           }else if('p' == gu8UartData){ 
             bBackFlag = TRUE;
           }
           ClearEvents();
         }
    break;
    case gERStateOverrideRadioSpiReg_c:
         if(OverrideSpiRadioRegisters()) {
             eRState = gERStateInit_c;
         }    
    break;
    case gERStateReadRadioSpiReg_c:
         if(ReadSpiRadioRegisters()) {
             eRState = gERStateInit_c;
         }    
    break;
    default:
    break;
  }
  return bBackFlag;
}


/**************************************************************************************/
bool_t OverrideSpiRadioRegisters(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t au8RxString[5];
  static uint8_t u8Index;
  static uint8_t u8RegAddress;
  static uint16_t u16RegValue;
  
  
  switch(oSRRState)
  {
    case gORStateInit_c:
         (void)Uart_BlockingStringTx("\f\r\nOverride SPI Radio Registers\r\n", gDefaultUartPort_c);
         (void)Uart_BlockingStringTx("\r\n-Press [p] Previous Menu\r\n", gDefaultUartPort_c);
         shortCutsEnabled = FALSE;   
         oSRRState = gORStateStart_c;
    break;
    case gORStateStart_c:
         (void)Uart_BlockingStringTx("\r\n -write the Register address in Hex and [enter]: 0x", gDefaultUartPort_c);
         u8Index = 0;
         oSRRState = gORWaitForTheAddress_c; 
    break;
    case gORWaitForTheAddress_c:
         if(evDataFromUART){
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData)){
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               oSRRState = gORStateStart_c;              
             }
           }else if((2 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             oSRRState = gORStateStart_c;   
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u8RegAddress = (uint8_t)(HexString2Dec16(au8RxString) & 0x00FF);
             (void)Uart_BlockingStringTx("\r\n -write the Register value to override in Hex and [enter]: 0x", gDefaultUartPort_c);
             u8Index = 0;
             oSRRState = gORWaitForTheValue_c; 
           }
           ClearEvents();
         }
    break;
    case gORWaitForTheValue_c:
         if(evDataFromUART)
         {
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData)){
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               oSRRState = gORStateStart_c;              
             }
           }else if((4 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             bBackFlag = TRUE;
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u16RegValue = HexString2Dec16(au8RxString);             
             (void)Radio_WriteRegisterBlocking(u8RegAddress, u16RegValue);
             (void)Uart_BlockingStringTx("\r\n Register overridden \r\n", gDefaultUartPort_c);
             u8Index = 0;
             oSRRState = gORStateStart_c; 
           }
           ClearEvents();
         }
    break;
    default:
    break;
  }
  return bBackFlag;  
}

/**************************************************************************************/
bool_t ReadSpiRadioRegisters(void)
{
  bool_t bBackFlag = FALSE;
  static uint8_t au8RxString[5];
  static uint8_t u8Index;
  static uint8_t u8RegAddress;
  static uint16_t u16RegValue;
  
  switch(rSRRState)
  {
    case gRRStateInit_c:
         (void)Uart_BlockingStringTx("\f\r\rRead SPI Radio Registers\r\n", gDefaultUartPort_c);
         (void)Uart_BlockingStringTx("\r\n-Press [p] Previous Menu\r\n", gDefaultUartPort_c);
         shortCutsEnabled = FALSE;   
         rSRRState = gRRStateStart_c;
    break;
    case gRRStateStart_c:
         (void)Uart_BlockingStringTx("\r\n -write the Register address in Hex and [enter]: 0x", gDefaultUartPort_c);
         u8Index = 0;
         rSRRState = gRRWaitForTheAddress_c; 
    break;
    case gRRWaitForTheAddress_c:
         if(evDataFromUART){
           if((!isAsciiHex(gu8UartData)) && ('\r' != gu8UartData))
           {
             if('p' == gu8UartData){ 
               bBackFlag = TRUE;
             }else{
               (void)Uart_BlockingStringTx("\r\n -Invalid Character!! ", gDefaultUartPort_c);
               rSRRState = gRRStateStart_c;              
             }
           }else if((2 == u8Index) && ('\r' != gu8UartData)){ 
             (void)Uart_BlockingStringTx("\r\n -Value out of Range!! ", gDefaultUartPort_c);
             rSRRState = gRRStateStart_c; 
           }else if(isAsciiHex(gu8UartData)){
             au8RxString[u8Index++] = gu8UartData;
           }else{
             au8RxString[u8Index] = 0;
             u8RegAddress = (uint8_t)(HexString2Dec16(au8RxString) & 0x00FF);
             (void)Radio_ReadRegisterBlocking(u8RegAddress, &u16RegValue); 
             (void)Uart_BlockingStringTx("\r\n -Register value : 0x", gDefaultUartPort_c);
             PrintByteOnHexFormatBlocking((uint8_t)(u16RegValue>>8), FALSE, gDefaultUartPort_c);
             PrintByteOnHexFormatBlocking((uint8_t)(u16RegValue), FALSE, gDefaultUartPort_c);
             (void)Uart_BlockingStringTx("\r\n", gDefaultUartPort_c);
             rSRRState = gRRStateStart_c; 
           }
           ClearEvents();
         }
    break;
    default:
    break;
  }
  return bBackFlag;  
} 

#endif


/**************************************************************************************/
bool_t ManualContinuousIdle(void)
{
  (void)MLMETestMode(gTestModeForceIdle_c);
  contTestRunning = gTestModeForceIdle_c;
  LEDsStop();
  return TRUE;
}

/**************************************************************************************/
bool_t ManualContinuousPrbsTransmission(void)
{
   (void)MLMETestMode(gTestModePRBS9_c);
   contTestRunning = gTestModePRBS9_c;
   LEDsBlinkLED4();
   return TRUE;
}

/**************************************************************************************/
bool_t ManualContinuousModulatedTransmission(void)
{
   (void)MLMETestMode(gTestModeContinuousTxModulated_c);
   contTestRunning = gTestModeContinuousTxModulated_c;
   LEDsBlinkLED4();
   return TRUE;
}

/**************************************************************************************/
bool_t ManualContinuousUnmodulatedTransmission(void)
{
   (void)MLMETestMode(gTestModeContinuousTxUnmodulated_c);
   contTestRunning = gTestModeContinuousTxUnmodulated_c;
   LEDsBlinkLED4();
   return TRUE;
}

/**************************************************************************************/
bool_t ManualContinuousReception(void)
{
   static bool_t ContRxOn = FALSE;
   static uint16_t PacketCount=0;
   bool_t bBackFlag = FALSE;
   uint8_t u8NewPacketString[] = {"Pkt: ####,  ### "};
   uint8_t u8Index;
   uint8_t u8TempLQIvalue;

   if(!ContRxOn){
     (void)Lcd_WriteStringBlocking(1,"  Cont. Rx On   ");
     (void)Lcd_WriteStringBlocking(2,"                ");      
     MLMESetPromiscuousMode(TRUE);
     bRxDone = FALSE;
     gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
     (void)MLMERXEnableRequest(gAppRxPacket, 0);
     ContRxOn = TRUE;
   }else{
     if(bRxDone){
       if(rxSuccessStatus_c == gAppRxPacket->rxStatus){
         PacketCount++;
         int2BCD(PacketCount,&u8NewPacketString[4]);
         for(u8Index=4;u8Index<9;u8Index++) u8NewPacketString[u8Index] += '0';
         (void)MLMELinkQuality(&u8TempLQIvalue);
         int2BCD((uint16_t)(u8TempLQIvalue),&u8NewPacketString[10]);
         u8NewPacketString[10] = ' ';
         u8NewPacketString[11] = '-';
         for(u8Index=12;u8Index<15;u8Index++) u8NewPacketString[u8Index] += '0';
         (void)Lcd_WriteStringBlocking(2,u8NewPacketString);      
         Led_Toggle(gLED4_c);         
       }
       bRxDone = FALSE;
       gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
       (void)MLMERXEnableRequest(gAppRxPacket, 0);
     }
     if(evSwitch4){
       PacketCount = 0;
       (void)MLMERXDisableRequest();
       ContRxOn=FALSE;
       MLMESetPromiscuousMode(FALSE);
       bBackFlag = TRUE;
       Led_Off(gLED4_c);
       ClearEvents();     
     }
   }

   return bBackFlag;
}

/**************************************************************************************/
bool_t ManualContinuousScan(void)
{
   static bool_t ContScanOn = FALSE;
   bool_t bBackFlag = FALSE;
   uint8_t u8ScanDoneString[] = {"Best Chann: 0x##"};

   if(!ContScanOn){
     (void)Lcd_WriteStringBlocking(1,"Continuous Scan ");
     (void)Lcd_WriteStringBlocking(2,"                ");      
     bScanDone = FALSE;
     (void)MLMEScanRequest(0xFFFF, gScanModeED_c, au8ScanResults);
     ContScanOn = TRUE;
   }else{
     if(bScanDone){
       u8ScanDoneString[14] = HexToAscii(bestChannel>>4);
       u8ScanDoneString[15] = HexToAscii(bestChannel);
       (void)Lcd_WriteStringBlocking(2,u8ScanDoneString);
       #if(FALSE == gLcdSupported_d)
         LEDsFlashValue(bestChannel-gChannel11_c,5);
       #endif                   
       if(evSwitch4){
         ContScanOn=FALSE;
         bBackFlag = TRUE;
         Led_Off(gLED4_c);         
         ClearEvents();      
       }else{
         DelayMs(100);DelayMs(100);DelayMs(50);
         bScanDone = FALSE;
         (void)MLMEScanRequest(0xFFFF, gScanModeED_c, au8ScanResults);
       }
     }
   }

   return bBackFlag;
}

/**************************************************************************************/
bool_t ManualContinuousEnergyDetect(void)
{
   static bool_t ContEdOn = FALSE;
   bool_t bBackFlag = FALSE;
   uint8_t u8EdDoneString[] = {"  Energy: -###  "};
   uint8_t u8Index;
   uint8_t u8TempEnergyValue;

   if(!ContEdOn){
     (void)Lcd_WriteStringBlocking(1,"Energy Detect   ");
     (void)Lcd_WriteStringBlocking(2,"                ");      
     ContEdOn = TRUE;
   }else{
     DelayMs(100);DelayMs(100);
     (void)MLMEEnergyDetect(&u8TempEnergyValue);
     int2BCD((uint16_t)u8TempEnergyValue,&u8EdDoneString[9]);
     u8EdDoneString[9] = ' ';
     u8EdDoneString[10] = '-';
     for(u8Index=11;u8Index<14;u8Index++) u8EdDoneString[u8Index] += '0';
     (void)Lcd_WriteStringBlocking(2,u8EdDoneString); 
     #if(FALSE == gLcdSupported_d)
       LEDs_PrintEnergyLevelOnLEDs(u8TempEnergyValue);
     #endif                   
     if(evSwitch4){
       ContEdOn=FALSE;
       bBackFlag = TRUE;
       ClearEvents();    
     }     
   }

   return bBackFlag;
}

/**************************************************************************************/
void DisplaySelectedState(ConnectivityStates_t state)
{
  if((state >= gConnSetChannelState_c) && (state <= gConnSetTrimFineState_c)){
    (void)Lcd_WriteStringBlocking(1,cu8SelectTags[state-gConnSetChannelState_c]);
    (void)Lcd_WriteStringBlocking(2,"Press SW2 or SW3");      
  }
}

/**************************************************************************************/
void SetNewChannel(channels_t newChannel)
{
  uint8_t u8ChannelString[]={" Channel: 0x##  "};

  (void)MLMETestMode(gTestModeForceIdle_c);
  (void)MLMESetChannelRequest(newChannel);
  (void)MLMETestMode(contTestRunning);
  
  u8ChannelString[12] = HexToAscii(newChannel>>4);
  u8ChannelString[13] = HexToAscii(newChannel);
  (void)Lcd_WriteStringBlocking(2,u8ChannelString);      
  #if(FALSE == gLcdSupported_d)
   LEDsFlashValue(testChannel-gChannel11_c,5);
  #endif 
}

/**************************************************************************************/
void SetNewPower(uint8_t newPower)
{
  uint8_t u8PowerString[]={"  Power: 0x##   "};

  (void)MLMETestMode(gTestModeForceIdle_c);
  (void)MLMEPAOutputAdjust(newPower);  
  (void)MLMETestMode(contTestRunning);
  
  u8PowerString[11] = HexToAscii(newPower>>4);
  u8PowerString[12] = HexToAscii(newPower);
  (void)Lcd_WriteStringBlocking(2,u8PowerString);  
  
  #if(FALSE == gLcdSupported_d)
   LEDsFlashValue(testPower,5);
  #endif 
}

/**************************************************************************************/
void SetNewTrim(uint8_t newTrim)
{
  uint8_t u8TrimString[]={"Crstl trim: 0x##"};
  
  (void)MLMETestMode(gTestModeForceIdle_c);
  (void)MLMEXtalAdjust(newTrim);  
  (void)MLMETestMode(contTestRunning);

  u8TrimString[14] = HexToAscii(newTrim>>4);
  u8TrimString[15] = HexToAscii(newTrim);
  (void)Lcd_WriteStringBlocking(2,u8TrimString);      

  #if(FALSE == gLcdSupported_d)
   LEDsFlashValue(testTrimmValue>>4,5);
  #endif 

}

/**************************************************************************************/
void DisplayNewTxTest(txTests_t newTxTest)
{
  (void)Lcd_WriteStringBlocking(2,cu8TxTestTags[newTxTest]);      
}

/**************************************************************************************/
void DisplayNewRxTest(RxTests_t newRxTest)
{
  (void)Lcd_WriteStringBlocking(2,cu8RxTestTags[newRxTest]);      
}


/**************************************************************************************/
void PrintTestParameters(bool_t bEraseLine)
{
  uint8_t u8lineLen = 63;
  uint8_t u8Index;
 
  if(bEraseLine){
    for(u8Index = 0;u8Index<u8lineLen;u8Index++){
      (void)Uart_BlockingByteTx('\b', gDefaultUartPort_c);
    }
  }
  
  (void)Uart_BlockingStringTx("Mode ", gDefaultUartPort_c);
  if(mTxOperation_c == testOpMode){
    (void)Uart_BlockingStringTx("Tx", gDefaultUartPort_c);
  }else{
    (void)Uart_BlockingStringTx("Rx", gDefaultUartPort_c);
  }
  (void)Uart_BlockingStringTx(", Channel ", gDefaultUartPort_c);
  PrintWordOnDecimalFormatBlocking((uint16_t)testChannel,2, FALSE, gDefaultUartPort_c);
  (void)Uart_BlockingStringTx(", Power ", gDefaultUartPort_c);
  PrintWordOnDecimalFormatBlocking((uint16_t)testPower,2, FALSE, gDefaultUartPort_c);
  (void)Uart_BlockingStringTx(", Crystal Trim ", gDefaultUartPort_c);
  PrintWordOnDecimalFormatBlocking((uint16_t)testTrimmValue,3, FALSE, gDefaultUartPort_c);
  (void)Uart_BlockingStringTx(", Payload ", gDefaultUartPort_c);
  PrintWordOnDecimalFormatBlocking((uint16_t)testPayloadLen,3, FALSE, gDefaultUartPort_c);
  (void)Uart_BlockingStringTx(" >", gDefaultUartPort_c);
}

/**************************************************************************************/
void ClearEvents(void)
{
  evSwitch1 = FALSE;
  evSwitch2 = FALSE;
  evSwitch3 = FALSE;
  evSwitch4 = FALSE;
  evDataFromUART = FALSE;
}

/**************************************************************************************/
void SetRadioRxOnTimeOut15ms(void)
{
  bRxDone = FALSE;
  gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
  (void)MLMERXEnableRequest(gAppRxPacket, 3750);
}

/**************************************************************************************/
void SetRadioRxOnNoTimeOut(void)
{
  bRxDone = FALSE;
  gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
  (void)MLMERXEnableRequest(gAppRxPacket, 0);
}

/**************************************************************************************/
void PrintPerRxFinalLine(uint16_t u16Received, uint16_t u16Total)
{
  (void)Uart_BlockingStringTx("Received ", gDefaultUartPort_c);
  PrintWordOnDecimalFormatBlocking(u16Received, 1, FALSE, gDefaultUartPort_c);
  (void)Uart_BlockingStringTx(" of ", gDefaultUartPort_c);
  PrintWordOnDecimalFormatBlocking(u16Total, 1, FALSE, gDefaultUartPort_c);
  (void)Uart_BlockingStringTx(" packets transmitted \r\n", gDefaultUartPort_c);
  (void)Uart_BlockingStringTx("\r\n Press [enter] to go back to the Per Rx test menu", gDefaultUartPort_c);
}

/* Place it in NON_BANKED memory */
#ifdef MEMORY_MODEL_BANKED
#pragma CODE_SEG __NEAR_SEG NON_BANKED
#else
#pragma CODE_SEG DEFAULT
#endif /* MEMORY_MODEL_BANKED */

/************************************************************************************
* User's Callbacks
************************************************************************************/

/* Place your callbacks here */


/************************************************************************************
* TouchpadCallback
* 
*
*
************************************************************************************/
#if gTargetBoard_c == gMc1323xRcm_c
void TouchpadCallback(touchpadEvent_t * event)
{
  uint8_t u8mmsX,u8mmsY;
  
  switch(event->EventType)
  {
    case gTouchpadBusError_c:
    break;
    case gTouchpadGpioEvent_c:
    break;
    case gTouchpadFingerPositionEvent_c:
    break;
    case gTouchpadPinchGestureEvent_c:
    break;
    case gTouchpadFlickGestureEvent_c:  
         u8mmsX = event->EventDataReport.flickReport.i8XFlickDistance;
         if(0x80 & u8mmsX){
           u8mmsX &= 0x7F;
           u8mmsX--;
           u8mmsX = ~(u8mmsX); 
         }         
         u8mmsY = event->EventDataReport.flickReport.i8YFlickDistance;
         if(0x80 & u8mmsY){
           u8mmsY &= 0x7F;
           u8mmsY--;
           u8mmsY = ~(u8mmsY); 
         }
         
         if((u8mmsX & 0x7F) > (u8mmsY & 0x7F)){
           if(0x80 & u8mmsX){ //Flick rigth
             evSwitch1 = TRUE;
           }else{             //Flick left
            
           }
         }else{
           if(0x80 & u8mmsX){ //Flick up
             evSwitch2 = TRUE;
           }else{             //Flick down
             evSwitch3 = TRUE;            
           }
         }
    break;
    case gTouchpadEarlyTapGestureEvent_c:
    break;
    case gTouchpadDoubleTapGestureEvent_c:
    break;
    case gTouchpadTapAndHoldGestureEvent_c:
    break;
    case gTouchpadSingleTapGestureEvent_c:
         evSwitch4 = TRUE;   //TAP
    break;
    case gTouchpadDevStatusEvent_c:
    break;
    case gTouchpadFlashEvent_c:
    break;
    default:
    break;
  }
}
#endif


/************************************************************************************
* UartTxCallback
* 
*
*
************************************************************************************/
void UartTxCallback(void)
{
    bUartTxDone = TRUE;  
}

/************************************************************************************
* UartRxCallback
* 
*
*
************************************************************************************/
void UartRxCallback(uint8_t u8UartFlags)
{
  uint8_t iByteNumber;
  (void)u8UartFlags;
  
  if(gManualUI_c != selectedUI){
    iByteNumber = 1;  
    (void)Uart_GetBytesFromRxBuffer(&gu8UartData, &iByteNumber, gDefaultUartPort_c);
    if(shortCutsEnabled){
      ShortCutsParser(gu8UartData);  
    }else{
      evDataFromUART = TRUE;
    }
  }
}

#if (TRUE == gKeyboardSupported_d) || (TRUE == gTouchpadSupported_d) || (TRUE == gKbiSupported_d)
/************************************************************************************
* KbiCallback
* 
*  This function should be set as the Kbi callback function in Kbi_Init
*
************************************************************************************/
#if gTargetBoard_c == gMc1323xRcm_c || gTargetBoard_c == gMc1323xRem_c
void KbiCallback(kbiPressed_t pressedKey)
{  
  (void)pressedKey;
  if(gKbiPressedKey0_c == pressedKey || gKbiPressedKey1_c == pressedKey || gKbiPressedKey2_c == pressedKey || gKbiPressedKey3_c == pressedKey \
     || gKbiPressedKey4_c == pressedKey || gKbiPressedKey5_c == pressedKey)
  {
     Keyboard_KbiEvent(pressedKey);
  }
  else if (gKbiPressedKey6_c == pressedKey)
  {
     #if gTargetBoard_c == gMc1323xRcm_c
     Touchpad_EventHandler();
     #endif
  }
   
} 
#else
void KbiCallback(kbiPressed_t pressedKey)
{  
  if(gSerialUI_c != selectedUI){
    switch(pressedKey)
    {
      case gKbiPressedKey0_c:
        evSwitch1 = TRUE;
      break;
      case gKbiPressedKey1_c:
        evSwitch2 = TRUE;
      break;
      case gKbiPressedKey2_c:
        evSwitch3 = TRUE;
      break;
      case gKbiPressedKey3_c:
        evSwitch4 = TRUE;
      break;
      default:
      break;
    }
  }
}


#endif 
      
#endif

/************************************************************************************
* KeyboardCallback
* 
*
*
************************************************************************************/
#if (gTargetBoard_c == gMc1323xRcm_c) || (gTargetBoard_c == gMc1323xRem_c)

void KeyboardCallback(keyboardButton_t keyPressed)
{
//  if(gSerialUI_c != selectedUI){
    switch(keyPressed)
    {
      case gSw1_c:
        evSwitch1 = TRUE;
      break;

      case gSw2_c:
        evSwitch2 = TRUE;
      break;

      case gSw3_c:
        evSwitch3 = TRUE;
      break;

      case gSw4_c:
        evSwitch4 = TRUE;
      break;

      default:
      break;
    }
//  }
}
#endif

/************************************************************************************
* LCDCallback
* 
*
*
************************************************************************************/
void LCDCallback(lcdErrors_t lcdError)
{
  (void)lcdError;
}

/************************************************************************************
* SMAC Callbacks
************************************************************************************/

/************************************************************************************
* MCPSDataConfirm
* 
*
*
************************************************************************************/
void MCPSDataConfirm(txStatus_t TransmissionResult)
{  
    bTxDone = TRUE;
}
 

/************************************************************************************
* MCPSDataIndication
* 
*
*
************************************************************************************/
void MCPSDataIndication(rxPacket_t *gsRxPacket)
{  
  bRxDone = TRUE;
}


/************************************************************************************
* MLMEScanConfirm
* 
*
*
************************************************************************************/
void MLMEScanConfirm(channels_t ClearestChann)
{
  bestChannel = ClearestChann; 
  bScanDone = TRUE;
}

/************************************************************************************
* MLMEResetIndication
* 
*
*
************************************************************************************/
void MLMEResetIndication(void)
{
  
}

/************************************************************************************
* MLMEWakeConfirm
* 
*
*
************************************************************************************/
void MLMEWakeConfirm(void)
{
  
}

/**************************************************************************************/
void ShortCutsParser(uint8_t u8UartData)
{
  evTestParameters = TRUE;
  evDataFromUART = FALSE;
  
//  if(!messengerOn)
  {
	    switch(u8UartData){
    case 't':
      testOpMode = mTxOperation_c;
    break;
    case 'r':
      testOpMode = mRxOperation_c;
    break;
    case 'q': 
      testChannel++;
      if(gChannel26_c < testChannel){
        testChannel = gChannel11_c;
      }
    break;
    case 'w':
      testChannel--;
      if(gChannel11_c > testChannel){
        testChannel = gChannel26_c;
      }
    break;
    case 'a':
      testPower++;
      if(gMaxOutputPower_c < testPower){
        testPower = 0;
      }
    break;
    case 's':
      testPower--;
      if(0xFF == testPower){
        testPower = gMaxOutputPower_c;
      }
    break;
    case 'z':
      testTrimmValue++;
    break;
    case 'x':
      testTrimmValue--;
    break;
    case 'n':
      testPayloadLen++;
      if(gMaxSmacSDULenght_c < testPayloadLen){
        testPayloadLen = 1;
      }    
    break;
    case 'm':
      testPayloadLen--;
      if(1 > testPayloadLen){
        testPayloadLen = gMaxSmacSDULenght_c;
      }
    break;
	/////////////////////////////////////////////////////////////
    case 'S':
    	gAppTxPacket->u8DataLength = 27;
      	MemoryCpy(&(gAppTxPacket->smacPdu.u8Data), "Equipo 9, Tecla Presionada",27);
	  	Led_Toggle(gLED1_c);
	  	DelayMs(200);
	  	Led_Toggle(gLED1_c);
      	(void)MCPSDataRequest(gAppTxPacket);
    break;
	//////////////////////////////////////////////////////////////

    
	/////////////////////////////////////////////////////////////
    case 'R':
        cTxRxState = gCTxRxStateInit_c;
        connState = gConnContinuousTxRxState_c;
        (void)MLMETestMode(gTestModeForceIdle_c);
        contTestRunning = gTestModeForceIdle_c;
        (void)Uart_BlockingStringTx("\f\r\nPress [p] to stop receiving promiscuous packets \r\n", gDefaultUartPort_c);
        MLMESetPromiscuousMode(TRUE);
        bRxDone = FALSE;
        gAppRxPacket->u8MaxDataLength = gMaxSmacSDULenght_c;
        (void)MLMERXEnableRequest(gAppRxPacket, 0);
        cTxRxState = gCTxRxStateRunnigRxTest_c;
    break;
	///////////////////////////////////////////////////////////
      	
      	
    default:
      evDataFromUART = TRUE;
      evTestParameters = FALSE;
    break;
  }
  }

}


#pragma CODE_SEG DEFAULT





