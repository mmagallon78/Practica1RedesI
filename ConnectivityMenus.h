/*****************************************************************************
* Connectivity test demo Menus header file.
* 
* Copyright (c) 2009, Freescale, Inc. All rights reserved.
*
* 
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
*****************************************************************************/

#include "EmbeddedTypes.h"          /*Include special data types*/             
#include "Timer_Interface.h"        /*Include all Timer functionality*/
#include "LED_Interface.h"          /*Include all LED functionality*/
#include "Utilities_Interface.h"    /*Include Utilities*/
#include "UART_Interface.h"         /*Include UART functionality*/  

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
extern uint8_t * const cu8FreescaleLogo[];
extern uint8_t * const cu8MainMenu[]; 
extern uint8_t * const cu8ShortCutsBar[];
extern uint8_t * const cu8ContinuousTestMenu[]; 
extern uint8_t * const cu8PerTxTestMenu[]; 
extern uint8_t * const cu8PerRxTestMenu[];
extern uint8_t * const cu8RangeTxTestMenu[];
extern uint8_t * const cu8RangeRxTestMenu[];
extern uint8_t * const cu8RadioRegistersEditMenu[];
extern uint8_t * const cu8ContinuousTestTags[]; 
extern uint8_t * const cu8SelectTags[];
extern uint8_t * const cu8TxTestTags[];
extern uint8_t * const cu8RxTestTags[];
extern uint8_t * const cu8SendReceivePacketsTx[];

 
/************************************************************************************
*************************************************************************************
* Module macros
*************************************************************************************
************************************************************************************/
#define gLEDsToggleTime_c (0x7933)      //250ms
#define isAsciiHex(Data) (((Data >= '0') && (Data <= '9')) \
                       || ((Data >= 'A') && (Data <= 'F')) \
                       || ((Data >= 'a') && (Data <= 'f'))) 
                       
#if defined(gMc1323xPlatform_d)
 #define gLEDsTimmer_c         gTmr4_c               
 #define gLEDsTimmerChannel_c  gTmrChannel0_c                      
#elif defined(gMcs08qe128Platform_d)
 #define gLEDsTimmer_c         gTmr1_c               
 #define gLEDsTimmerChannel_c  gTmrChannel0_c                      
#elif defined(gMcs08Gt60Platform_d)
 #define gLEDsTimmer_c         gTmr2_c               
 #define gLEDsTimmerChannel_c  gTmrChannel0_c                      
#endif

/*This defines sets the different RF energy triggers to turn ON/OFF the LEDs when
  Displaying energy levels*/
#define mEnergyTrigger1_c 0
#define mEnergyTrigger2_c 100
#define mEnergyTrigger3_c 150
#define mEnergyTrigger4_c 200

/************************************************************************************
*************************************************************************************
* Module type definitions
*************************************************************************************
************************************************************************************/

typedef enum ConnectivityTestUIs_tag
{
  gNoUI_c,
  gSerialUI_c,
  gManualUI_c,
  gMaxUI_c
}ConnectivityTestUIs_t;


typedef enum ConnectivityStates_tag
{
  gConnInitState_c,
  gConnIdleState_c,
  gConnSelectTest_c,
  gConnContinuousTxRxState_c,
  gConnPerState_c,
  gConnRangeState_c,
  gConnRegEditState_c,
  /////////////////
  gConnSendReceivePacketsTxState_c,
  gConnSendReceivePacketsRxState_c,
  ////////////////////
  gConnSetChannelState_c,
  gConnSetPowerState_c,
  gConnSetTxTestState_c,
  gConnSetRxTestState_c,
  gConnSetTrimCoarseState_c,
  gConnSetTrimFineState_c,
  gConnContinuousTxState_c,
  gConnPerTxState_c,
  gConnRangeTxState_c,
  gConnContinuousRxState_c,
  gConnPerRxState_c,
  gConnRangeRxState_c
}ConnectivityStates_t;

typedef enum ContinuousTxRxTestStates_tag 
{
  gCTxRxStateInit_c,
  gCTxRxStateIdle_c,
  gCTxRxStateSelectTest_c,
  gCTxRxStateRunnigEdTest_c,
  gCTxRxStateRunnigScanTest_c,
  gCTxRxStateRunnigRxTest_c,  
  gCTxRxMaxState_c
}ContinuousTxRxTestStates_t;

typedef enum PerTxStates_tag 
{
  gPerTxStateInit_c,
  gPerTxStateIdle_c,
  gPerTxStateSelectPacketNum_c,
  gPerTxStateStartTest_c,
  gPerTxStateRunningTest_c,
  gPerTxStateSendingLastFrames_c,
  gPerTxStateMaxState_c
}PerTxStates_t;

typedef enum PerRxStates_tag 
{
  gPerRxStateInit_c,
  gPerRxStateIdle_c,
  gPerRxWaitStartTest_c,
  gPerRxStateStartTest_c,
  gPerRxStateRunningTest_c,
  gPerRxStateReceivingLastFrames_c,
  gPerrxStateMaxState_c
}PerRxStates_t;

typedef enum RangeTxStates_tag 
{
  gRangeTxStateInit_c,
  gRangeTxStateIdle_c,
  gRangeTxWaitStartTest_c,
  gRangeTxStateStartTest_c,
  gRangeTxStateRunningTest_c,
  gRangeTxStatePrintTestResults_c,
  gRangeTxStateMaxState_c
}RangeTxStates_t;

typedef enum RangeRxStates_tag 
{
  gRangeRxStateInit_c,
  gRangeRxStateIdle_c,
  gRangeRxWaitStartTest_c,
  gRangeRxStateStartTest_c,
  gRangeRxStateRunningTest_c,
  gRangeRxStatePrintTestResults_c,
  gRangeRxStateMaxState_c
}RangeRxStates_t;

typedef enum EditRegsStates_tag 
{
  gERStateInit_c,
  gERWaitSelection_c,
  gERStateOverrideDirectReg_c,
  gERStateOverrideIndirectReg_c,
  gERStateReadDirectReg_c,
  gERStateReadIndirectReg_c,
  gERStateOverrideRadioSpiReg_c,
  gERStateReadRadioSpiReg_c,
  gERStateMaxState_c
}EditRegsStates_t;

typedef enum ODRStates_tag 
{
  gODRStateInit_c,
  gODRStateStart_c,
  gODRWaitForTheAddress_c,
  gODRWaitForTheValue_c,  
  gODRStateIdle_c,
  gODRStateMaxState_c
}ODRStates_t;

typedef enum OIRStates_tag 
{
  gOIRStateInit_c,
  gOIRStateStart_c,
  gOIRWaitForTheAddress_c,
  gOIRWaitForTheValue_c,  
  gOIRStateIdle_c,
  gOIRStateMaxState_c
}OIRStates_t;

typedef enum RDRStates_tag 
{
  gRDRStateInit_c,
  gRDRStateStart_c,
  gRDRWaitForTheAddress_c,
  gRDRStateMaxState_c
}RDRStates_t;

typedef enum RIRStates_tag 
{
  gRIRStateInit_c,
  gRIRStateStart_c,
  gRIRWaitForTheAddress_c,
  gRIRStateMaxState_c
}RIRStates_t;

typedef enum overrideRegistersStates_tag 
{
  gORStateInit_c,
  gORStateStart_c,
  gORWaitForTheAddress_c,
  gORWaitForTheValue_c,  
  gORStateIdle_c,
  gORStateMaxState_c
}overrideRegistersStates_t;

typedef enum readRegistersStates_tag
{
  gRRStateInit_c,
  gRRStateStart_c,
  gRRWaitForTheAddress_c,
  gRRStateMaxState_c
}readRegistersStates_t;

typedef enum{
  gPacketErrorRateTx_c,
  gRangeTx_c,  
  gManualContinuousIdle_c,
  gManualContinuousPrbsTransmission_c,
  gManualContinuousModulatedTransmission_c,
  gManualContinuousUnmodulatedTransmission_c,
  gMaxTxTest_c
}txTests_t;

typedef enum{
  gPacketErrorRateRx_c,
  gRangeRx_c, 
  gManualContinuousReception_c,
  gManualContinuousScan_c,
  gManualContinuousEnergyDetect_c,
  gMaxRxTest_c
}RxTests_t;

typedef enum LEDsStates_tag
{
  mLEDsStopped_c,
  mLEDsBlinking_c,
  mLED4Blinking_c,
  mLEDsRunning_c  
}LEDsStates_t;

typedef enum SendReceivePacketsTx_tag 
{
  gSendReceivePacketsTxInit_c,
  gSendReceivePacketsTxSelectTest_c,
  gSendReceivePacketsTxSinglePacket_c,
  gSendReceivePacketsTxSizeVariablePacket_c,
  gSendReceivePacketsTxMessenger_c
}SendReceivePacketsTx_t;

typedef enum SendReceivePacketsRx_tag 
{
  gSendReceivePacketsRxInit_c,
  gSendReceivePacketsRxSinglePacket_c,
  gSendReceivePacketsRxSizeVariablePacket_c,
  gSendReceivePacketsRxMessenger_c
}SendReceivePacketsRx_t;


/************************************************************************************
*************************************************************************************
* Interface functions prototypes
*************************************************************************************
************************************************************************************/
void InitLEDs(void);
void LEDsBlink(void);
void LEDsBlinkLED4(void);
void LEDsRun(void);
void LEDsStop(void);
void LEDsFlashValue(uint8_t u8Value, uint8_t u8Time100ms);
void LEDs_PrintValue3LEDs(uint8_t LedHexValue);
void LEDs_PrintEnergyLevelOnLEDs(uint8_t u8EnergyLevel);
void PrintMenu(uint8_t * const pu8Menu[], uartPortNumber_t port);

