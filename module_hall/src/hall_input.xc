/**
 * Module:  module_hall
 * Version: 1v0alpha2

 * orgler@tin.it synapticon 01/2013
  *
 **/                                   
#include "hall_input.h"
#include <stdlib.h>
#include <print.h>
#include <stdint.h>
#include "refclk.h"
#include "dc_motor_config.h"
#include <xscope.h>

extern  out port p_ifm_ext_d3;
extern  port p_ifm_ext_d0;

// uart 115200 Baud 1 Bit = 8.6�sec iByte 87�sec
// 32  32Bit Werte = 128Bytes -> 5,568msec

//========== global variables for encoder ==========
int iEncoderDirection 		=0;
int iCountEncoderPulses		=0;
int iNrEncoderPulses		=400;
int iEncoderCountMicroSeconds=0;
int iEncoderPeriodMicroSeconds=0;
int iEncoderPeriodNext;
int iEncoderSpeed=0;

int iHallDirection    = 0;
int iCountHallPulses  = 0;
int iNrHallPulses     = 1;
int iHallCountMicroSeconds=0;
int iHallPeriodMicroSeconds=0;
int iHallPeriodNext;
int iHallSpeed=0;




void run_uart(chanend c_motvalue, clock clk1)
{
timer tx;
unsigned ts;					// newest timestamp
int iStep=0,iStepReturn;

unsigned uMotorValues[32];
unsigned uMotorCommand[16];
unsigned uMotorParameter[32];

unsigned uSendValue;
unsigned uTemp;
unsigned iIndex2;
unsigned iIndex1;
unsigned iRxIndex;
unsigned char cxRxBuf[512];
int iBitCount;
unsigned char cRxByte;

unsigned char cMaskByte;
unsigned char cSendByte;
unsigned char cStartByte=0x7E;  // tilde
unsigned char cBit;
unsigned char cStopBit;
unsigned char CRC1,CRC2;
unsigned char cTxFlag;

#define defStartBit 0
#define defStopBit  1
#define defBitOne   1
#define defBitZero  0

#define defStepTxCRC   70
#define defStepTxOnly  71

tx :> ts;  // first value

	while(1)
	{

	 if(iStep > 52)	  tx when timerafter(ts + 1084) :> ts;
     if(iStep == 1)   tx when timerafter(ts + 125) :>  ts;
//	 if(iStep > 1 && iStep < 50)tx when timerafter(ts + 1082 * 2) :> ts;
//	 p_ifm_ext_d3 <: 0;


     switch(iStep)
     {
     case 0: iStep++;
	 	 	 configure_in_port(p_ifm_ext_d0, clk1);
	 	 	 iRxIndex = 0;
	 	 	 cBit 	  = 0;
             break;

     case 1: p_ifm_ext_d0 :> cBit;
    	     if(cBit == 0 ){
     	 	 iStep++;
    	     //p_ifm_ext_d3 <: 1;
     	 	 cStopBit = 0;
    	     tx when timerafter(ts + 1082) :> ts;
    	     }
    	     break;

     case 2:  p_ifm_ext_d0 :> cBit;
     	 	 if(cBit == 0 ){
     	 		 iStep++;
     	 		// p_ifm_ext_d3 <: 1;
     	 		 tx when timerafter(ts + 1080*2) :> ts;
     	 	 }
     	 	 else iStep = 1;
     	 	 iBitCount  = 8;
     	 	 cRxByte    = 0;
    	     break;

     case 3:  p_ifm_ext_d0 :> cBit;
              cRxByte  = cRxByte >> 1;
     	 	  if(cBit) cRxByte |= 0x80;
 	 	      //p_ifm_ext_d3 <: 1;
	 		  tx when timerafter(ts + 1080*2) :> ts;
	 	      //iBitCount--;
	 	      if(--iBitCount==0)iStep++;
	 	      break;

     case 4:  p_ifm_ext_d0 :> cStopBit;
     	 	  iStep++;
     	 	  if(cStopBit==0){iStep=1; iRxIndex=0;}
              break;
     case 5:  iStep++;
    	      if(iRxIndex==0)
     	 	  {
    	 	  if(cRxByte != 0x7E) iStep=1;
     	 	  }
    	      break;
     case 6:  cxRxBuf[iRxIndex++]= cRxByte;
              iStep=1;
              if(iRxIndex == cxRxBuf[2])
              iStep=7;
              break;

     case 7:  CRC1 = cxRxBuf[0];
     	 	  CRC2 = cxRxBuf[0];
     	 	  iIndex1 = 1;
     	 	  while(iIndex1 < (cxRxBuf[2]-2))
     	 	  {
     	 		 CRC1 ^= cxRxBuf[iIndex1];
     	 		 CRC2 += cxRxBuf[iIndex1];
     	 		 iIndex1++;
     	 	  }
     	 	  CRC2 += cxRxBuf[iIndex1];
    	      switch(cxRxBuf[1])
              {
              case 'V': iStep = 40; break;    // read motor values from XMOS ==> PC
              case 'C': iStep = 10; break;    // write motor commands from PC => XMOS and readout motor values
              case 'P': iStep = 20; break;    // read parameter from XMOS ==> PC
              case 'X': iStep = 30; break;    // write parameter to XMOS
              default: iStep=0; break;
              }
              break;


//======================== write commands to xmos =================================
 	  case 10:  iIndex1=0;
 	            iIndex2=4;
 	            while(iIndex1 < 8)
 	            {
 	            uTemp =  cxRxBuf[iIndex2++];  uTemp = uTemp << 8;
 	            uTemp += cxRxBuf[iIndex2++];  uTemp = uTemp << 8;
 	            uTemp += cxRxBuf[iIndex2++];  uTemp = uTemp << 8;
 	            uTemp += cxRxBuf[iIndex2++];
 	            uMotorCommand[iIndex1++] = uTemp;
 	            }
 	            iStep++;
 	            break;


 	  case 11:  iIndex1=0;
 	  	  	  	uMotorCommand[7] = 1;
 	            while(iIndex1 <= 4)   // last 4, 5, 6, 7,
 	            {
 		        c_motvalue <:iIndex1;
 	  	  	  	c_motvalue <: uMotorCommand[iIndex1];
 	  	  	  	c_motvalue <: uMotorCommand[iIndex1+1];
 	  	  	  	c_motvalue <: uMotorCommand[iIndex1+2];
 	  	  	  	c_motvalue <: uMotorCommand[iIndex1+3];
 	  	  	  	iIndex1 += 4;
 	            }
           	   	iStep=40;  // next step readout motor values
 	            break;
 /*
						else if(cmd2 >= 32 && cmd2 < 64)  { iTemp = (int) cmd2; c_motvalue <: iMotValue[iTemp-32]; 	}
						else if(cmd2 >= 64 && cmd2 < 96)  { iTemp = (int) cmd2; c_motvalue <: iMotPar[iTemp-64]; 	}
						else if(cmd2 >= 96 && cmd2 < 128) { iTemp = (int) cmd2; c_motvalue :> iTemp1;  iMotPar[iTemp-96] = iTemp1; iUpdateFlag=1;}
*/

 	  case 20:	iIndex1 =  0;                 // read parameter from XMOS ==> PC
 	            while(iIndex1 < 32)
 	            {
 	            c_motvalue <: (iIndex1+64); 	c_motvalue :> uMotorParameter[iIndex1];
 	     	 	iIndex1++;
 	            }
 	            cTxFlag = 'P';
 		        iStep=51;
 		        break;



 // =============00 'X'  write parameter to xmos 	=========================
	  case 30:  iIndex1=0;
 	            iIndex2=4;
 	            while(iIndex1 < 32)
 	            {
 	            uTemp =  cxRxBuf[iIndex2++];  uTemp = uTemp << 8;
 	            uTemp += cxRxBuf[iIndex2++];  uTemp = uTemp << 8;
 	            uTemp += cxRxBuf[iIndex2++];  uTemp = uTemp << 8;
 	            uTemp += cxRxBuf[iIndex2++];
 	            uMotorParameter[iIndex1++] = uTemp;
 	            }
 	            iStep++;
 	            break;

 	  case 31:  iIndex1=0;
 	  	  	  	while(iIndex1 < 32)
 	            {
 		        c_motvalue <: (iIndex1+96);
 	  	  	  	c_motvalue <: uMotorParameter[iIndex1];
 	  	  	  	iIndex1++;
 	            }
           	   	iStep=20;  // next step readout parameter
 	            break;
 //========================================================================



     case 40:	iIndex1 =  0;				// read motor values from XMOS ==> PC
     	 	 	while(iIndex1 < 32)
     	 	 	{
     	 	 	c_motvalue <: (iIndex1+32);
     	 	 	c_motvalue :> uMotorValues[iIndex1];
     	 	 	c_motvalue :> uMotorValues[iIndex1+1];
     	 	 	c_motvalue :> uMotorValues[iIndex1+2];
     	 	 	c_motvalue :> uMotorValues[iIndex1+3];
      	 	    iIndex1+=4;
                }
	            iStep=51;
	            cTxFlag = 'M';
	            break;

     case 51: iStep++;
             configure_out_port(p_ifm_ext_d0, clk1, 1);
    		 CRC1 = 0;
        	 CRC2 = 0;
             break;
//-------------------------------------
     case 52: cSendByte = cStartByte;
             iIndex1=0;
             iStepReturn= iStep+1;
             iStep     = defStepTxCRC;
             break;
     case 53:  cSendByte = cTxFlag;              // 'M' = Motor values 'P' = parameter
              iStepReturn= iStep+1;
              iStep     = defStepTxCRC;
              break;
    case 54:  cSendByte = 4+32*4+2;             // total numbers of Byte
              iStepReturn= iStep+1;
              iStep     = defStepTxCRC;
              break;
    case 55:  cSendByte = 'R';                   // reserve
              iStepReturn= iStep+1;
              iStep     = defStepTxCRC;
              break;

     case 56: if(cTxFlag == 'M')
    	      uSendValue = uMotorValues[iIndex1];
              else
              uSendValue = uMotorParameter[iIndex1];
    	      cSendByte  = (unsigned char)(uSendValue & 0xFF);
              iStepReturn= iStep+1;
              iStep= defStepTxCRC;
              break;
     case 57: uSendValue = uSendValue >> 8;
    	      cSendByte  = (unsigned char)(uSendValue & 0xFF);
              iStepReturn= iStep+1;
              iStep= defStepTxCRC;
              break;
     case 58: uSendValue = uSendValue >> 8;
    	      cSendByte  = (unsigned char)(uSendValue & 0xFF);
              iStepReturn= iStep+1;
              iStep= defStepTxCRC;
              break;
     case 59: uSendValue = uSendValue >> 8;
    	      cSendByte   = (unsigned char)(uSendValue & 0xFF);
             iStepReturn = iStep+1;
             iStep       = defStepTxCRC;
             break;
     case 60:iIndex1++;
    	     if(iIndex1 < 32) iStep=56;
    	     else
    	      {
              cSendByte = CRC1;
     		  CRC2 += cSendByte;
              iStepReturn= iStep+1;
              iStep     = defStepTxOnly;
    	      }
              break;
     case 61: cSendByte = CRC2;
              iStepReturn= iStep+1;
              iStep     = defStepTxOnly;
              break;
     case 62: iStep=0;
              break;

//----------------------  tx of one byte ------------
     case 70: p_ifm_ext_d0 <: defStartBit;
             cMaskByte = 0x01;
    		 CRC1 ^= cSendByte;
    		 CRC2 += cSendByte;
             iStep=72;
             break;

     case 71: p_ifm_ext_d0 <: defStartBit;
             cMaskByte = 0x01;
             iStep++;
             break;
     case 72: iStep++; break;
     case 73: if(cSendByte & cMaskByte)
    	     p_ifm_ext_d0 <: defBitOne;
             else
             p_ifm_ext_d0 <: defBitZero;
             iStep++;
             break;
     case 74: if(cMaskByte == 0x80) iStep++;
             else
             {
             iStep--;
             cMaskByte *= 2;
             }
             break;
     case 75: p_ifm_ext_d0 <: defStopBit;
     	 	 iStep++;
             break;
     case 76: iStep=iStepReturn;
             break;

     default: iStep=0; break;
     }
	}// end while 1
}

void CalcEncoderSpeed()
{
	  iEncoderPeriodMicroSeconds  = iEncoderCountMicroSeconds;
	  iEncoderSpeed   = 0xAA000000;
	  if(iEncoderPeriodMicroSeconds)
	  {
	  iEncoderSpeed  = 120000;
   	  iEncoderSpeed *= iNrEncoderPulses;
	  iEncoderSpeed /= iEncoderPeriodMicroSeconds;  // period in �sec
   	  iEncoderSpeed /= POLE_PAIRS; 					// pole pairs
	  }
	  if(iEncoderDirection < 0) iEncoderSpeed = -iEncoderSpeed;

	  iEncoderSpeed &= 0x00FFFFFF;
	  iEncoderSpeed |= 0xAA000000;

	  iEncoderPeriodNext = iEncoderPeriodMicroSeconds * 256;    // add 10%
	  iEncoderPeriodNext /= 230;
}


void CalcHallSpeed()
{
	  iHallPeriodMicroSeconds     = iHallCountMicroSeconds;

	  iHallSpeed   = 0xAA000000;
	  if(iHallPeriodMicroSeconds)
	  {
	  iHallSpeed  = 10000000;
	  iHallSpeed *= iNrHallPulses;
	  iHallSpeed /= iHallPeriodMicroSeconds;    	// period in �sec
	  iHallSpeed /= POLE_PAIRS; 				// pole pairs
	  }
	  if(iHallDirection < 0) iHallSpeed = -iHallSpeed;

	  iHallSpeed &= 0x00FFFFFF;
	  iHallSpeed |= 0xAA000000;

	  iHallPeriodNext = iHallPeriodMicroSeconds * 256;    // add 10%
	  iHallPeriodNext /= 230;
}

int function_EncoderReadout(port in p_encoder){

int   iEncoderCountx=0;
int	  iStepEncoder  =0;
int	  iEncoderOk    =0;
int   iEncState1	=0;
int   iEncState2	=0;

	while(iEncoderCountx < 20)
	  {
	  switch(iStepEncoder)
	  {
		  case 0: p_encoder :> iEncState1; iEncState1 &= 0x07; iStepEncoder++;
		          iEncoderOk++;
		  	  	  break;
		  case 1: p_encoder :> iEncState2; iEncState2 &= 0x07;
				  if(iEncState2 != iEncState1) iStepEncoder=0;
				  else iEncoderOk++;
				  break;
	  }
	  iEncoderCountx++;
	  }// end countx

	  if(iEncoderOk >= 10) return(iEncState1);
	  else return(-1);
}


void run_hall( chanend c_hall, port in p_hall, port in p_encoder)
 {
  timer tx;
  unsigned ts;					// newest timestamp
  unsigned cmd;
  int iTemp;

  int iHallAngle1;		        // newest angle (base angle on hall state transition)
  int iHallAngle2;
  int delta_angle;
  int iAngleLast;
  int iAngleDiff;


  int iCountTransitionSum;
  int iCountTransitionNew;
  int iCountTransitionFiltered;
  int iCountTransitionEstimated;
  int iHallCountOneTransition=0;
  int iTimeSaveOneTransition=0;


  unsigned iHallStateNew;			// newest hall state
  unsigned iHallStateOld;
  unsigned iHallState1,iHallState2;
  unsigned uHallNext,uHallPrevious;


  int iStepHall      =0;
  int iHallPosAbsolut=0;
  int iHallError=0;
  int iAngleDeltaSum 	= 0;
  int iAngleDeltaValue  = 682;

//=========== encoder =======================
  int iEncoderCountx;

  int iEncoderPinState;
  int iEncoderReferenz;
  int iEncoderStateNew;
  int iEncoderStateOld;
  int iEncoderError		    =0;
  int iEncoderPosAbsolut	=0;

  int iEncoderNext=0;
  int iEncoderPrevious=0;
  int iEncoderAngle = 1024;

  tx :> ts;  // first value

  p_hall :> iHallState1;
  iHallStateNew      = iHallState1;
  iHallStateOld = iHallState1;


  iEncoderPinState =  0;
  iEncoderCountx   = 32;
  while(iEncoderCountx > 0){
  iTemp = function_EncoderReadout(p_encoder);
  if(iTemp != -1) { iEncoderPinState = iTemp; iEncoderCountx=0;}
  }


  iEncoderStateNew = iEncoderPinState & 0x03;
  iEncoderStateOld = iEncoderStateNew;

  //********************* LOOP 1�sec ****************************
  while(1) {


//==================================== encoder ============================================================
//
//---------------------------------------------------------------------------------------------------------
#define defState0  0
#define defState1  2
#define defState2  3
#define defState3  1
#define defEncoderPeriodMax	  200000  //200msec

	  iEncoderCountMicroSeconds++;   		// period in �sec

	  iTemp = function_EncoderReadout(p_encoder);
	  if(iTemp != -1)iEncoderPinState = iTemp;

	  iEncoderReferenz = iEncoderPinState & 0x04;
	  iEncoderStateNew = iEncoderPinState & 0x03;

	 if(iEncoderStateOld != iEncoderStateNew)
	 {
		 if(iEncoderStateNew == iEncoderNext)    {iEncoderPosAbsolut++; iEncoderAngle++; iEncoderDirection++; iEncoderDirection &= 0x03;  }
	     if(iEncoderStateNew == iEncoderPrevious){iEncoderPosAbsolut--; iEncoderAngle--; iEncoderDirection--; if(iEncoderDirection< -3) iEncoderDirection=-3;  }
         iEncoderAngle &= 0xFFFF;   					// max 65535/rotation
         if(iEncoderReferenz)iEncoderAngle=0xAA000000;  // info about null reference

	      switch(iEncoderStateNew)
	  	  {
			  case defState0:   iEncoderNext=defState1; iEncoderPrevious=defState3;  break;
			  case defState1:   iEncoderNext=defState2; iEncoderPrevious=defState0;  break;
			  case defState2:   iEncoderNext=defState3; iEncoderPrevious=defState1;  break;
			  case defState3:   iEncoderNext=defState0; iEncoderPrevious=defState2;  break;
			  default: iEncoderError++; break;
	  	  }
	      iEncoderStateOld = iEncoderStateNew;

	      iCountEncoderPulses++;
	      if(iCountEncoderPulses >= iNrEncoderPulses)
	    	  {
	    	  CalcEncoderSpeed();
	    	  iCountEncoderPulses	         = 0;
	    	  iEncoderCountMicroSeconds      = 0;
	    	  }


       }// ======== end of NewState =====================

       if(iEncoderCountMicroSeconds > iEncoderPeriodNext)
	    	     CalcEncoderSpeed();

		if(iEncoderCountMicroSeconds > defEncoderPeriodMax)
			{
			iEncoderCountMicroSeconds = defEncoderPeriodMax;
			iEncoderSpeed   = 0xAA000000;
			}


	      /*
	      switch(iNrEncoderPulses)
	      {
	      case 10:  if(iEncoderPeriodMicroSeconds < 7143)    iNrEncoderPulses = 20;
	    	  	   break;
	      case 20:  if(iEncoderPeriodMicroSeconds > 9000)    iNrEncoderPulses = 10;
	      	  	   if(iEncoderPeriodMicroSeconds < 1429)     iNrEncoderPulses = 60;
	      	  	   break;
	      case 60:  if(iEncoderPeriodMicroSeconds > 1786)    iNrEncoderPulses = 20;
	      	       break;
	      default: iNrEncoderPulses=1; break;
	      }
	      */


//========================================== end encoder ===============================================



//============================================ H A L L =================================================
#define defHallState0 3
#define defHallState1 2
#define defHallState2 6
#define defHallState3 4
#define defHallState4 5
#define defHallState5 1
#define defPeriodMax 200000  //200msec

	  iHallCountMicroSeconds++;   		// period in �sec
	  iHallCountOneTransition++;
	  iAngleDeltaSum += iAngleDeltaValue;


	  switch(iStepHall)
	  {
		  case 0: p_hall :> iHallState1; iHallState1 &= 0x07; iStepHall++;
		  	  	  break;
		  case 1: p_hall :> iHallState2; iHallState2 &= 0x07;
				  if(iHallState2 == iHallState1) iStepHall++;
				  else iStepHall=0;
				  break;
		  case 2: p_hall :> iHallState2; iHallState2 &= 0x07;
				  if(iHallState2 == iHallState1) iHallStateNew = iHallState2;
				  else iStepHall=0;
				  break;
	  }


      if(iHallStateNew != iHallStateOld)
      {
 	      	  if(iHallStateNew == uHallNext)    {iHallPosAbsolut++; iHallDirection++; iHallDirection &= 0x03;  }
 	      	  if(iHallStateNew == uHallPrevious){iHallPosAbsolut--; iHallDirection--; if(iHallDirection < -3) iHallDirection=-3; }

 	      	  //if(iHallDirection >= 0) // CW  3 2 6 4 5 1

 	      	  switch(iHallStateNew)
 	      	  {
			  case defHallState0: iHallAngle1 =     0;  uHallNext=defHallState1; uHallPrevious=defHallState5;  break;
			  case defHallState1: iHallAngle1 =   682;  uHallNext=defHallState2; uHallPrevious=defHallState0;  break;   //  60
			  case defHallState2: iHallAngle1 =  1365;  uHallNext=defHallState3; uHallPrevious=defHallState1;  break;
			  case defHallState3: iHallAngle1 =  2048;  uHallNext=defHallState4; uHallPrevious=defHallState2;  break;   // 180
			  case defHallState4: iHallAngle1 =  2730;  uHallNext=defHallState5; uHallPrevious=defHallState3;  break;
			  case defHallState5: iHallAngle1 =  3413;  uHallNext=defHallState0; uHallPrevious=defHallState4;  break;   // 300 degree
			  default: iHallError++; break;
 	      	  }// end switch


	      iCountHallPulses++;
	      if(iCountHallPulses >= iNrHallPulses)
	      {
	    	  CalcHallSpeed();
	    	  iCountHallPulses 		  = 0;
	    	  iHallCountMicroSeconds  = 0;
	      }

	      /*
	      switch(iNrHallPulses)
	      {
	      case 1:  if(iHallPeriodMicroSeconds < 7143)    iNrHallPulses = 2;
	    	  	   break;
	      case 2:  if(iHallPeriodMicroSeconds > 9000)    iNrHallPulses = 1;
	      	  	   if(iHallPeriodMicroSeconds < 1429)    iNrHallPulses = 6;
	      	  	   break;
	      case 6:  if(iHallPeriodMicroSeconds > 1786)    iNrHallPulses = 2;
	      	       break;
	      default: iNrHallPulses=1; break;
	      }
*/
	   iTimeSaveOneTransition  = iHallCountOneTransition;
	   iCountTransitionNew     = iHallCountOneTransition;
	   iHallCountOneTransition = 0;
       delta_angle             = 0;
       iHallStateOld  	       = iHallStateNew;

        if(iHallDirection==1){
    	if(iHallAngle1 < 1024 && iAngleLast > 3072)
    		iAngleDiff  = (iHallAngle1 + 4096) - iAngleLast;
    	else
    		iAngleDiff  = iHallAngle1 - iAngleLast;
       }
      }// end (iHallStateNew != iHallStateOld

      if(iHallCountMicroSeconds > iHallPeriodNext)
	    	     CalcHallSpeed();

		if(iHallCountMicroSeconds > defPeriodMax)
			{
			iHallCountMicroSeconds = defPeriodMax;
			iHallSpeed   = 0xAA000000;
			}




 		if(iHallCountOneTransition)
 		{

 		if(iHallCountOneTransition == 1)
 		{
 			iCountTransitionSum      -= iCountTransitionFiltered;
 			iCountTransitionSum      += iCountTransitionNew;
 			iCountTransitionFiltered  = iCountTransitionSum/4;
 			iCountTransitionEstimated = (iCountTransitionFiltered*3)/4 + iCountTransitionNew/4;

 			iTemp = iCountTransitionNew - iCountTransitionFiltered;
 			iTemp *= 682;
 			iTemp /= iCountTransitionFiltered;
 			iAngleDeltaSum += iTemp;
 			if(iAngleDeltaSum > 692) iAngleDeltaSum = 692;
			if(iAngleDeltaSum < 672) iAngleDeltaSum = 672;
  		}

// 		if(iCountTransitionEstimated)
//		delta_angle = (682 *iHallCountOneTransition)/iCountTransitionEstimated; //iTimeSaveOneTransition;
		if(iCountTransitionEstimated)
		delta_angle = iAngleDeltaSum/iCountTransitionEstimated; //iTimeSaveOneTransition;
 		}

	  if(delta_angle >= 680) delta_angle = 680;

	  if(iHallCountOneTransition > 50000) iHallDirection = 0;

	  iHallAngle2 = iHallAngle1;
      if(iHallDirection > 0)  iHallAngle2 += delta_angle;
      if(iHallDirection < 0) iHallAngle2 -= delta_angle;
      iHallAngle2 &= 0x0FFF;    // 4095
      iAngleLast  = iHallAngle2;

//======================end HALL ===================================================================






   tx when timerafter(ts + 250) :> ts;   // 250 => 1�sec 125= 0,5�sec

	#pragma ordered
 	    select {
			case c_hall :> cmd:
			    if  (cmd == 8) { c_hall <: iHallSpeed;   iHallSpeed &= 0x00FFFFFF;
			 	 	 	 	 	   c_hall <: iHallAngle2;
			 	 	 	 	 	   c_hall <: iHallPosAbsolut;
			 	 	 	 	 	   c_hall <: iHallStateNew;
			 	 	 	 	 	 }
			  else if  (cmd == 9) {
				 	 	 	 	  c_hall <: iEncoderSpeed;   iEncoderSpeed &= 0x00FFFFFF;
				 			 	  c_hall <: iEncoderAngle;   iEncoderAngle &= 0xFFFF;  // send and clear info about null reference
				 			 	  c_hall <: iEncoderPosAbsolut;
				 			 	  c_hall <: iEncoderPinState + iEncoderReferenz*10;
			 	 	 	 	 	 }
			break;
			default:  break;
 	    }// end of select
  }// end while 1
}





