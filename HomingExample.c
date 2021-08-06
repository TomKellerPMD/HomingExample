// TLK 8/5/2021  V1.0
// HomeExample.c : Defines the entry point for the console application.
// This examples demonstrates three different homing routines.  The user should select the appriopriate routine
// based on the availble feedback devices in the system.
//  1.  Use Home Switch
//  2.  Use first encoder index out of negative limit switch
//  3.  Home to negative limit (no encoder) 
//
// Axis should be fully configured and ready to move when these functions are called
// Use SetSignalSense to define active hi or active low home, index, and limits.
// For MC5x113 a Home Capture occurs on a rising edge by default.


#include "c-motion.h"
#include "PMDutil.h"
#include "PMDdiag.h"
#include "PMDsys.h"
#include "PMDecode.h"
#include <stdlib.h>
#include <stdio.h>
#include "HomingExample.h"



// HomeSwitch()
//  This assumes it is desirable to approach the home switch from its postive side and at least a negative limit switch is present.
//  
//  The HomeSwitch will become the zero position.
//  1. First check to see if HomeSwitch is already active, if so move away in positive direction and stop when inactive.  Go to step 3.
//  2. Check to see if NegativeLimit is active, if so move in positive direction.  Stop when on the positive side (far side) of the home switch.
//  3. Move toward HomeSwitch in negative direction.
//  4. Stop after HomeSwitch is detected.
//  5. Request Capture Value and use that to "zero" the HomeSwitch using AdjustActualPosition
//  6. Move to Final Position.

void HomeSwitch(PMDAxisHandle * phAxis)
{
	    
	
	PMDuint16 status;
	PMDint32 positioncapture,FinalPosition,cmdpos;
	PMDint32 HomeVelocity,HomeAcceleration;
	PMDresult result;
	PMDuint32 TimeOut;
	HomeVelocity = 3000;
	HomeAcceleration = 6;
	FinalPosition = 30;
	TimeOut = 20000;

	printf("\nHomeSwitch...\n");
	
	PMDSetCaptureSource(phAxis, PMDCaptureSourceHome);
	PMDSetProfileMode( phAxis, PMDProfileModeVelocity);
	PMDSetAcceleration(phAxis, HomeAcceleration );
			
	// First check state of HomeSwitch
	PMDGetSignalStatus(phAxis, &status);
	if(!(status&PMDSignalStatusEncoderHome))
	{
			 // Home switch is already active so we need to move away.
		printf("Home switch is already active.  Moving away...\n");
		PMDSetBreakpointValue(phAxis,PMDBreakpoint1,0x00080008);
		PMDSetBreakpoint(phAxis, PMDBreakpoint1,PMDAxis1,PMDBreakpointActionSmoothStop,PMDBreakpointTriggerSignalStatus);
		PMDResetEventStatus(phAxis,0);
		PMDSetVelocity( phAxis, HomeVelocity );  //move in positive direction	
		PMDUpdate(phAxis);
		PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusMotionComplete,TimeOut))
		printf("Home switch is now inactive....\n");
	
	}
		
	
	PMDResetEventStatus( phAxis, (PMDuint16)~PMDEventStatusCaptureReceived );
	PMDResetEventStatus( phAxis, (PMDuint16)~PMDEventStatusMotionComplete );
	PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusBreakpoint1);
	
	// Need to clear out any previous captures to rearm the capture mechanism
	PMDGetCaptureValue(phAxis,&positioncapture);
    
	PMDSetEventAction(phAxis, PMDEventActionEventMotionError, PMDEventActionAbruptStop);
	
	// Breakpoints will stop motion when home switch is active.
	PMDSetBreakpointValue(phAxis, PMDBreakpoint1, 0x00080008);
	PMDSetBreakpoint(phAxis, PMDBreakpoint1, PMDAxis1, PMDBreakpointActionSmoothStop, PMDBreakpointTriggerEventStatus);
	
	//Check to see if we are at negative limit
	if (!(status & PMDSignalStatusNegativeLimit))
	{
		printf("\nIn Negative Limit.\nNow moving in positve direction looking for home switch...\n\n");
		PMDSetVelocity(phAxis, HomeVelocity);
		PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusInNegativeLimit);
		PMDUpdate(phAxis);
		PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusCaptureReceived,TimeOut))
		PMDGetCaptureValue(phAxis, &positioncapture);   // only for re-arming capture, throw away value
		printf("\nHome switch found, moving to positve side....\n\n");
		status = 0;
		while (!status)   // wait for home switch to go inactive
		{
			PMDGetSignalStatus(phAxis, &status);
			status &= PMDSignalStatusEncoderHome;
		}
		printf("Home Switch inactive. Re-entering Home switch from positve side\n\n");

		PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusCaptureReceived);
		PMDGetCaptureValue(phAxis, &positioncapture);   // only for re-arming capture, throw away value
	}
	
	printf("Looking for Home switch...\n");

	PMDSetVelocity( phAxis, -HomeVelocity );
	PMDUpdate( phAxis );

	// Since we are doing a SmoothStop the motion will stop some time after the home switch goes active
	PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusCaptureReceived | PMDEventStatusInNegativeLimit, TimeOut))
	PMD_ABORTONERROR(WaitForHomingEvent( phAxis, PMDEventStatusMotionComplete, TimeOut))

	// Check to see if NegativeLimit was reached without seeing home switch.
	PMDGetEventStatus(phAxis, &status);
    if(status&PMDEventStatusInNegativeLimit)
	{
       printf("\nIn Negative Limit, Home switch not found.\nNow moving in positve direction looking for home switch...\n\n");
	   PMDSetVelocity(phAxis, HomeVelocity);
	   PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusInNegativeLimit); 
	   PMDUpdate(phAxis);
	   PMD_ABORTONERROR(WaitForHomingEvent( phAxis, PMDEventStatusCaptureReceived, TimeOut))
	   printf("\nHome switch found, moving to positve side....\n\n");
	   status=0;
	   while(!status)   // wait for home switch to go inactive
            {
				PMDGetSignalStatus(phAxis,&status);
				status&=PMDSignalStatusEncoderHome;
            }
       printf("Home Switch inactive. Re-entering Home switch from positve side\n\n");
	  
	   PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusCaptureReceived);
	   PMDGetCaptureValue(phAxis, &positioncapture);   // only for re-arming capture, throw away value
	
	   PMDSetVelocity(phAxis, -HomeVelocity);
	   PMDUpdate(phAxis);
	   PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusCaptureReceived, TimeOut))
    
	}

    printf("\nHome switch found and motion is stopped.\n");
	PMDGetCaptureValue(phAxis, &positioncapture);
	
	// The Adjust Actual Position command creates a reference relative to the location of the home switch.
	PMDAdjustActualPosition(phAxis, -positioncapture);
	printf("\nCapture Value=%d.\n",positioncapture);
		
	printf("\nMove to Final Position.....\n");

    PMDResetEventStatus( phAxis, (PMDuint16)~PMDEventStatusCaptureReceived );
    PMDResetEventStatus( phAxis, (PMDuint16)~PMDEventStatusMotionComplete);
	PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusBreakpoint1);

	PMDSetProfileMode(phAxis, PMDProfileModeTrapezoidal);
	PMDSetPosition(phAxis, FinalPosition);
	PMDSetVelocity(phAxis, HomeVelocity);
	PMDUpdate(phAxis);  // Move away from  home switch to "Finalposition". 
    
	PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusMotionComplete, TimeOut))
	PMDGetCommandedPosition(phAxis, &cmdpos);

	printf("\nHoming Complete.  Stopped at position %d\n",cmdpos);

}

//  HomeIndex()
//  This will move the axis to the negative limit.  It will then move away from the limit until the 
//  Index is seen. Assumes current position is not on the negative side of NegativeLimit. 
//  The first index on the postive side of the negative limit will become the zero position.
//  1. First check to see if the Negative Limit Switch is already active, if so move away.
//  2. Move toward negative limit
//  3. Stop after limit switch is detected
//  4. Move out of limit switch until index is detected
//  5. Request Capture Value and use that to "zero" the index
//  6. Move to Final Position


void HomeIndex(PMDAxisHandle * phAxis)
{
	PMDuint16 status;
	PMDint32 positioncapture, FinalPosition, cmdpos;
	PMDint32 HomeVelocity, HomeAcceleration;
	PMDresult result;
	PMDuint32 TimeOut;
	HomeVelocity = 3000;
	HomeAcceleration = 6;
	FinalPosition = 30;
	TimeOut = 20000;

	printf("\nIndexHoming...\n");
    
	PMDSetCaptureSource(phAxis, PMDCaptureSourceIndex);
	PMDSetProfileMode( phAxis, PMDProfileModeVelocity);
	PMDSetAcceleration( phAxis, HomeAcceleration );
		
	// First check state of the Negative Limit Switch
	PMDGetSignalStatus(phAxis, &status);
	if(!(status&PMDSignalStatusNegativeLimit))
	{
			 // Negative Limit switch is already active so we need to move away.
		    printf("Negative Limit switch is already active.  Moving away...\n");
			PMDResetEventStatus(phAxis,PMDEventStatusInNegativeLimit);
			PMDSetVelocity( phAxis, HomeVelocity );  //move in positive direction
			PMDUpdate(phAxis);
            status=0;
			while(!status)   // wait for negative limit to go inactive
            {
				PMDGetSignalStatus(phAxis,&status);
				status&=PMDSignalStatusNegativeLimit;
            }
	 	
	}
		
	printf("Moving toward Negatime Limit switch...\n");
	PMDSetVelocity( phAxis, -HomeVelocity );  //move in negative direction
	PMDUpdate(phAxis);

	PMD_ABORTONERROR(WaitForHomingEvent( phAxis, PMDEventStatusInNegativeLimit, TimeOut))
	PMDResetEventStatus(phAxis, ~PMDEventStatusInNegativeLimit);
	printf("\nNegative Limit found now moving out...\n");
	PMDSetVelocity (phAxis, HomeVelocity);
	PMDUpdate(phAxis);
	status=0;
	while(!status)   // wait for negative limit to go inactive
    {
		PMDGetSignalStatus(phAxis,&status);
		status&=PMDSignalStatusNegativeLimit;
    }
		
        //  Since the negative limit is no longer active we can add a stop here but it is not necessary.
		//	PMDSetStopMode(phAxis, PMDSmoothStopMode);
		//	PMDUpdate(phAxis);
    printf("\nNegative Limit no longer active, now looking for index...\n");    
	PMDResetEventStatus( phAxis, (PMDuint16)~PMDEventStatusCaptureReceived );
	PMDResetEventStatus( phAxis, (PMDuint16)~PMDEventStatusMotionComplete );
	// Need to clear out any previous captures to rearm the capture mechanism
	PMDGetCaptureValue(phAxis,&positioncapture);

	PMDSetBreakpointValue(phAxis,0,0x00080008);  // Trigger on CaptureBit in EventStatus
	PMDSetBreakpoint(phAxis,0,PMDAxis1,PMDBreakpointActionSmoothStop,PMDBreakpointTriggerEventStatus);
    // Breakpoints will stop motion when index is seen.
		
	// Since we are doing a SmoothStop the motion will stop some time after the index appears
	PMD_ABORTONERROR(WaitForHomingEvent( phAxis, PMDEventStatusCaptureReceived, TimeOut))
	PMD_ABORTONERROR(WaitForHomingEvent( phAxis, PMDEventStatusMotionComplete, TimeOut))
	
	printf("\nIndex seen and motion is stopped.\n");

	PMDGetCaptureValue(phAxis, &positioncapture);
	PMDAdjustActualPosition(phAxis, -positioncapture);
    
	
	//	PMDClearPositionError(phAxis);  // useful for Stepper/Encoder systems
	//	PMDUpdate(phAxis);

	printf("\nMove to Final Position.....\n");

	PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusCaptureReceived);
	PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusMotionComplete);
	PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusBreakpoint1);

	PMDSetProfileMode(phAxis, PMDProfileModeTrapezoidal);
	PMDSetPosition(phAxis, FinalPosition);
	PMDSetVelocity(phAxis, HomeVelocity);
	PMDUpdate(phAxis);  // Move away from  home switch to "Finalposition". 

	PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusMotionComplete, TimeOut))
	PMDGetCommandedPosition(phAxis, &cmdpos);

	printf("\nHoming Complete.  Stopped at position %d\n", cmdpos); 
	
}




//*************************************
// HardStopHome()
// 
// The hard stop will become the zero potion.
//  
//  1. Start move in the negative direction
//	2. Wait For Motion Error resulting from obstruction.
//  3. Wait for saturatrion to Motor Limit.
//  4. SetActualPosition for desired position of obstruction.
//  5. ClearPositionError
//***************************************

void HardStopHome(PMDAxisHandle* phAxis)
{
	PMDuint16 MotionErrorActionRestore,MotorLimitRestore,MotorLimit;
	PMDint32 FinalPosition, cmdpos;
	PMDint32 HomeVelocity, HomeAcceleration;
	PMDint16 activecommand=0;
	PMDresult result;
	PMDuint32 TimeOut;
	
	HomeVelocity = 3000;
	HomeAcceleration = 6;
	FinalPosition = 30;
	TimeOut = 20000;
	MotorLimit = 1000;  //This will control the force pushing against the hard stop 
	
	PMDGetMotorLimit(phAxis, &MotorLimitRestore);
	PMDGetEventAction(phAxis, PMDEventActionEventMotionError, &MotionErrorActionRestore);

	PMDSetEventAction(phAxis, PMDEventActionEventMotionError, PMDEventActionAbruptStop);
	PMDSetMotorLimit(phAxis, MotorLimit);

	PMDSetProfileMode( phAxis, PMDProfileModeVelocity);
	PMDSetAcceleration(phAxis, HomeAcceleration );
	
	printf("Moving toward hard stop...\n");
	PMDSetVelocity( phAxis, -HomeVelocity );
	PMDUpdate( phAxis );
	PMD_ABORTONERROR(WaitForHomingEvent( phAxis, PMDEventStatusMotionError,TimeOut))
	
	while (activecommand < MotorLimit)   // wait for active motor command to saturate
	{
		PMDGetActiveMotorCommand(phAxis, &activecommand);
	}
	
	printf("At hard stop.\n");

	//  The location of the hard stop will become position 0
	PMDSetActualPosition(phAxis,0);
		
	PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusMotionComplete);
	
	// At this point the profile is stoped but there is a large steady state position error to flush.
	PMDClearPositionError(phAxis);
	PMDUpdate(phAxis);

	// Move to "Finalposition". 
	PMDSetProfileMode(phAxis, PMDProfileModeTrapezoidal);
	PMDSetPosition(phAxis, FinalPosition);
	PMDSetVelocity(phAxis, HomeVelocity);
	PMDUpdate(phAxis);  

	PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusMotionComplete, TimeOut))
	PMDGetCommandedPosition(phAxis, &cmdpos);

	printf("\nHoming Complete.  Stopped at position %d\n", cmdpos);
	
	//restore settings
	PMDSetEventAction(phAxis, PMDEventActionEventMotionError, MotionErrorActionRestore);
	PMDSetMotorLimit(phAxis, MotorLimitRestore);

	return;
}

//******************
// StepperLimitSwitchHome()
//
//
//  This assumes the negative limit switch will be found when travelling in the negative direction.  And no encoder is present.
//  The negative limit will become the zero position.
//  1. Check state of negative limit.
//  2. If not active start move toward negative Limit at nominal speed.
//  2. When negative limit is reached reverse direction and move away from negative limit.
//  3. When negative limit is no longer active move slowing back into negative limit. This should be done with a velocity < 1 step (or microstep)/cycle
//  2. Default reaction to a limit switch event is a Stop.
//  3. Set current position to "zero". 

void StepperLimitSwitchHome(PMDAxisHandle* phAxis)
{

	PMDint32 FinalPosition, cmdpos;
	PMDint32 HomeVelocity, SlowVelocity, HomeAcceleration;
	PMDuint32 TimeOut;
	PMDuint16 status;
	PMDresult result;
		
	TimeOut = 20000;
	FinalPosition = 30;
	HomeVelocity = 100000;
	HomeAcceleration = 100;
	SlowVelocity = 7000;  // should be less than 65536 (i.e. 1 uStep/cycle)

	printf("\nStepperLimitSwitchHome...\n");

	PMDResetEventStatus(phAxis, (PMDuint16)~PMDEventStatusMotionComplete);
	PMDSetAcceleration(phAxis, HomeAcceleration);

	// First check state if the Negative Limit Switch
	PMDGetSignalStatus(phAxis, &status);
	if ((status & PMDSignalStatusNegativeLimit))   // if negative limit is not active
	{
		printf("Move toward Negatime Limit switch...\n");
		PMDSetProfileMode(phAxis, PMDProfileModeVelocity);
		PMDSetVelocity(phAxis, -HomeVelocity);
		PMDSetAcceleration(phAxis, 256);
		PMDUpdate(phAxis);
		PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusInNegativeLimit, TimeOut))
		printf("\nNegative Limit found now moving out of switch slowly.\n");
	}
	else
	{
		// Negative Limit switch is already active so we need to move away.
		printf("Negative Limit switch is already active.  Moving away...\n");
	}

	PMDResetEventStatus(phAxis, ~PMDEventStatusInNegativeLimit);
	PMDSetVelocity(phAxis, HomeVelocity);  //move in positive direction
	PMDUpdate(phAxis);
	status = 0;
	while (!status)   // wait for negative limit to go inactive
	{
		PMDGetSignalStatus(phAxis, &status);
		status &= PMDSignalStatusNegativeLimit;
	}

	printf("\nNegative Limit no longer active,  moving back into switch slowly.\n");

	PMDSetVelocity(phAxis, -SlowVelocity);
	PMDUpdate(phAxis);
	PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusInNegativeLimit, TimeOut))

	printf("\nNegative Limit found again.\n");

	PMDSetActualPosition(phAxis, 0);  // negative limit is now zero position

	printf("\nMove to FinalPosition.....\n");
	PMDResetEventStatus(phAxis, ~PMDEventStatusMotionComplete);
	PMDResetEventStatus(phAxis, ~PMDEventStatusInNegativeLimit);

	// Move to "Finalposition". 
	PMDSetProfileMode(phAxis, PMDProfileModeTrapezoidal);
	PMDSetPosition(phAxis, FinalPosition);
	PMDSetVelocity(phAxis, HomeVelocity);
	PMDUpdate(phAxis);

	PMD_ABORTONERROR(WaitForHomingEvent(phAxis, PMDEventStatusMotionComplete, TimeOut))
	PMDGetCommandedPosition(phAxis, &cmdpos);

	printf("\nHoming Complete.  Stopped at position %d\n", cmdpos); 
	
	return;

}

//*****************************************************************************
// WaitForHomingEvent 
//
//   Waits for the specified event to be set in the EventStatus register.
//
//*****************************************************************************
PMDresult WaitForHomingEvent(PMDAxisHandle* phAxis, PMDuint16 eventmask,PMDuint32 TimeOut)
{
	PMDresult eventoccured = -1;
	PMDuint16 status = 0;
	PMDresult result = PMD_NOERROR;
	PMDint32 actpos, cmdpos;
	PMDuint32 startime,stoptime,sampletime,currenttime;

	printf("Waiting for event %04X on axis %d.\n", eventmask, phAxis->axis + 1);
	printf("Hit space to abort.\n");

	PMDGetSampleTime(phAxis, &sampletime);
	PMDGetTime(phAxis, &startime);
	stoptime = (PMDuint32) ((float)TimeOut / ((float)sampletime * 0.001) + startime);

	while (eventoccured==-1 && result == PMD_NOERROR)
	{
        PMDTaskWait(200);
		PMDGetTime(phAxis,&currenttime);
		if (currenttime > stoptime)
		{
			printf("Time Out Waiting For Event");
			return PMD_ERR_WaitTimedOut;
		}
		
		PMDGetCommandedPosition(phAxis, &cmdpos);
		PMDGetActualPosition(phAxis, &actpos);
		result = PMDGetEventStatus( phAxis, &status );
		
	//	printf("cmd=%d   act=%d evt=%x\n", cmdpos, actpos,status); 
		if (status & eventmask)
		{
			eventoccured=PMD_ERR_OK;
			printf("Event(s) %04X Set.\n", status);
			
		}
	}
	if (status & PMDEventStatusMotionError)
	{
		printf("-------------- Motion Error!\n");
	}
	
	return eventoccured;
}

