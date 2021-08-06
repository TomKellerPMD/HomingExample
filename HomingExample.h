/*  HomingExanple.h    */

void HomeSwitch(PMDAxisHandle * phAxis);
void HomeIndex(PMDAxisHandle * phAxis);
void StepperLimitSwitchHome(PMDAxisHandle * phAxis);
void HardStopHome(PMDAxisHandle* phAxis);
int WaitForHomingEvent(PMDAxisHandle* phAxis, PMDuint16 eventmask, PMDuint32 TimeOut);

