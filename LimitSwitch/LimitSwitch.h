#ifndef LIMITSWITCH_H_
#define LIMITSWITCH_H_
/*
 Returns state of a Switch - that is a Digital Input which may be closed or had closure (a falling
 transition) recently.
 
 Normally used for such sensors as a limit switch which is checked often and must be recognized as
 activated even if there is bounce or the switch is currently open because of, for example, motion
 which accidentally opens the switch.
 
 This is idiosyncratic: the switch is marked closed if it was closed at any time since the last check
 which could conceivably be a long time ago and the current state of the switch could be legitimately
 be open.
 
 Assumes switch is polled frequently and any closed state since the last poll must be recognized.
 
 Assumes Digital Input is wired to pull up (high) for inactive and switched (shorted) to ground low
 for active.
 
 Uses a counter to assure capturing a change of state any time since the last poll.
 
 Assumes the Digital Input has a counter available to it (they are limited so be careful).
*/
class DigitalInput;
class Counter;

class LimitSwitch
{
public:
	LimitSwitch(int channel);
	~LimitSwitch();
	bool IsOn();  // true if on, closed, conducting
	bool IsOff(); // true if off, open, non-conducting - the negation of IsOn()
private:
	enum SwitchStates {kIsNotOn=false, kIsOn=true}; // return to user values
	DigitalInput* mSwitch;
	Counter* mSwitchCounter;
};

#endif // LIMITSWITCH_H_
