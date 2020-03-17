package pidtuner;

public class PID_ATune
{
  /*
  Tunes a PID controller using the relay method

  A process exists that responds to some stimulus, for example:
  	motor rotation is a result of power applied - RPM/volts or RPS/fraction of available power
	oven temperature is a result of power applied - degrees F/ampere

  The scenario is there may be a disturbance resulting in an error or deviation between the desired 
  process variable setpoint and the actual process variable.

  This error must be corrected efficiently - as quickly as possible without too much overshoot or overdamped.

  The assumption is if, for example, a process is temporarily too low it can be brought into compliance
  faster by giving temporarily more control than its natural state.

  Notice that the process variable is measured in some units and the stimulus is measured in some other units.

  Tuning factors are determined by the process variable response to particular known pertubation.  Simply,
  hit the process with a known blow and measure how it responds.

  The perturbation used by this tuner is everytime the process variable falls below the Setpoint then
  hit the process with a specified step up in control and everytime the process variable goes above the Setpoint
  then hit the process with the same size step down in control. (Explanation assumes DIRECT control not REVERSE control.)

  Tuner Produces Two Factors:
   Ku  the Ultimate Gain (where the process STARTs to oscillate)
   Pu  the Ultimate Time Period

         4                (control perturbation amplitude)
  Ku => -- * ------------------------------------------------------------
		pi   (process variable amplitude resulting from the perturbation)

  Pu => the time period of the oscillations forced by the alternating step perturbation. (Also known as Tu.)
  
  If gain applied to a process is less than Ku then approaches to the setpoint are slow and any oscillations fade.
  (Gain is the amplification of the error. Oscillations are caused by the delay or lag in the system response to the control stimulus.)
  If gain applied to a process is more than the Ku then the process oscillations grow.
  The gain at Ku produces stable oscillations.

  The mathematical question to answer is
  How much of the stimulus must be applied to reduce the error in the process variable in acceptable ways?

  That error can be expressed 3 ways:
  1. error at an instant - units of the process variable
  2. error accumulated over a longer period of time - time integrated process variable
  3. rate of change of the error over a short period of time - time differentiated process variable

  To compensate for each of the three errors multiplicative adjustment factors are applied:

  1. Kp - (control stimulus units / process units)
  error [process units] * Kp [stimulus units/process units] = stimulus [stimulus units]
  Example: error [RPM] * Kp [volts/RPM] = adjusted power [volts]

  2. Ki - (control stimulus units / process units) / time
  time accumulated error [process units * time] * Ki [(control stimulus units / process units) / time]
  Example: time accumulated error [RPM*seconds] * Ki [volts/RPM/second] = adjusted power [volts]

  3. Kd - (control stimulus units / process units) * time
  time rate of change of error [process units / time] * Kd [(control stimulus units / process units) * time)]
  Example: time rate of change of error [RPM/second] * Kd [volts/RPM*second] = adjusted power [volts]

  Kp, Ki, and Kd are various calculated combinations of Ku and Pu and are selected for the desired process behavior of
  slow or fast to respond and overshoot more or less or not at all.  See comments and code near the getKp, Ki, and Kd methods.

  The Ziegler-Nichols rule is a heuristic PID tuning rule that attempts to produce good values for the three PID gain parameters:
	Kp - the controller path gain
	Ti - the controller's integrator time constant
	Td - the controller's derivative time constant
  given two measured feedback loop parameters derived from measurements:
	the period Tu of the oscillation frequency at the stability limit
	the gain margin Ku for loop stability
	with the goal of achieving good regulation (disturbance rejection).

  The tuning method starts with the user picking a process variable setpoint and knowing the control signal
  that produces that setpoint.  For example, say we want to tune a process at 4000 RPM (at motor shaft).
  We need to know, also, the amount of power needed to spin the motor that fast - let's say it's determined
  to be .76 (controller setting of the fraction of the full power avaiable).

  Next a control signal perturbation must be choosen.  This number will be added and subtracted from the
  setpoint control signal.  Pick a reasonably large number that does NOT destroy the process.  In this
  example maybe .1 is a reasonable step up and down from the .76 power needed to maintain the Setpoint
  (steps will oscillate rapidly between .66 and .86). Thus the amplitude of the control signal perturbation is 2*Output Step.

  The tuning process is not determining the process variable at the highest and lowest control signal.  The
  method is much more dynamic than that and is determining not just how much the process changes but how fast it changes.

  The method is to apply a high control signal and watch for when the process rises above the Setpoint.  Remember that
  transit time as the start of a cycle and immediately reduce the control signal to the low value.  Watch the process
  variable continue to increase (from process lag) and note the time of the peak then watch it decrease (from the
  low control signal) below the process Setpoint.  Repeatedly lower the control signal when the process goes above
  the Setpoint and raise the control signal when the process goes below the Setpoint.

  Repeat the cycle several times to acheive a steady oscillation from hitting the process high when it goes low and
  lowering the process when it goes high.

  Measure the process variable amplitude between its maximum and the minimum.

  Measure the time of one period - say maximum to maximum.

  The control signal perturbation is known - that was an input to the tuner - twice the step size from the Setpoint.
  
  Note there are other nomenclatures than Ku, Pu, kP, kI, kD.  Related you will see Kc == Kp, Tu = Pu, and Ti and Td
  related to kI and kD

  Tuner Setpoint and input are the process variable units
  Output and oStep (Output step) are the control signal units that effect the process variable
  Time input is milliseconds

  Result methods retain the input units but return seconds - not milliseconds (see the division by 1000 in the code)
 
  Kp, Ki, Kd may be tuned using different units than the controller uses but that can lead to some confusion
  as the parameters must have the unts converted.

  For example a CTRE Talon motor controller will perform a PID controller using Kp, Ki, Kd with units of:
	encoder edges per motor shaft rotation per 100 ms and the time step is ms
  If the tuner was given gear box output shaft RPM and seconds, then a conversion to the Talon required units must be done.
  The example prorgam shows this.  Better yet, just tune with the native units of the controller.
   
  //commonly used functions **************************************************************************

    PID_ATune(double Setpoint, double Output, double oStep, DIRECTION ControllerDirection, CONTROL_TYPE ControlType, int SampleTime);
												// * Constructor
												// Sets the process variable setpoint and the control signal Output that produces that
												// setpoint
												// oStep is the perturbation of the Output control signal that sets up the oscillation 
												// of the process variable
    int Runtime(double input, int millis);		// * Similar to the PID Compute function, returns:
    											// 0 a time step toward tuning has been completed okay
    											// 1 tuning completed for better or for worse; no longer running tuning
    											// 2 called too quickly; time step has not elapsed; no action taken
												// 3 a peak was found and tuning parameters are available; additional tuning attempts
												//  will be made

    void Cancel();								// * Stops the AutoTune
	
	void SetOutputStep(double);					// * how far above and below the starting value will the output step
												// same as the constructor
	double GetOutputStep();						//
	
	void SetControlType(CONTROL_TYPE); 			// * Determines if the tuning parameters returned will be PI (D=0)
												// same as the constructor
	CONTROL_TYPE GetControlType();				//   or PID.  (0=PI, 1=PID)
	
	void SetLookbackTime(int);					// * how far back are we looking to identify peaks
												// default
	int GetLookbackTime();						//
	
	void SetSampleTime(int);					// * millisec
												// same as the constructor

	void SetNoiseBand(double);					// * the autotune will ignore signal chatter smaller than this value
												// default
	double GetNoiseBand();						//   this should be accurately set

	void SetControllerDirection(DIRECTION);	 	// * Sets the Direction, or "Action" of the controller. DIRECT
										  	  	//   means the output will increase when error is positive. REVERSE
										  	  	//   means the opposite.  it's very unlikely that this will be needed
										  	  	//   once it is set in the constructor.

// * once autotune is complete (Runtime returns 1 or 3),these functions contain the computed tuning parameters.	
	double GetKp();								// Kp input units
	double GetKi();								// Ki input units / second
	double GetKd();								// Kd input units * second
	double GetKu();								// * Ultimate Gain - input units
	double GetPu();								// * Ultimate Period - seconds
	double GetPeak_1();							// * time last maximum found - seconds
	double GetPeak_2();							// * time 2nd to the last maximum found - seconds

  private:
    void FinishUp();
    void SetKuPu();
    */

/**********************/

	// PID or PI
	public enum CONTROL_TYPE {PI_CONTROL, PID_CONTROL};

	// Control to process ratio positive means direct and negative means reversed.
	// The PID will either be connected to a DIRECT acting process (increasing Output leads
 	// to  increasing Input) or a REVERSE acting process (increasing Output leads to decreasing Input.)
	public enum DIRECTION {DIRECT, REVERSE};

/**********************/
	private boolean isMax, isMin;
	private double setpoint;
	private double oStep;
	private double noiseBand;
	private CONTROL_TYPE controlType;
	private boolean running;
	private int peak1, peak2, lastTime;
	private int sampleTime;
	private int nLookBack;
	private DIRECTION controllerDirection;
	private int peakType;
	private double[] lastInputs = new double[101];
	private int inputCount;
    private double[] peaks = new double[10];
	private int peakCount;
	private boolean justchanged;
	private double absMax, absMin;
	private double outputStart;
	private double Ku, Pu;
	private double output;

public PID_ATune(double Setpoint, double Output, double oStep, DIRECTION ControllerDirection, CONTROL_TYPE ControlType, int SampleTime)
{
	this.setpoint = Setpoint;
	outputStart = Output;
	this.SetOutputStep(oStep);
	this.controlType = ControlType;
	this.noiseBand = Setpoint * 0.01;
	this.running = false;
	this.SetSampleTime(SampleTime);
	this.SetLookbackTime(3*SampleTime);
	this.peakCount = 0;
	this.SetControllerDirection(ControllerDirection);
}

public void Cancel()
{
	running = false;
} 
 
public int Runtime(double input, int millis)
{
	if(peakCount>=10 && running) // filled peaks array and didn't find peaks within tolerance so end now anyway
	{
		running = false;
		FinishUp();
		return 1;
	}
	
	if(running && (millis-lastTime)<sampleTime) return 2; // called before sample time elapsed
	lastTime = millis;
	double refVal = input;
	if(!running)
	{ //initialize working variables the first time around
	  // assumption seems to be the first input sample is at steady state for the setpoint and output to the controller
		peakType = 0; //first time indicator => 0, then max => +1, min => -1
		peakCount=0;
		peak1 = millis;
		justchanged=false;
		inputCount = 0;
		absMax=refVal;
		absMin=refVal;
		output = outputStart + (controllerDirection==DIRECTION.DIRECT ? oStep: -oStep); // assure output bumped initially; it's done below also but only if outside noise band
		inputCount = 0;
		running = true;
	}
	else
	{
		if(refVal>absMax)absMax=refVal;
		if(refVal<absMin)absMin=refVal;
	}
	
	//oscillate the output base on the input's relation to the setpoint
	
	if(refVal>setpoint+noiseBand) output = outputStart - (controllerDirection==DIRECTION.DIRECT ? oStep: -oStep);
	else if (refVal<setpoint-noiseBand) output = outputStart + (controllerDirection==DIRECTION.DIRECT ? oStep: -oStep);

	if(inputCount < nLookBack)
	{  //we don't want to trust the maxes or mins until the inputs array has been filled; [nLookBack-1]=oldest, nLookBack[0]=newest
	lastInputs[nLookBack-1-inputCount++] = refVal;
	return 0;
	}

  //id peaks

  isMax=true;
  isMin=true;

  for(int i=nLookBack-1;i>=0;i--)
  {  //determine if current the max or min relative to all the nLookBack samples in lastInputs?
    double val = lastInputs[i];
    if(isMax) isMax = refVal>val;
    if(isMin) isMin = refVal<val;
    lastInputs[i+1] = lastInputs[i]; //move the last inputs up 1 place and make room for the new in [0]
  }
  lastInputs[0] = refVal;  
  
  if(isMax) //this is max compared to nLookBack samples
  {
    if(peakType==0)peakType=1; //this is max on first time
    if(peakType==-1) //this is max coming from a min
    {
      peakType = 1;
      peak2 = peak1; //save the time of the previous max peak; assumes the first peak found is a max otherwise peak1 is initialized at millis and peak1==peak2 -- but so what;
    }
    peak1 = millis; //record the time of the current max peak
    peaks[peakCount] = refVal; //first peakCount is 0 then bumped by each min coming from a max - see below
   
  }
  else if(isMin)
  {
    if(peakType==0)peakType=-1; //this is min on first time
    if(peakType==1) //this is min coming from a max
    {
      peakType=-1;
      peakCount++;
      justchanged=true;
	}
    
    if(peakCount<10)peaks[peakCount] = refVal;
  }
  
  if(justchanged && peakCount>2) //2 periods used if count is 3; can increase to say 7 to oscillate more often before checking (note that peaks has [10] elements capacity unless changed)
  { //we've transitioned.  check if we can autotune based on the last peaks being consistent
    double avgSeparation = (Math.abs(peaks[peakCount-1]-peaks[peakCount-2])+Math.abs(peaks[peakCount-2]-peaks[peakCount-3]))/2;
    if( avgSeparation < 0.20*(absMax-absMin))  // the constant fraction to tolerate for tuning is arbitrary; may need adjusting
	{
	  if(peakCount>7)
	  {
		  running = false;
		  FinishUp();
		  return 1;
	  }
	  else
	  {
		  SetKuPu();
		  justchanged=false;
	  	  return 3;
	  }
	}
  }
  justchanged=false;
  return 0;
}

void FinishUp()
{
	  SetKuPu();
}

void SetKuPu()
{
      //we can generate tuning parameters!
      Ku = 4*(2*oStep)/((absMax-absMin)*Math.PI); // units are whatever the output/input is - say Power/RPM, for example [Talon %VBus/RPM]
      Pu = (double)(peak1-peak2) / 1000.;  // compute period and convert from millisecs to secs
}

public double getOutput()
{
	return output;
}

public double GetPeak_1()
{
	return (double)peak1/1000.;
}

public double GetPeak_2()
{
	return (double)peak2/1000.;
}

/*
There are a lot of "minor" variations on the numbers.  Here's another:
Table 1: Ziegler–Nichols rule
Control type Kp     Ki       Kd
P            0.50Ku -        -
PI           0.45Ku 1.2Ku/Pu -
PID          0.60Ku 2Ku/Pu   Ku*Pu/8


Rule Name	Tuning Parameters
Classic Ziegler-Nichols	Kp = 0.6 Ku     Ti = 0.5 Tu     Td = 0.125 Tu
Pessen Integral Rule	Kp = 0.7 Ku     Ti = 0.4 Tu     Td = 0.15 Tu
Some Overshoot	Kp = 0.33 Ku    Ti = 0.5 Tu     Td = 0.33 Tu
No Overshoot	Kp = 0.2 Ku     Ti = 0.5 Tu     Td = 0.33 Tu
http://www.mstarlabs.com/control/znrule.html

https://en.wikipedia.org/wiki/Ziegler-Nichols_method
Ziegler–Nichols method
Control Type				Kp      Ti	    Td	    Ki	        Kd
P		     				0.5Ku	–    	–	    –       	–
PI	 			   		    0.45Ku	Tu/1.2	–   	0.54Ku/Tu	–
PD    						0.8Ku	–	    Tu/8	–       	KuTu/10
classic PID    				0.6Ku   Tu/2    Tu/8	1.2Ku/Tu    3KuTu/40}
Pessen Integral Rule	    7Ku/10 	2Tu/5 	3Tu/20 	1.75Ku/Tu 	21KuTu/200
some overshoot	 			Ku/3	Tu/2	Tu/3	0.666Ku/Tu 	KuTu/9
no overshoot				Ku/5 	Tu/2	Tu/3	(2/5)Ku/Tu 	KuTu/15}
The ultimate gain Ku is defined as 1/M, where M = the amplitude ratio, Ki=Kp/Ti and Kd=KpTd.

For more sophisticated version of this tuner:
https://github.com/t0mpr1c3/Arduino-PID-AutoTune-Library/blob/master/PID_AutoTune_v0/PID_AutoTune_v0.cpp
*/

public double GetKp() // input units
{
	return controlType==CONTROL_TYPE.PID_CONTROL ? 0.6 * Ku : 0.4 * Ku;
}

public double GetKi() // input units / second
{
	return controlType==CONTROL_TYPE.PID_CONTROL ? 1.2*Ku / Pu : 0.48 * Ku / Pu;  // Ki = Kc/Ti
}

public double GetKd() // input unit * second
{
	return controlType==CONTROL_TYPE.PID_CONTROL ? 0.075 * Ku * Pu : 0;  //Kd = Kc * Td
}

public double GetKu()
{
	return Ku;
}

public double GetPu()
{
	return Pu;
}

public void SetOutputStep(double Step)
{
	oStep = Step;
}

public double GetOutputStep()
{
	return oStep;
}

public void SetControlType(CONTROL_TYPE Type)
{
	controlType = Type;
}

public CONTROL_TYPE GetControlType()
{
	return controlType;
}
	
public void SetNoiseBand(double Band)
{
	noiseBand = Band;
}

public double GetNoiseBand()
{
	return noiseBand;
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
public void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
	sampleTime = NewSampleTime;
   }

}

// milliseconds to look back for min/max;
// user must make sure not to have longer than 1 period of the process variable oscillation
// limited to 1 to 100 samples without notification!!!!!
public void SetLookbackTime(int value)
{
	nLookBack = value / sampleTime;

    if (nLookBack<1) nLookBack = 1;
	
	if(nLookBack > 100) nLookBack = 100;

	return;
}

public int GetLookbackTime()
{
	return nLookBack * sampleTime;
}
/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (increasing Output leads
 * to  increasing Input) or a REVERSE acting process (increasing Output leads to decreasing Input.)
 * We need to know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
public void SetControllerDirection(DIRECTION Direction)
{
   controllerDirection = Direction;
}

}
