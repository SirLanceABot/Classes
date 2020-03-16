package frc.robot;

public class PID_ATune
{
  /*
  //commonly used functions **************************************************************************

    PID_ATune(double Setpoint, double Output, double oStep, DIRECTION ControllerDirection, CONTROL_TYPE ControlType, int SampleTime);
												// * Constructor

    int Runtime(double input, int millis);		// * Similar to the PID Compute function, returns:
    											// 0 a time step toward tuning has been completed okay
    											// 1 tuning completed for better or for worse; no longer running tuning
    											// 2 called too quickly; time step has not elapsed; no action taken
    											// 3 a peak was found and tuning parameters are available; additional tuning attempts will be made

    void Cancel();								// * Stops the AutoTune
	
	void SetOutputStep(double);					// * how far above and below the starting value will the output step?
	double GetOutputStep();						//
	
	void SetControlType(CONTROL_TYPE); 			// * Determines if the tuning parameters returned will be PI (D=0)
	CONTROL_TYPE GetControlType();				//   or PID.  (0=PI, 1=PID)
	
	void SetLookbackTime(int);					// * how far back are we looking to identify peaks
	int GetLookbackTime();						//
	
	void SetSampleTime(int);					// * millisec

	void SetNoiseBand(double);					// * the autotune will ignore signal chatter smaller than this value
	double GetNoiseBand();						//   this should be accurately set
	
	double GetKp();								// * once autotune is complete (Runtime returns 1 or 3),
	double GetKi();								//   these functions contain the computed tuning parameters.
	double GetKd();								//
	double GetKu();								//
	double GetPu();								//
	double GetPeak_1();							// * time last maximum found
	double GetPeak_2();							// * time 2nd to the last maximum found
	void SetControllerDirection(DIRECTION);	 	// * Sets the Direction, or "Action" of the controller. DIRECT
										  	  	//   means the output will increase when error is positive. REVERSE
										  	  	//   means the opposite.  it's very unlikely that this will be needed
										  	  	//   once it is set in the constructor.
  private:
    void FinishUp();
    void SetKuPu();
    */

	// control types:
	public enum CONTROL_TYPE {PI_CONTROL, PID_CONTROL};
	public enum DIRECTION {DIRECT, REVERSE};

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
*/

public double GetKp()
{
	return controlType==CONTROL_TYPE.PID_CONTROL ? 0.6 * Ku : 0.4 * Ku;
}

public double GetKi()
{
	return controlType==CONTROL_TYPE.PID_CONTROL ? 1.2*Ku / Pu : 0.48 * Ku / Pu;  // Ki = Kc/Ti
}

public double GetKd()
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

public void SetControlType(CONTROL_TYPE Type) // 0=PI_CONTROL, 1=PID_CONTROL
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
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  We need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
public void SetControllerDirection(DIRECTION Direction)
{
   controllerDirection = Direction;
}

}
