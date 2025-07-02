// UltraSonic sensor input or fake data from left joystick
// and spiky data from button press left joystick button

package frc.robot;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final int joystickPort = 0;
  private static final XboxController controller = new XboxController(0);
  private static boolean useJoystick = false;

  private static final int kUltrasonicPort = 0;
  private final AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
  
  // compute the Savitzky-Golay filtering coefficients assuming sampling at a fixed time interval
  int SGWindowSize = 10; // size of filter window
  private final SavitzkyGolayFilter SGfilter = new SavitzkyGolayFilter(SGWindowSize);

  // Another Savitzky-Golay Filter used after Spike Filter of raw data
  private final SavitzkyGolayFilter SSGfilter = new SavitzkyGolayFilter(SGWindowSize);

  // Spike Filter
  private final SpikeFilter Sfilter = new SpikeFilter(.3, .4, 1); // guesstimate numbers to test response

  // Median filter to discard outliers; filters over 5 samples
  private final MedianFilter Mfilter = new MedianFilter(5);

  // MAD filter to replace outliers; filter over 9 samples; reject 3.5 times norm
  private final MADFilter madfilter = new MADFilter(9, 3.5);

  // IQR filter to replace outliers; filter over 9 samples; reject 1.5 times norm
  private final IQRFilter iqrfilter = new IQRFilter(9, 1.5);

  // One Euro filter
  double frequency = 50; // Hz
  double mincutoff = 1.0; // Hz
  double beta = 0.1;     
  double dcutoff = 1.0;
  private final OneEuroFilter oefilter = new OneEuroFilter(frequency, mincutoff, beta, dcutoff); 

  // Alpha-Beta Filter
  private final AlphaBetaFilter abfilter = new AlphaBetaFilter(0.8, 0.2, 0.02, 0., 0.); // use actual first data point would be better

  double yActual = 0;

  // spike filter
  double yPredictedS = 0.;

  // normally use the output of the preceding spike filter that has been tuned

  // least squares filter
  double yPredictedSG = 0.;

  // least squares filter after spike filter
  double yPredictedSSG = 0.;

  // median filter
  double yPredictedM = 0.;

  // MAD filter
  double yPredictedMAD = 0.;

  // IQR filter
  double yPredictedIQR = 0.;

  // One Euro filter
  double yPredictedOE = 0.;

  // Alpha-Beta Filter
  double yPredictedAB = 0.;

  // WPILog
  DoubleLogEntry rawLogEntry;
  DoubleLogEntry spikeFilterLogEntry;
  DoubleLogEntry SGfilterLogEntry;
  DoubleLogEntry SSGfilterLogEntry;
  DoubleLogEntry MedianFilterLogEntry;
  DoubleLogEntry MADfilterLogEntry;
  DoubleLogEntry IQRfilterLogEntry;
  DoubleLogEntry OEfilterLogEntry;
  DoubleLogEntry ABfilterLogEntry;

  public void robotInit() {
    
    DataLogManager.start();
    DataLog log = DataLogManager.getLog();
    String USname = new String("/UltraSonic/"); // make a prefix tree structure for the ultrasonic data

    rawLogEntry = new DoubleLogEntry(log, USname+"raw", "RawCounts");
    spikeFilterLogEntry = new DoubleLogEntry(log, USname+"spikeFilter", "PredictedCounts");
    SGfilterLogEntry = new DoubleLogEntry(log, USname+"SGfilter", "PredictedCounts");
    SSGfilterLogEntry = new DoubleLogEntry(log, USname+"SSGfilter", "PredictedCounts");
    MedianFilterLogEntry = new DoubleLogEntry(log, USname+"Medianfilter", "PredictedCounts");
    MADfilterLogEntry = new DoubleLogEntry(log, USname+"MADfilter", "PredictedCounts");
    IQRfilterLogEntry = new DoubleLogEntry(log, USname+"IQRfilter", "PredictedCounts");
    OEfilterLogEntry = new DoubleLogEntry(log, USname+"OEfilter", "PredictedCounts");
    ABfilterLogEntry = new DoubleLogEntry(log, USname+"ABfilter", "PredictedCounts");
    }

    @Override
    public void robotPeriodic()
    {
      useJoystick = DriverStation.isJoystickConnected(joystickPort);
    }

    @Override
    public void teleopPeriodic() {

    calculate();

    SmartDashboard.putNumber("actual", yActual);
    rawLogEntry.append(yActual);

    // spike filter
    SmartDashboard.putNumber("Spike filter", yPredictedS);
    spikeFilterLogEntry.append(yPredictedS);

    // normally use the output of the preceding spike filter that has been tuned

    // least squares filter
    SmartDashboard.putNumber("SG filter", yPredictedSG);
    SGfilterLogEntry.append(yPredictedSG);

    // least squares filter after spike filter
    SmartDashboard.putNumber("Spike SG filter", yPredictedSSG);
    SSGfilterLogEntry.append(yPredictedSSG);

    // median filter
    SmartDashboard.putNumber("Median filter", yPredictedM);
    MedianFilterLogEntry.append(yPredictedM);

    // MAD filter
    SmartDashboard.putNumber("MAD filter", yPredictedMAD);
    MADfilterLogEntry.append(yPredictedMAD);

    // IQR filter
    SmartDashboard.putNumber("IQR filter", yPredictedIQR);
    IQRfilterLogEntry.append(yPredictedIQR);

    // One Euro filter
    SmartDashboard.putNumber("OE filter", yPredictedOE);
    OEfilterLogEntry.append(yPredictedOE);

    // Alpha-Beta filter
    SmartDashboard.putNumber("AB filter", yPredictedAB);
    ABfilterLogEntry.append(yPredictedAB);
  }

  public void calculate()
  {
      if (useJoystick) // joystick to replace analog input
      {
        yActual = controller.getLeftY(); // forward/backward
        yActual += controller.getLeftStickButtonPressed() ? 0.5 : 0.;
      }
      else // analog input
      {
        // sensor returns a value from 0-4095 that is scaled to inches
        // ultrasonics tend to be quite noisy and susceptible to sudden outliers
        // Pin 3-AN- Outputs analog voltage with a scaling factor of (Vcc/512) per cm.
        // A supply of 5V yields ~9.8mV/cm and 3.3V yields ~6.4mV/cm
        var voltsPerCM = edu.wpi.first.wpilibj.RobotController.getVoltage5V() / 512.;
        yActual = m_ultrasonic.getVoltage()/voltsPerCM/30.4/*cm per foot*/;        
      }

      // SmartDashboard.putNumber("5v", voltsPerCM);
      // spike filter
      yPredictedS = Sfilter.calculate(yActual);

      // normally use the output of the preceding spike filter that has been tuned

      // least squares filter
      yPredictedSG = SGfilter.calculate(yActual);
  
      // least squares filter after spike filter
      yPredictedSSG = SSGfilter.calculate(yPredictedS);

      // median filter
      yPredictedM = Mfilter.calculate(yActual);
  
      // MAD filter
      yPredictedMAD = madfilter.calculate(yActual);

      // IQR filter
      yPredictedIQR = iqrfilter.calculate(yActual);

      // One Euro filter
      yPredictedOE = oefilter.calculate(yActual, Timer.getFPGATimestamp());

      // Alpha-Beta Filter
      yPredictedAB = abfilter.calculate(yActual);
  }
}
