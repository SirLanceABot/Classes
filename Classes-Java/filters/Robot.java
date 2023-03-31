//Cool trick, if your file path starts with "/U/...", it will write a file to the first connected USB flash drive it can find on the RoboRIO. Much easier to get the files out of the RIO.

// /U/logs/memorytable.csv will be written on the first flash drive, folder 'logs', file name 'memorytable.csv'.

// UltraSonic sensor input or fake data from a joystick and spiky data from button A press/release

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  XboxController xbox = new XboxController(0);

  //private final File memoryLog = new File("/U/logs/memorytable.csv");
  private final Path memoryLog = Path.of("/U/memorytable.csv");
  private static final int kUltrasonicPort = 0;
  private final AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
  
  // compute the Savitzky-Golay filtering coefficients assuming sampling at a fixed time interval
  int SGWindowSize = 10; // size of filter window
  private final SavitzkyGolayFilter SGfilter = new SavitzkyGolayFilter(SGWindowSize);

  // Another Savitzky-Golay Filter used after Spike Filter of raw data
  private final SavitzkyGolayFilter SSGfilter = new SavitzkyGolayFilter(SGWindowSize);

  // Spike Filter
  private final SpikeFilter Sfilter = new SpikeFilter(100., .8, 2); // guesstimate numbers to test response

  // Median filter to discard outliers; filters over 9 samples
  private final MedianFilter Mfilter = new MedianFilter(9);

  // MAD filter to replace outliers; filter over 9 samples; reject 3.5 times norm
  private final MADFilter madfilter = new MADFilter(9, 3.5);

  // IQR filter to replace outliers; filter over 9 samples; reject 1.5 times norm
  private final IQRFilter iqrfilter = new IQRFilter(9, 1.5);

  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTableEntry xyEntry;
  double x = 0;
  double y = 0;
  // network tables only passes through the first 255 no matter how large the array is
  double[] xy = new double[254]; // saving 2 values so largest multiple of 2 that's less than 255
  int xyCounter = 0;
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

  // private void logMemoryUse() {
  //   try {  
  //     var temp = System.currentTimeMillis() + "," + (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory() + "\n");
  //     Files.writeString(memoryLog, temp, StandardCharsets.UTF_8, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
  //   } catch (IOException e) {
  //     // Couldn't write to the file -- handle it how you want
  //     System.out.println(e);
  //   }
  // }

  // WPILog
  DoubleLogEntry rawLogEntry;
  DoubleLogEntry spikeFilterLogEntry;
  DoubleLogEntry SGfilterLogEntry;
  DoubleLogEntry SSGfilterLogEntry;
  DoubleLogEntry MedianFilterLogEntry;
  DoubleLogEntry MADfilterLogEntry;
  DoubleLogEntry IQRfilterLogEntry;

  private double flip = -1.;

  public void robotInit() {
    
    // DataLog log = new DataLog("/home/lvuser", "MyUStestLog"+System.currentTimeMillis()+".wpilog");
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
    
    //Get the default instance of NetworkTables that was created automatically
    //when your program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //Get the table within that instance that contains the data. There can
    //be as many tables as you like and exist to make it easier to organize
    //your data. In this case, it's a table called datatable.
    NetworkTable table = inst.getTable("datatable");
    //Get the entries within that table that correspond to the X and Y values
    //for some operation in your program.
    xEntry = table.getEntry("X");
    yEntry = table.getEntry("Y");  
    xyEntry = table.getEntry("XY");

    System.out.println("the hierarchy " + NetworkTable.getHierarchy("")); // [/]

    addPeriodic(()->calculate(), 0.05);
    }

    @Override
    public void robotPeriodic()
    {
      // System.out.println(m_ultrasonic.getVoltage());
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    flip = -flip;

    // logMemoryUse();

    // sensor returns a value from 0-4095 that is scaled to inches
    // returned value is filtered with a rolling median filter, since ultrasonics
    // tend to be quite noisy and susceptible to sudden outliers

    //System.out.println("Sensor " + y[n-1] + ",  Least Square Predication " + yPredictedSG);

    SmartDashboard.putNumber("actual", fuzz(yActual, flip));
    rawLogEntry.append(yActual);

    // spike filter
    SmartDashboard.putNumber("Spike filter", fuzz(yPredictedS, flip));
    SmartDashboard.putNumber("Spike filter diff", fuzz(yActual - yPredictedS, flip));
    spikeFilterLogEntry.append(yPredictedS);

    // normally use the output of the preceding spike filter that has been tuned

    // least squares filter
    SmartDashboard.putNumber("SG filter", fuzz(yPredictedSG, flip));
    SmartDashboard.putNumber("SG filter diff", fuzz(yActual - yPredictedSG, flip));
    SGfilterLogEntry.append(yPredictedSG);

    // least squares filter after spike filter
    SmartDashboard.putNumber("Spike SG filter", fuzz(yPredictedSSG, flip));
    SmartDashboard.putNumber("Spike SG filter diff", fuzz(yPredictedS - yPredictedSSG, flip));
    SSGfilterLogEntry.append(yPredictedSSG);

    // median filter
    SmartDashboard.putNumber("Median filter", fuzz(yPredictedM, flip));
    SmartDashboard.putNumber("Median filter diff", fuzz(yActual - yPredictedM, flip));
    MedianFilterLogEntry.append(yPredictedM);

    // MAD filter
    SmartDashboard.putNumber("MAD filter", fuzz(yPredictedMAD, flip));
    SmartDashboard.putNumber("MAD filter diff", fuzz(yActual - yPredictedMAD, flip));
    MADfilterLogEntry.append(yPredictedMAD);

    // IQR filter
    SmartDashboard.putNumber("IQR filter", fuzz(yPredictedIQR, flip));
    SmartDashboard.putNumber("IQR filter diff", fuzz(yActual - yPredictedIQR, flip));
    IQRfilterLogEntry.append(yPredictedIQR);

    //Using the entry objects, set the value to a double. The keys are actually "/datatable/X" and "/datatable/Y".
    xEntry.setDouble(yActual);
    yEntry.setDouble(yPredictedSG);

    if(xyCounter <= xy.length-2) {
      xy[xyCounter++] = yActual;
      xy[xyCounter++] = yPredictedSG;
      if(xyCounter >= xy.length) {
        xyEntry.setDoubleArray(xy);
        System.out.println("done");
        // Robot.super.endCompetition();
      }
    }

    // logMemoryUse();
  }
  
  /**
   * a little fuzz to keep the SmartDashboard graph going
   * @param data
   * @param fuzzPhase
   * @return fuzzy data
   */
  double fuzz(double data, double fuzzPhase)
  {
    return data + Math.copySign(Math.ulp(data), fuzzPhase);
  }

  public void calculate()
  {
     // Pin 3-AN- Outputs analog voltage with a scaling factor of (Vcc/512) per inch.
      // A supply of 5V yields ~9.8mV/in. and 3.3V yields ~6.4mV/in.
      var voltsPerInch = edu.wpi.first.wpilibj.RobotController.getVoltage5V() / 512.;
      // SmartDashboard.putNumber("5v", voltsPerInch);
      yActual = m_ultrasonic.getVoltage()/voltsPerInch;

      // // fake an Ultrasonic sensor with a joystick
      // yActual = 4095. * xbox.getRightY();
      // yActual = xbox.getAButtonPressed() ? yActual*1.20 : yActual;
      // yActual = xbox.getAButtonReleased() ? yActual*.75 : yActual;

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
  }

  }

/////////////////////////////


/*
package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Robot extends TimedRobot {
  // distance in inches the robot wants to stay from an object
  private static final double kHoldDistance = 12.0;

  // factor to convert sensor values to a distance in inches
  private static final double kValueToInches = 0.125;

  // proportional speed constant
  private static final double kP = 0.05;

  private static final int kLeftMotorPort = 0;
  private static final int kRightMotorPort = 1;
  private static final int kUltrasonicPort = 0;

  // median filter to discard outliers; filters over 10 samples
  private final MedianFilter m_filter = new MedianFilter(10);

  private final AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(new PWMVictorSPX(kLeftMotorPort),
      new PWMVictorSPX(kRightMotorPort));


  @Override
  public void teleopPeriodic() {
    // sensor returns a value from 0-4095 that is scaled to inches
    // returned value is filtered with a rolling median filter, since ultrasonics
    // tend to be quite noisy and susceptible to sudden outliers
    double currentDistance = m_filter.calculate(m_ultrasonic.getValue()) * kValueToInches;

    // convert distance error to a motor speed
    double currentSpeed = (kHoldDistance - currentDistance) * kP;

    // drive robot
    m_robotDrive.arcadeDrive(currentSpeed, 0);
  }
}

*/
