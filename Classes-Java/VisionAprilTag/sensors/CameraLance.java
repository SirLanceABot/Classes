package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Base class of cameras that supply the pose of the robot.
 * 
 * <p>Defines required methods for camera classes to provide 1-D, 2-D, and 3-D poses.
 * 
 * <p>Provides method to put the robot 3-D pose from the camera on NetworkTables for display purposes.
 */
public abstract class CameraLance
{
    public static final NetworkTableInstance NTinstance = NetworkTableInstance.getDefault();
    private final StructPublisher<Pose3d> botpose;

    CameraLance(String name)
    {
        var tableLogged = NTinstance.getTable(name + "Logged");
        botpose = tableLogged.getStructTopic("pose3d", Pose3d.struct).publish();
    }

    abstract boolean isFresh();
    abstract Pose3d getPose3d();
    abstract Pose2d getPose2d();
    abstract double getTX();
    abstract double getTY();
    abstract void update();

    /**
     * Publish the 3-D pose to NT table "camera name"Logged/pose3d
     * <p>AdvantageScope format
     */
    public void publishPose3d()
    {
        if (isFresh())
        {
            botpose.set(getPose3d());
        }
    }
}
