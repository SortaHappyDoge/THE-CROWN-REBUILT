package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final double kRobotMaxSpeed = 3; // In meters per second
    //public static final double kRobotMaxSpeedAutonomous = 3; // In meters per second
    public static final double kRobotMaxAcceleration = 3; // In meters per second^2
    //public static final double kRobotMaxAccelerationAutonomous = 3; // In meters per second^2
    public static final double kRobotMaxAngularSpeed = 2*Math.PI; // In radians per second
    //public static final double kRobotMaxAngularSpeedAutonomous = 2*Math.PI; // In radians per second
    public static final double kRobotMaxAngularAcceleration = 2*Math.PI; // In degrees per second^2
    //public static final double kRobotMaxAngularAccelerationomous = 2*Math.PI; // In degrees per second^2
    public static final boolean kEnableModuleFeedforwards = false;

    public static final double kWheelDiameter = 0.076; // In meters
    public static final double kWheelCircumference = kWheelDiameter * Math.PI; // In meters

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
      kRobotMaxSpeed, kRobotMaxAcceleration,
      kRobotMaxAngularSpeed, kRobotMaxAngularAcceleration);


    /**
     * @WARNING THESE ARE TEMPORARY VALUES MADE FOR TESTING
     */
    public static final Translation2d kShooterOffset = new Translation2d(-1,1); // In meters
    public static final double kShooterOffsetDistance = Math.sqrt(Math.pow(kShooterOffset.getX(), 2)+Math.pow(kShooterOffset.getY(), 2));
    public static final double kShooterOffsetAngle = Math.atan2(kShooterOffset.getY(), kShooterOffset.getX()); // In radians
    public static final double[] kShootingDistances = {3, 5, 8}; // Precalculated configs based on the distance to the target TODO: Write a better comment
    public static final double[] kShootingAngles = {80, 70, 60};
    public static final double[] kShootingRPMs = {5000, 5500, 6000};
    public static final double[] kAirtimes = {1, 0.8, 0.75};
    public static final Translation2d kTargetPosBlue = new Translation2d(5, 4);
    public static final Translation2d kTargetPosRed = new Translation2d(11, 4);
    public static final double[] kCircleApproachPID = {1, 0, 0}; // P, I and D in that order

    /**
     * @WARNING these constants MUST be initialized at the start of the robot code.
     * Once initialized, the "hasInitialized" versions of the variables MUST be set
     * to true. When using these constants MUST first be check if the value has
     * been initialized with the "hasInitialized" versions of the variables
     */
    public static class InitializedConstants{
        public static boolean hasInitializedRobotConfig = false;
        public static RobotConfig kRobotConfig;

    }
}
