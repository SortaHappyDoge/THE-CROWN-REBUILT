package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

public final class Constants {
    public static final double kRobotMaxSpeed = 3; // In meters per second
    //public static final double kRobotMaxSpeedAutonomous = 3; // In meters per second
    public static final double kRobotMaxAcceleration = 3; // In meters per second^2
    //public static final double kRobotMaxAccelerationAutonomous = 3; // In meters per second^2
    public static final double kRobotMaxAngularSpeed = 2*180; // In degrees per second
    //public static final double kRobotMaxAngularSpeedAutonomous = 2*Math.PI; // In degrees per second
    public static final double kRobotMaxAngularAcceleration = 2*Math.PI; // In degrees per second^2
    //public static final double kRobotMaxAngularAccelerationomous = 2*Math.PI; // In degrees per second^2
    public static final boolean kEnableModuleFeedforwards = false;

    public static final double kWheelDiameter = 0.076; // In meters
    public static final double kWheelCircumference = kWheelDiameter * Math.PI; // In meters

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
      kRobotMaxSpeed, kRobotMaxAcceleration,
      kRobotMaxAngularSpeed, kRobotMaxAngularAcceleration);

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
