package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final double kRobotMaxSpeed = 5.85216; // In meters per second
    //public static final double kRobotMaxSpeedAutonomous = 3; // In meters per second
    public static final double kRobotMaxAcceleration = 3; // In meters per second^2
    //public static final double kRobotMaxAccelerationAutonomous = 3; // In meters per second^2
    public static final double kRobotMaxAngularSpeed = 6*Math.PI; // In radians per second !!! It doesn't seem to be radians, need to fix conversion
    //public static final double kRobotMaxAngularSpeedAutonomous = 2*Math.PI; // In radians per second
    public static final double kRobotMaxAngularAcceleration = 6*Math.PI; // In degrees per second^2 !!! Probably wrong
    //public static final double kRobotMaxAngularAccelerationomous = 2*Math.PI; // In degrees per second^2
    public static final boolean kEnableModuleFeedforwards = false;

    //public static final double kWheelDiameter = 0.076; // In meters
    //public static final double kWheelCircumference = kWheelDiameter * Math.PI; // In meters

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
      kRobotMaxSpeed, kRobotMaxAcceleration,
      kRobotMaxAngularSpeed, kRobotMaxAngularAcceleration
	);


	public static final double kGravitationalConstant = 9.81;	// In meters per second^2


    /**
     * @WARNING THESE ARE TEMPORARY VALUES MADE FOR TESTING
     */
    /*
	public static final Translation2d kShooterOffset = new Translation2d(-1,1); // In meters
    public static final double kShooterOffsetDistance = Math.sqrt(Math.pow(kShooterOffset.getX(), 2)+Math.pow(kShooterOffset.getY(), 2));
    public static final double kShooterOffsetAngle = Math.atan2(kShooterOffset.getY(), kShooterOffset.getX()); // In radians
    public static final double[] kShootingDistances = {2, 3}; // Precalculated configs based on the distance to the target TODO: Write a better comment
    public static final double[] kShootingAngles = {80, 70};
    public static final double[] kShootingRPMs = {5000, 5500};
    public static final double[] kAirtimes = {1, 0.8};
    public static final double[] kCircleApproachPID = {0.3, 0, 0}; // P, I and D in that order
    public static final double[] kTurretCorrectionPID = {20, 0, 0}; // P, I and D in that order
    */

	/**
	 * @WARNING these values must be adjusted
	 */
	public static final Translation2d kHubPosBlue = new Translation2d(4.626, 4.035);
    public static final Translation2d kHubPosRed = new Translation2d( 12.896, 4.035);
	public static final double kHubHeight =  1.828;


	public static final double[] kShootingDistanceMinMax = {0, 8};	// First index for minimum, second index for maximum distance in meters
	public static final double[] kPeakProjectileHeightMinMax = {kHubHeight + 0.10, kHubHeight + 2.0};	// First index for minimum, second index for maximum peak projectile height in meters
    // public static final double kDistanceToPeakProjectileHeightConstant = 1.0/10.0;	// Not needed
    /**
	 * @param distance effective distance to target in meters
	 * @return wanted peak height for the projectile
	 */
	public static final double kDistanceToPeakProjectileHeightMapper(double distance){
		return MathUtil.clamp(
            (kShootingDistanceMinMax[1] - distance) *
			(
				(/*kDistanceToPeakProjectileHeightConstant * */(kPeakProjectileHeightMinMax[1] - kPeakProjectileHeightMinMax[0])) /
		 		(kShootingDistanceMinMax[1] - kShootingDistanceMinMax[0])
			)
			+
			kPeakProjectileHeightMinMax[0], 
            kPeakProjectileHeightMinMax[0], kPeakProjectileHeightMinMax[1])
			
		;
	};

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
