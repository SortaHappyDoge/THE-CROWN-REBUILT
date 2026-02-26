package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    // Stuff related to the drivetrain :D
    public static final int kFrontLeftDriveMotorCANid = 4;
    public static final int kFrontLeftSteerMotorCANid = 5;
    public static final int kFrontLeftAzimuthEncoderCANid = 6;
    public static final int kRearLeftDriveMotorCANid = 8;
    public static final int kRearLeftSteerMotorCANid = 9;
    public static final int kRearLeftAzimuthEncoderCANid = 10;
    public static final int kRearRightDriveMotorCANid = 12;
    public static final int kRearRightSteerMotorCANid = 13;
    public static final int kRearRightAzimuthEncoderCANid = 14;
    public static final int kFrontRightDriveMotorCANid = 16;
    public static final int kFrontRightSteerMotorCANid = 17;
    public static final int kFrontRightAzimuthEncoderCANid = 18;




    public static final double kRobotMaxSpeed = 5.85216; // In meters per second
    //public static final double kRobotMaxSpeedAutonomous = 3; // In meters per second
    public static final double kRobotMaxAcceleration = 3; // In meters per second^2
    //public static final double kRobotMaxAccelerationAutonomous = 3; // In meters per second^2
    public static final double kRobotMaxAngularSpeed = 12*Math.PI; // In radians per second !!! It doesn't seem to be radians, need to fix conversion
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

    // Stuff related to the turret :C
    public static final int kTurretAzimuthMotorCANid = 20;
    public static final boolean kTurretAzimuthMotorInverted = false;
    public static final boolean kTurretAzimuthMotorEncoderInverted = false;
    public static final double kTurretAzimuthMotorEncoderPositionFactor = 27.77/2.0;
    public static final double kTurretAzimuthMotorMaxSpeedPercentage = 0.5;
    public static final double[] kTurretAzimuthMotorPID = {0.015, 0 , 1.5};
    public static final double kTurretAzimuthMotorRampRate = 0.3;
    
    public static final int kTurretPitchMotorCANid = 21;
    public static final boolean kTurretPitchMotorInverted = false;
    public static final boolean kTurretPitchEncoderInverted = false;
    public static final double kTurretPitchEncoderPositionFactor = 360;
    public static final double kTurretPitchMaxSpeedPercentage = 0.2;
    public static final double[] kTurretPitchPID = {0.015, 0 , 1.5};
    public static final double kTurretPitchRampRate = 0.3;
    
    public static final int kTurretFlywheelMotorCANid = 22;
    public static final boolean kTurretFlywheelMotorInverted = false;
    public static final boolean kTurretFlywheelEncoderInverted = false;
    public static final double kTurretFlywheelEncoderPositionFactor = 360;
    public static final double kTurretFlywheelEncoderVelocityFactor = 360;
    public static final double kTurretFlywheelMaxSpeedPercentage = 1;
    public static final double[] kTurretFlywheelPIDF = {0.015, 0 , 1.5, 0.0};
    public static final double kTurretFlywheelRampRate = 0;


    // Stuff related to the intake :3
    public static final int kIntakeArmCANid = 24;
    public static final boolean kIntakeArmMotorInverted = true;
    public static final boolean kIntakeArmEncoderInverted = true;
    public static final double kIntakeArmEncoderPositionFactor = 360;
    public static final double kIntakeArmMaxSpeedPercentage = 0.4;
    public static final double kIntakeArmForceSpeedPercentage = 0.0;
    public static final double[] kIntakeArmPIDF = {0.03, 0.0, 0.0, 0.52};  // P, I, D and F in that order
    public static final double kIntakeArmRampRate = 1;
    public static final double kIntakeArmEncoderOffset = 0;
    public static final double kIntakeArmClosedPosition = 65;    // In degrees
    public static final double[] kIntakeArmMinMax = {kIntakeArmClosedPosition, 170.0};
    public static final double kIntakeArmTolerance = 5;   // In degrees

    public static final int kIntakeMouthCANid = 61;
    public static final boolean kIntakeMouthMotorInverted = false;
    public static final boolean kIntakeMouthEncoderInverted = false;
    public static final double kIntakeMouthEncoderPositionFactor = 360.0/4;
    public static final double kIntakeMouthEncoderVelocityFactor = 360.0/4;
    public static final double kIntakeMouthMaxSpeedPercentage = 1;
    public static final double[] kIntakeMouthPIDF = {0.00000, 0, 0.000, 0.0017}; // P, I, D and F in that order
    public static final double kIntakeMouthRampRate = 0.3;
    
    public static final int kFeederCANid = 26;
    public static final boolean kFeederMotorInverted = false;
    public static final boolean kFeederEncoderInverted = false;
    public static final double kFeederEncoderPositionFactor = 360.0/4;
    public static final double kFeederEncoderVelocityFactor = 360.0/4;
    public static final double kFeederMaxSpeedPercentage = 0.8;
    public static final double[] kFeederPIDF = {0.0006, 0, 0.000, 0.0001}; // P, I, D and F in that order
    public static final double kFeederRampRate = 0.3;
    
    public static final int kKickerCANid = 27;
    public static final boolean kKickerMotorInverted = false;
    public static final boolean kKickerEncoderInverted = false;
    public static final double kKickerEncoderPositionFactor = 360.0/4;
    public static final double kKickerEncoderVelocityFactor = 360.0/4;
    public static final double kKickerMaxSpeedPercentage = 1;
    public static final double[] kKickerPIDF = {0.0006, 0, 0.000, 0.00015}; // P, I, D and F in that order
    public static final double kKickerRampRate = 0.1;
    
    
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
