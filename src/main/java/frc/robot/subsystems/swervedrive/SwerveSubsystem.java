package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightLocalization;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    public SwerveDrive swerveDrive;
    
    LimelightLocalization localizationLimelight = new LimelightLocalization("limelight-localz");
    
    public SwerveSubsystem(Pose2d startPose){
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.kRobotMaxSpeed, startPose);
        }
        catch(IOException err){
            System.err.println(err.getMessage());
            System.err.println("Cannot find swerve configuration files under: " + swerveJsonDirectory.getName());
            DriverStation.reportError("Cannot find swerve configuration files", err.getStackTrace());
            System.exit(1);
        }

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            swerveDrive.field.setRobotPose(pose);
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            swerveDrive.field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            swerveDrive.field.getObject("path").setPoses(poses);
        });

        
        if(Robot.isSimulation()){
            swerveDrive.setHeadingCorrection(false);
            swerveDrive.setCosineCompensator(false);
        }
    }

    @Override
    public void periodic(){
        localizationLimelight.setRobotOrientation(
            getHeading().getDegrees(), 0/*getYawVelocity().getDegrees() */
        );
        addVisionMeasurement();
        //System.out.println("Heading: "+getHeading().getDegrees());
    }

    public ChassisSpeeds processVelocityToChassisSpeeds(
        double vx, 
        double vy, 
        double vomega,
        Rotation2d robotAngle,
        boolean fieldRelative
        )
    {
        ChassisSpeeds speeds = new ChassisSpeeds();

        /*  
         * TODO:
         *  Implement map border protection for the velocities
         */

        if(fieldRelative) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vomega, robotAngle/*.unaryMinus()*/);
        else speeds = ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, vomega, robotAngle/*.unaryMinus()*/);

        return speeds;
    }
    public void drive(ChassisSpeeds speeds){
        swerveDrive.drive(speeds);
    }

    /**
     * @return blue alliance field origin robot position as a Translation2d object in in (Xmeters, Ymeters)
     */
    public Translation2d getRobotPosition(){
        return swerveDrive.field.getRobotPose().getTranslation();
    }
    /**
     * @return field relative robot speeds as a Translation2d object in (Xmeters per second, Ymeters per second)
     */
    public Translation2d getRobotSpeedsField(){
        return new Translation2d(swerveDrive.getFieldVelocity().vxMetersPerSecond, swerveDrive.getFieldVelocity().vyMetersPerSecond) ;
    }

    /**
     * @return returns robot heading in degrees
     * @WARNING the return angle is CCW positive (hopefully ;-;)
     */
    public Rotation2d getHeading(){
        return swerveDrive.getOdometryHeading();

    }
    /**
     * Sets both the YAGSL gyro and odometry headings to newHeading
     * @param newHeading heading in degrees
     */
    public void resetHeading(double newHeading){
        swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), Rotation2d.fromDegrees(newHeading)));
    }

    public Rotation2d getYawVelocity(){
        return Rotation2d.fromRadians(swerveDrive.getGyroRotation3d().getZ());
    }

    public void addVisionMeasurement(){
        LimelightHelpers.PoseEstimate visionEstimate = localizationLimelight.getBotPoseEstimate(
            0, 0, 0,
            0, 16, 360
        );
        if(visionEstimate == null) return;
        if(visionEstimate.isMegaTag2){
            swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 999999));
        }
        else{
            swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999));
        }
        swerveDrive.addVisionMeasurement(visionEstimate.pose, visionEstimate.timestampSeconds);
    }

    /**
     * @TODO make better exception handling, add a try/catch for
     * AutoBuilder configuration
     */
    public void configureAutoBuilder(){
        if(!Constants.InitializedConstants.hasInitializedRobotConfig){
            return;
        }

        AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity, // MUST BE ROBOT RELATIVE(says the docs)
                (speeds, feedforwards) -> {
                    if (Constants.kEnableModuleFeedforwards) {
                        swerveDrive.drive(
                                speeds,
                                swerveDrive.kinematics.toSwerveModuleStates(speeds),
                                feedforwards.linearForces());
                    } else {
                        swerveDrive.setChassisSpeeds(speeds);
                    }
                },
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID
                ),
                Constants.InitializedConstants.kRobotConfig, // The robot configuration settings in the PathPlanner GUI
                () -> {
                    // To flip the paths based on alliance, field (0,0) always starts from blue side
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Subsystem requirement
        );

    }
}
