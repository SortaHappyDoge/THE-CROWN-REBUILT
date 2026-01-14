package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.io.IOException;

import org.dyn4j.geometry.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightLocalization;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


public class SwerveSubsystem extends SubsystemBase {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive swerveDrive;
    
    LimelightLocalization localizationLimelight = new LimelightLocalization("limelight-localz");
    
    public SwerveSubsystem(){
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.kRobotMaxSpeed, new Pose2d(4, 1, new Rotation2d(0)));
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
        System.out.println("balls");
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
     * @return returns robot heading in degrees
     * @WARNING the return angle is CCW positive (hopefully ;-;)
     */
    public Rotation2d getHeading(){
        return swerveDrive.getYaw();
    }
    public Rotation2d getYawVelocity(){
        return Rotation2d.fromRadians(swerveDrive.getGyroRotation3d().getZ());
    }

    public void addVisionMeasurement(){
        LimelightHelpers.PoseEstimate visionEstimate = localizationLimelight.getBotPoseEstimate(
            0, 0, 0
        );
        if(visionEstimate == null) return;
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
