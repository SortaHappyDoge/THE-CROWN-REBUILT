package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
  SwerveSubsystem m_swerveDrive;
  public Joystick joystick = new Joystick(0);
  public RobotContainer() {
    m_swerveDrive = new SwerveSubsystem();
    initializePathPlanner();

    configureBindings();
  }

  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(
      new RunCommand(
        () -> m_swerveDrive.drive(m_swerveDrive.processVelocityToChassisSpeeds(
          joystick.getRawAxis(1)*Constants.kRobotMaxSpeed, 
          joystick.getRawAxis(0)*Constants.kRobotMaxSpeed,
          joystick.getRawAxis(2)*2*Math.PI,
          m_swerveDrive.getHeading(),
          true
          )), m_swerveDrive)
    );
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile("ExamplePath");
      return AutoBuilder.pathfindThenFollowPath(path, Constants.kPathfindingConstraints);
      //return AutoBuilder.pathfindToPose(new Pose2d(2, 5, new Rotation2d(0)), Constants.kPathfindingConstraints);
    } catch (Exception e) {
      // TODO: handle exception
    }
    return null;
    //return Commands.print("No autonomous command configured");
  }


  /**
   * @TODO make the exception handling and reporting better. Make it
   * so not having PathPlanner doesn't stop manual controlling of the robot
   */
  public void initializePathPlanner(){
    try{
      Constants.InitializedConstants.kRobotConfig = RobotConfig.fromGUISettings();
      Constants.InitializedConstants.hasInitializedRobotConfig = true;
    } 
    catch (Exception err) {
      Constants.InitializedConstants.hasInitializedRobotConfig = false;
      System.err.println(err.getMessage());
      System.err.println("Cannot configure AutoBuilder with the acquired RobotConfig");
      DriverStation.reportError("Cannot configure AutoBuilder", err.getStackTrace());
      System.exit(1);
    }

    m_swerveDrive.configureAutoBuilder();

    PathfindingCommand.warmupCommand();
  }
}
