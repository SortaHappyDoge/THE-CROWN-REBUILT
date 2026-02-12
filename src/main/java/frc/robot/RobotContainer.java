package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
	SwerveSubsystem m_swerveDrive;
	TurretSubsystem m_turret;

	public static Joystick driveJoystick = new Joystick(0);

	public RobotContainer() {
		m_swerveDrive = new SwerveSubsystem(new Pose2d(4, 1, new Rotation2d()));
		m_turret = new TurretSubsystem(m_swerveDrive);
		initializePathPlanner();

		configureBindings();
	}

	private void configureBindings() {
		m_swerveDrive.setDefaultCommand(
				new RunCommand(
						() -> m_swerveDrive.drive(m_swerveDrive.processVelocityToChassisSpeeds(
								-MathUtil.applyDeadband(driveJoystick.getRawAxis(1) * Constants.kRobotMaxSpeed, 0.02),
								-MathUtil.applyDeadband(driveJoystick.getRawAxis(0) * Constants.kRobotMaxSpeed, 0.02),
								-MathUtil.applyDeadband(driveJoystick.getRawAxis(2) * Constants.kRobotMaxAngularSpeed,
										0.02),
								m_swerveDrive.getHeading(),
								true)),
						m_swerveDrive));
		
		/*new JoystickButton(driveJoystick, 1).whileTrue(
			m_turret.LockRobotToTarget(
				new Translation2d(m_swerveDrive.swerveDrive.getFieldVelocity().vxMetersPerSecond, m_swerveDrive.swerveDrive.getFieldVelocity().vxMetersPerSecond),
				m_swerveDrive.getHeading(),
				Constants.kShootingDistances[0],
				Constants.kAirtimes[0],
				1
			)
		);*/

		new JoystickButton(driveJoystick, 8).onTrue(
			new InstantCommand(
				() -> m_swerveDrive.resetHeading(0)
			)
		);
	}

	public Command getAutonomousCommand() {
		PathPlannerPath path;
		try {
			path = PathPlannerPath.fromPathFile("ExamplePath");
			return AutoBuilder.pathfindThenFollowPath(path, Constants.kPathfindingConstraints);
			// return AutoBuilder.pathfindToPose(new Pose2d(2, 5, new Rotation2d(0)),
			// Constants.kPathfindingConstraints);
		} catch (Exception e) {
			// TODO: handle exception
		}
		return null;
		// return Commands.print("No autonomous command configured");
	}

	/**
	 * @TODO make the exception handling and reporting better. Make it
	 *       so not having PathPlanner doesn't stop manual controlling of the robot
	 */
	public void initializePathPlanner() {
		try {
			Constants.InitializedConstants.kRobotConfig = RobotConfig.fromGUISettings();
			Constants.InitializedConstants.hasInitializedRobotConfig = true;
		} catch (Exception err) {
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
