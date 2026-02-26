package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
	SwerveSubsystem m_swerveDrive;
	TurretSubsystem m_turret;
	IntakeSubsystem m_intake;

	public static Joystick driveJoystick = new Joystick(0);

	public PIDController rotation_test = new PIDController(0.03, 0, 0.0003);

	public RobotContainer() {
		m_swerveDrive = new SwerveSubsystem(new Pose2d(4, 1, new Rotation2d()));
		m_turret = new TurretSubsystem(m_swerveDrive);
		m_intake = new IntakeSubsystem();
		initializePathPlanner();

		//rotation_test.enableContinuousInput(0, 360);
		configureBindings();
	}

	private void configureBindings() {
		m_swerveDrive.setDefaultCommand(
			new RunCommand(
					() -> m_swerveDrive.drive(m_swerveDrive.processVelocityToChassisSpeeds(
							-MathUtil.applyDeadband(driveJoystick.getRawAxis(1) * Constants.kRobotMaxSpeed, 0.1),
							-MathUtil.applyDeadband(driveJoystick.getRawAxis(0) * Constants.kRobotMaxSpeed, 0.1),
							-MathUtil.applyDeadband(driveJoystick.getRawAxis(4) * Constants.kRobotMaxAngularSpeed, 0.1),
							m_swerveDrive.getHeading(),
							true)),
					m_swerveDrive)
		);
	
		
		new JoystickButton(driveJoystick, 1).whileTrue(
			new RunCommand(
				() -> {
					m_turret.turnTurretTo(m_turret.lastDesiredShooterHeading /*- m_turret.lastShooterHeadingError*/ - m_swerveDrive.getHeading().getDegrees());
					System.out.println("Required shooter heading" + (m_turret.lastDesiredShooterHeading - m_swerveDrive.getHeading().getDegrees()));
				}, 
			m_turret)
		);
		
		
		new JoystickButton(driveJoystick, 8).onTrue(
			new InstantCommand(
				() -> m_swerveDrive.resetHeading(0)
			)
		);
		/*new JoystickButton(driveJoystick, 7).onTrue(
			new InstantCommand(
				() -> {m_intake.setMouthRPM(6000);}
			)
		).onFalse(
			new InstantCommand(
				() -> {m_intake.setMouthRPM(0);}
			)
		);*/
		new JoystickButton(driveJoystick, 5).onTrue(m_intake.deployIntake());
		new JoystickButton(driveJoystick, 6).onTrue(m_intake.retractIntake());
		//new JoystickButton(driveJoystick, 5).onTrue(m_intake.toggleIntake());
		
		new JoystickButton(driveJoystick, 3).onTrue(
			new InstantCommand(
				() -> {m_intake.setFeederRPM(6000);},
				m_intake
			)
		).onFalse(
			new InstantCommand(
				() -> {m_intake.setFeederRPM(0);},
				m_intake
			)	
		);

		new JoystickButton(driveJoystick, 4).onTrue(
			new InstantCommand(
				() -> {
					m_intake.setKickerRPM(6000);
					m_intake.setFeederRPM(6000);
				},
				m_intake
			)
		).onFalse(
			new InstantCommand(
				() -> {
					m_intake.setKickerRPM(0);
					m_intake.setFeederRPM(0);
				},
				m_intake
			)	
		);
		
		
		new JoystickButton(driveJoystick, 7).onTrue(
			new InstantCommand(
				() -> {m_turret.setFlywheelRPM(4000);},
				m_intake
			)	
		).onFalse(
			new InstantCommand(
				() -> {m_turret.setFlywheelRPM(0);},
				m_intake
			)	
		);

		new JoystickButton(driveJoystick, 2).onTrue(
			new InstantCommand(
				() -> {
					m_intake.setMouthRPM(5600);
				},
				m_intake)
		).onFalse(
			new InstantCommand(
				() -> {
					m_intake.setMouthRPM(0);
				},
				m_intake
			)
		);
	}

	public Command getAutonomousCommand() {
		PathPlannerPath path;
		try {
			path = PathPlannerPath.fromPathFile("ExamplePath");
			return AutoBuilder.pathfindThenFollowPath(path, Constants.kPathfindingConstraints);

		} catch (Exception e) {
		}
		return null;
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
