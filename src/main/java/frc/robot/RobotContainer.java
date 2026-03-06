package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.thethriftybot.server.msgHandler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
	SwerveSubsystem m_swerveDrive;
	TurretSubsystem m_turret;
	IntakeSubsystem m_intake;

	public static Joystick driveJoystick = new Joystick(0);
	
	public PIDController rotation_test = new PIDController(0.03, 0, 0.0003);

	private final SendableChooser<Command> autoChooser;
	public PathPlannerPath leftTrenchLeavePath;
	public PathPlannerPath rightTrenchLeavePath;
	public PathPlannerPath leftTrenchEnterPath;
	public PathPlannerPath rightTrenchEnterPath;

	public PathPlannerPath rotationPIDTest;
	
	public RobotContainer() {
		m_swerveDrive = new SwerveSubsystem(new Pose2d(4, 1, Rotation2d.fromDegrees(90)));
		m_turret = new TurretSubsystem(m_swerveDrive);
		m_intake = new IntakeSubsystem(m_turret);

		initializePathPlanner();

		// PATPHLANNER

		NamedCommands.registerCommand("DisableTurret", new InstantCommand(
			() -> {
				m_turret.flywheelOn = false;
				m_turret.isTurretActive = false;
				m_turret.turnTurretTo(180);
				System.out.println("Disabbled turret");
			},
			m_turret
		));
		NamedCommands.registerCommand("DisableIntake", new InstantCommand(
			() -> {
				m_intake.setArmAngle(Constants.kIntakeArmClosedPosition);
				System.out.println("Disabled intake");
			},
			m_intake
			));
		NamedCommands.registerCommand("DeployIntake", m_intake.deployIntakeCmd());	
		NamedCommands.registerCommand("ForceIntakeDown", m_intake.forceIntakeDown());	
		NamedCommands.registerCommand("EnableMouth", new InstantCommand(
			() -> {
				m_intake.setMouthRPM(Constants.kIntakeMouthOnSpeedRPM);
			},
			m_intake
		));	
		NamedCommands.registerCommand("EnableTurret", new InstantCommand(
			() -> {
				m_turret.flywheelOn = true;
				m_turret.isTurretActive = true;
				System.out.println("Enabled turret and flywheel");
			},
			m_turret
		));
		NamedCommands.registerCommand("StartFeeder", new InstantCommand(
			() -> {
				m_intake.toggleCirculation();
				System.out.println("Started circulation");
			},
			m_intake
		));
		NamedCommands.registerCommand("AngleIntake", new InstantCommand(
			() -> {
				m_intake.setArmAngle(Constants.kIntakeArmMiddlePoint);
				System.out.println("Disabled angling the intake, for... reasons...... ;-;");
			},
			m_intake
		));

		try{
			leftTrenchLeavePath = PathPlannerPath.fromPathFile("LeftTrenchLeave");
		    rightTrenchLeavePath = PathPlannerPath.fromPathFile("RightTrenchLeave");
			leftTrenchEnterPath = PathPlannerPath.fromPathFile("LeftTrenchEnter");
			rightTrenchEnterPath = PathPlannerPath.fromPathFile("RightTrenchEnter");
		    rotationPIDTest = PathPlannerPath.fromPathFile("RotationPIDTest");
		}
		catch(Exception e){

		}

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();
		
		SmartDashboard.putData("Timer", 
			new Sendable(){
				public void initSendable(SendableBuilder builder){
					builder.addDoubleProperty("Time Left", () -> DriverStation.getMatchTime(), null);
				}
			} 
		);

		SmartDashboard.putData("Battery Voltage",
			new Sendable(){
				public void initSendable(SendableBuilder builder){
					builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);
				}
			}
		);
		
		SmartDashboard.putData("Battery Amperage",
			new Sendable(){
				public void initSendable(SendableBuilder builder){
					builder.addDoubleProperty("Battery Amperage", () -> RobotController.getInputCurrent(), null);
				}
			}
		);

	}

	private void configureBindings() {
		
		/*m_swerveDrive.setDefaultCommand(
			new RunCommand(
					() -> m_swerveDrive.drive(m_swerveDrive.processVelocityToChassisSpeeds(
							-MathUtil.applyDeadband(driveJoystick.getRawAxis(1) * Constants.kRobotMaxSpeed, 0.1),
							-MathUtil.applyDeadband(driveJoystick.getRawAxis(0) * Constants.kRobotMaxSpeed, 0.1),
							-MathUtil.applyDeadband(driveJoystick.getRawAxis(4) * Constants.kRobotMaxAngularSpeed, 0.1),
							m_swerveDrive.getHeading(),
							true)),
					m_swerveDrive)
		);*/
		m_swerveDrive.setDefaultCommand(
			new RunCommand(
				() -> {
					m_swerveDrive.joystickDrive(
						-MathUtil.applyDeadband(driveJoystick.getRawAxis(1), 0.05),
						-MathUtil.applyDeadband(driveJoystick.getRawAxis(0), 0.05),
						-MathUtil.applyDeadband(driveJoystick.getRawAxis(4), 0.05),
						true
					);
				},
				m_swerveDrive
			)
		);

		new JoystickButton(driveJoystick, 2).onTrue(
			new InstantCommand(
				() -> {m_turret.toggleTurretActive();},
				m_turret
			)
		);
		
		new JoystickButton(driveJoystick, 8).onTrue(
			new InstantCommand(
				() -> m_swerveDrive.resetHeading(180)
			)
		);
		

		/*new JoystickButton(driveJoystick, 5).onTrue(
			new InstantCommand(
				() -> m_intake.toggleIntakeCmd(),
			 	m_intake
			)
		);*/
		new JoystickButton(driveJoystick, 5).onTrue(
			new InstantCommand(
				() -> m_intake.toggleIntakeCmd(),
			 	m_intake
			)
		);
		new JoystickButton(driveJoystick, 6).onTrue(
			new InstantCommand(
				() -> {m_intake.toggleMouth(Constants.kIntakeMouthOnSpeedRPM);},
				m_intake
			)
		);
		new JoystickButton(driveJoystick, 4).onTrue(
			new InstantCommand(
				() -> {m_intake.toggleCirculation();},
				m_intake
			)
		);
		
		
		new JoystickButton(driveJoystick, 2).onTrue(
			new InstantCommand(
				() -> {m_turret.toggleFlywheel();},
				m_turret
			)	
		);

		new JoystickButton(driveJoystick, 3).onTrue(
			m_intake.unjam()
		).onTrue(
			new InstantCommand(
				() -> {
					m_turret.isUnjamming = true;
					m_intake.isUnjamming = true;
				}
			)
		).onFalse(
			new InstantCommand(
				() -> {
					m_turret.isUnjamming = false;
					m_intake.isUnjamming = false;
				}
			)
		);

		new POVButton(driveJoystick, 0).onTrue(
			new InstantCommand(() -> {
				m_turret.flywheelOverride = !m_turret.flywheelOverride;
			},
			m_turret
			)
		);
		new JoystickButton(driveJoystick, 1).onTrue(
			new InstantCommand(() -> {
				m_intake.feederOverride = !m_intake.feederOverride;
			},
			m_intake
			)
		);
		
		new POVButton(driveJoystick, 270).onTrue(
			trenchLeaveEnterRight()
		);
		new POVButton(driveJoystick, 90).onTrue(
			trenchLeaveEnterLeft()
		);
		new POVButton(driveJoystick, 180).onTrue(
			new InstantCommand(
				() -> CommandScheduler.getInstance().cancelAll()
			)
		);
	}



	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	
	public Command trenchLeaveEnterRight(){
		if(DriverStation.getAlliance().get() == Alliance.Blue &&m_turret.currentZone == 0){
			return AutoBuilder.pathfindThenFollowPath(rightTrenchLeavePath, Constants.AutonConstants.kPathfindingConstraints);
		}
		else if(DriverStation.getAlliance().get() == Alliance.Blue && m_turret.currentZone == 1){
			return AutoBuilder.pathfindThenFollowPath(rightTrenchEnterPath, Constants.AutonConstants.kPathfindingConstraints);
		}
		else if(DriverStation.getAlliance().get() == Alliance.Red && m_turret.currentZone == 1){
			return AutoBuilder.pathfindThenFollowPath(rightTrenchEnterPath, Constants.AutonConstants.kPathfindingConstraints);
		}
		else if(DriverStation.getAlliance().get() == Alliance.Red && m_turret.currentZone == 2){
			return AutoBuilder.pathfindThenFollowPath(rightTrenchLeavePath, Constants.AutonConstants.kPathfindingConstraints);
		}
		else{
			return new WaitCommand(0);
		}
	}
	public Command trenchLeaveEnterLeft(){
		if(DriverStation.getAlliance().get() == Alliance.Blue &&m_turret.currentZone == 0){
			return AutoBuilder.pathfindThenFollowPath(leftTrenchLeavePath, Constants.AutonConstants.kPathfindingConstraints);
		}
		else if(DriverStation.getAlliance().get() == Alliance.Blue && m_turret.currentZone == 1){
			return AutoBuilder.pathfindThenFollowPath(leftTrenchEnterPath, Constants.AutonConstants.kPathfindingConstraints);
		}
		else if(DriverStation.getAlliance().get() == Alliance.Red && m_turret.currentZone == 1){
			return AutoBuilder.pathfindThenFollowPath(leftTrenchEnterPath, Constants.AutonConstants.kPathfindingConstraints);
		}
		else if(DriverStation.getAlliance().get() == Alliance.Red && m_turret.currentZone == 2){
			return AutoBuilder.pathfindThenFollowPath(leftTrenchLeavePath, Constants.AutonConstants.kPathfindingConstraints);
		}
		else{
			return new WaitCommand(0);
		}
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
