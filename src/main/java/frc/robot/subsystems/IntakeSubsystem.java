package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    TurretSubsystem m_turret;
	
	public SparkMax m_intakeArm = new SparkMax(Constants.kIntakeArmCANid, MotorType.kBrushless);
    public SparkMaxConfig kIntakeArmConfig = new SparkMaxConfig();
    public SparkMax m_intakeMouth = new SparkMax(Constants.kIntakeMouthCANid, MotorType.kBrushless);
    public SparkMaxConfig kIntakeMouthConfig = new SparkMaxConfig();
    public SparkMax m_feeder = new SparkMax(Constants.kFeederCANid, MotorType.kBrushless);
    public SparkMaxConfig kFeederConfig = new SparkMaxConfig();
    public SparkMax m_kicker /* :3 */ = new SparkMax(Constants.kKickerCANid, MotorType.kBrushless);
    public SparkMaxConfig kKickerConfig = new SparkMaxConfig();


    public boolean isArmDeployed = false;
    public boolean hasArmDeployed = false;
	public boolean isMouthOn = false;
	public boolean isFeederOn = false;
    public boolean feederOverride = false;
	public boolean isKickerOn = false;
    public boolean isUnjamming = false;

    double desiredIntakeArmAngle = Constants.kIntakeArmClosedPosition;

    double lastP = 0;
    double lastD = 0; 
    double lastF = 0;
    double lastRPM = 0;
    boolean lastRun = false;
    double lastCurrent = 0.0;


    public IntakeSubsystem(TurretSubsystem m_turret){
		this.m_turret = m_turret;

        kIntakeArmConfig.idleMode(IdleMode.kBrake);
        kIntakeArmConfig.smartCurrentLimit(40);
        kIntakeArmConfig.inverted(Constants.kIntakeArmMotorInverted);
        kIntakeArmConfig.voltageCompensation(10);
        kIntakeArmConfig.absoluteEncoder
            .inverted(Constants.kIntakeArmEncoderInverted)
            .positionConversionFactor(Constants.kIntakeArmEncoderPositionFactor)
        ;
        kIntakeArmConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(Constants.kIntakeArmPIDF[0], Constants.kIntakeArmPIDF[1], Constants.kIntakeArmPIDF[2], ClosedLoopSlot.kSlot0)
            .outputRange(-Constants.kIntakeArmMaxSpeedPercentage, Constants.kIntakeArmMaxSpeedPercentage, ClosedLoopSlot.kSlot0)
            .pid(Constants.kIntakeArmDeployedPID[0], Constants.kIntakeArmDeployedPID[1], Constants.kIntakeArmDeployedPID[2], ClosedLoopSlot.kSlot1)
            .outputRange(-Constants.kIntakeArmMaxSpeedPercentage, Constants.kIntakeArmMaxSpeedPercentage, ClosedLoopSlot.kSlot1)
            .pid(Constants.kintakeArmForcePID[0], Constants.kintakeArmForcePID[1], Constants.kintakeArmForcePID[2], ClosedLoopSlot.kSlot2)
            .outputRange(0, Constants.kIntakeArmForceMaxSpeedPercentage, ClosedLoopSlot.kSlot2)
        ;
        kIntakeArmConfig.closedLoop.feedForward
            .kCos(Constants.kIntakeArmPIDF[3])
        ;
        kIntakeMouthConfig.closedLoopRampRate(Constants.kIntakeArmRampRate);
        m_intakeArm.configure(kIntakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        kIntakeMouthConfig.idleMode(IdleMode.kCoast);
        kIntakeMouthConfig.smartCurrentLimit(40);
        kIntakeMouthConfig.inverted(Constants.kIntakeMouthMotorInverted);
        kIntakeMouthConfig.voltageCompensation(10);
        kIntakeMouthConfig.encoder
            //.inverted(Constants.kIntakeMouthEncoderInverted)
            .positionConversionFactor(Constants.kIntakeMouthEncoderPositionFactor)
            .velocityConversionFactor(Constants.kIntakeMouthEncoderVelocityFactor)
        ;
        kIntakeMouthConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.kIntakeMouthPIDF[0], Constants.kIntakeMouthPIDF[1], Constants.kIntakeMouthPIDF[2])
            .outputRange(-Constants.kIntakeMouthMaxSpeedPercentage, Constants.kIntakeMouthMaxSpeedPercentage)
        ;
        kIntakeMouthConfig.closedLoop.feedForward
            .kV(Constants.kIntakeMouthPIDF[3])
        ;
        kIntakeMouthConfig.closedLoopRampRate(Constants.kIntakeMouthRampRate);
        m_intakeMouth.configure(kIntakeMouthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        kFeederConfig.idleMode(IdleMode.kCoast);
        kFeederConfig.smartCurrentLimit(40);
        kFeederConfig.inverted(Constants.kFeederMotorInverted);
        kFeederConfig.voltageCompensation(10);
        kFeederConfig.encoder
            //.inverted(Constants.kFeederEncoderInverted)
            .positionConversionFactor(Constants.kFeederEncoderPositionFactor)
            .velocityConversionFactor(Constants.kFeederEncoderVelocityFactor)
        ;
        kFeederConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.kFeederPIDF[0], Constants.kFeederPIDF[1], Constants.kFeederPIDF[2])
            .outputRange(-Constants.kFeederMaxSpeedPercentage, Constants.kFeederMaxSpeedPercentage)
        ;
        kFeederConfig.closedLoop.feedForward
            .kV(Constants.kFeederPIDF[3])
        ;
        kFeederConfig.closedLoopRampRate(Constants.kFeederRampRate);
        m_feeder.configure(kFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kFeederConfig.idleMode(IdleMode.kCoast);


        kKickerConfig.idleMode(IdleMode.kCoast);
        kKickerConfig.smartCurrentLimit(40);
        kKickerConfig.inverted(Constants.kKickerMotorInverted);
        kKickerConfig.voltageCompensation(10);
        kKickerConfig.encoder
            //.inverted(Constants.kKickerEncoderInverted)
            .positionConversionFactor(Constants.kKickerEncoderPositionFactor)
            .velocityConversionFactor(Constants.kKickerEncoderVelocityFactor)
        ;
        kKickerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.kKickerPIDF[0], Constants.kKickerPIDF[1], Constants.kKickerPIDF[2])
            .outputRange(-Constants.kKickerMaxSpeedPercentage, Constants.kKickerMaxSpeedPercentage)
        ;
        kKickerConfig.closedLoop.feedForward
            .kV(Constants.kKickerPIDF[3])
        ;
        kKickerConfig.closedLoopRampRate(Constants.kKickerRampRate);
        m_kicker.configure(kKickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
        //dashboardPIDcontrolInitIntakeMouth();
        //dashboardPIDcontrolInitIntakeArm();
        //dashboardPIDcontrolInitKicker();
        //dashboardPIDcontrolInitFeeder();
    }

    

    @Override
    public void periodic(){
		updateFeeder();
		updateKicker();
        //updateIntakeArm();


        SmartDashboard.putBoolean("IsFeederOn", isFeederOn);
        SmartDashboard.putBoolean("IsMouthOn", isMouthOn);
        SmartDashboard.putBoolean("FEEDER OVERRIDE", feederOverride);



		//System.out.println("isArmDeployed: " + isArmDeployed);
        //dashboardPIDcontrolLoopIntakeMouth();
        //dashboardPIDcontrolLoopFeeder();
    }

    public double getArmPosition(){
        return m_intakeArm.getAbsoluteEncoder().getPosition();
    }


	public void toggleCirculation(){
		if(isFeederOn){
			isFeederOn = false;
			isKickerOn = false;			
		}
		else{
			isFeederOn = true;
			isKickerOn = true;
		}
	}

    public void setMouthRPM(double RPM){
        if(RPM == 0){
            m_intakeMouth.set(0);
        }
        else{
            m_intakeMouth.getClosedLoopController().setSetpoint(MathUtil.clamp(RPM, -Constants.kIntakeMouthMaxSpeedRPM, Constants.kIntakeMouthMaxSpeedRPM), ControlType.kVelocity);
        }
    }

	public void toggleMouth(double RPM){
		if(isMouthOn){
			setMouthRPM(0);
		}else{
			setMouthRPM(RPM);
		}
		isMouthOn = !isMouthOn;
	}

    public void setArmAngle(double angle){
        m_intakeArm.getClosedLoopController().setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void forceArmDown(){
        //m_intakeArm.set(Constants.kIntakeArmForceSpeedPercentage);
        m_intakeArm.getClosedLoopController().setSetpoint(Constants.kIntakeArmMinMax[1], ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public FunctionalCommand deployIntakeCmd(){
        return new FunctionalCommand(
            () -> {
                setArmAngle(Constants.kIntakeArmMinMax[1]); 
                System.out.println("Deploying intake");
            },
            () -> {
                // System.out.println("Arm rot: " + getArmPosition());
                // System.out.println("Target: " + Constants.kIntakeArmMinMax[1]);
            }, 
            interrupted -> {
                forceArmDown();
                isArmDeployed = true;
                System.out.println("Intake deployed");
            },
            () -> {return Math.abs(Constants.kIntakeArmMinMax[1] - getArmPosition()) < Constants.kIntakeArmTolerance;},
			this
        );
    }

    public FunctionalCommand retractIntakeCmd(){
        return new FunctionalCommand(
            () -> {
                setArmAngle(Constants.kIntakeArmMinMax[0]);
                isMouthOn = false;
                System.out.println("Retracting intake");
            },
            () -> {
                // System.out.println("Arm rot: " + getArmPosition());
                // System.out.println("Target: " + Constants.kIntakeArmMinMax[0]);
            }, 
            interrupted -> { 
                isArmDeployed = false; 
                m_intakeArm.set(0); 
                System.out.println("Intake retracted"); 
            }, 
            () -> {return (getArmPosition() - Constants.kIntakeArmMinMax[0]) < Constants.kIntakeArmTolerance;},
			this
        );
    }
    public FunctionalCommand forceIntakeDown(){
        return new FunctionalCommand(
            () -> {
                intakeForcePID(Constants.kIntakeArmMinMax[1]);
                System.out.println("Deploying intake");
            },
            () -> {
                // System.out.println("Arm rot: " + getArmPosition());
                // System.out.println("Target: " + Constants.kIntakeArmMinMax[1]);
            }, 
            interrupted -> {
                forceArmDown();
                isArmDeployed = true;
                System.out.println("Intake deployed");
            },
            () -> {return Math.abs(Constants.kIntakeArmMinMax[1] - getArmPosition()) < Constants.kIntakeArmTolerance*10;},
			this
        );
    }
    public void intakeForcePID(double angle){
        m_intakeArm.getClosedLoopController().setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot2);
    }
    
    public void toggleIntakeCmd(){
        if(isArmDeployed){
            CommandScheduler.getInstance().schedule(retractIntakeCmd());
        }
        else{
            CommandScheduler.getInstance().schedule(deployIntakeCmd());
        }
    }
    /*public void toggleIntake(){
        if(isArmDeployed){
            desiredIntakeArmAngle = Constants.kIntakeArmMiddlePoint;
            isArmDeployed = false;
            hasArmDeployed = false;
        }
        else{
            desiredIntakeArmAngle = Constants.kIntakeArmMinMax[1];
            isArmDeployed = true;
        }
    }*/

    public void setFeederRPM(double RPM){
        if(RPM == 0.0){
            m_feeder.set(0);
        }
        else{
            m_feeder.getClosedLoopController().setSetpoint(MathUtil.clamp(RPM, -Constants.kFeederMaxSpeedRPM, Constants.kFeederMaxSpeedRPM), ControlType.kVelocity);
        }
    }

	public void updateFeeder(){
        if(feederOverride){
            setFeederRPM(Constants.kFeederOnSpeedRPM);
            return;
        }
        if(isUnjamming){
            return;
        }
		if(!isFeederOn){
			setFeederRPM(0);
		}
		else if(m_turret.flywheelReady){
			setFeederRPM(Constants.kFeederOnSpeedRPM);
		}
		else{
			setFeederRPM(Constants.kFeederStandbySpeedRPM);
		}
	}

    public void setKickerRPM(double RPM){
        if(RPM == 0.0){
            m_kicker.set(0);
        }
        else{
            m_kicker.getClosedLoopController().setSetpoint(MathUtil.clamp(RPM, -Constants.kKickerMaxSpeedRPM, Constants.kKickerMaxSpeedRPM), ControlType.kVelocity);
        }
    }

	public void updateKicker(){
        if(feederOverride){
            setKickerRPM(Constants.kKickerOnSpeedRPM);
            return;
        }
		if(!isKickerOn){
			setKickerRPM(0);
		}
		else if(m_turret.flywheelReady){
			setKickerRPM(Constants.kKickerOnSpeedRPM);
		}
		else{
			setKickerRPM(Constants.kKickerStandbySpeedRPM);
		}
	}

    /*public void updateIntakeArm(){
        if(!isArmDeployed || hasArmDeployed){
            setArmAngle(desiredIntakeArmAngle);
        }
        else if(isArmDeployed && hasArmDeployed){
            forceArmDown();
        }
        else if(isArmDeployed && !hasArmDeployed){
            if((m_intakeArm.getEncoder().getPosition() + Constants.kIntakeArmTolerance) > Constants.kIntakeArmMinMax[1]){
                hasArmDeployed = true;
            }
        }
    }*/

    public Command unjam(){
        return new InstantCommand( 
            () -> {
                setFeederRPM(Constants.kFeederUnjamSpeedRPM);
                setKickerRPM(Constants.kKickerUnjamSpeedRPM);
                m_turret.m_turretFlywheelMotor.set(Constants.kFlywheelUnjamSpeedPercentage);
            },
            this, m_turret
        );
    }


    public void dashboardPIDcontrolInitIntakeMouth(){
        lastP = Constants.kIntakeMouthPIDF[0];
        lastD = Constants.kIntakeMouthPIDF[2];
        lastF = Constants.kIntakeMouthPIDF[3];

        SmartDashboard.putNumber("Intake/MouthP", lastP);
        SmartDashboard.putNumber("Intake/MouthD", lastD);
        SmartDashboard.putNumber("Intake/MouthF", lastF);
        SmartDashboard.putNumber("Intake/MouthRPM", 0);
        SmartDashboard.putBoolean("Intake/RunMouth", false);
    }
    public void dashboardPIDcontrolLoopIntakeMouth(){
        if(!DriverStation.isFMSAttached()){
            final double p = SmartDashboard.getNumber("Intake/MouthP", lastP);
            final double d = SmartDashboard.getNumber("Intake/MouthD", lastD);
            final double f = SmartDashboard.getNumber("Intake/MouthF", lastF);
            final double RPM = SmartDashboard.getNumber("Intake/MouthRPM", 0);
            final boolean run = SmartDashboard.getBoolean("Intake/RunMouth", false);

            SmartDashboard.putNumber("MouthRPM", m_intakeMouth.getEncoder().getVelocity()/Constants.kIntakeMouthEncoderVelocityFactor);
            SmartDashboard.putNumber("MouthCurrent", m_intakeMouth.getOutputCurrent());

            if(p != lastP || d != lastD || f != lastF || RPM != lastRPM || run != lastRun){
                lastP = p; lastD = d; lastF = f; lastRPM = RPM; lastRun = run;
                kIntakeMouthConfig.closedLoop.pid(p, 0, d);
                kIntakeMouthConfig.closedLoop.feedForward.kV(f);
                m_intakeMouth.configure(kIntakeMouthConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                if(run){
                    m_intakeMouth.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
                }
                else{
                    m_intakeMouth.set(0);
                }
            }
        }
    }
    
    public void dashboardPIDcontrolInitKicker(){
        lastP = Constants.kKickerPIDF[0];
        lastD = Constants.kKickerPIDF[2];
        lastF = Constants.kKickerPIDF[3];

        SmartDashboard.putNumber("Intake/KickerP", lastP);
        SmartDashboard.putNumber("Intake/KickerD", lastD);
        SmartDashboard.putNumber("Intake/KickerF", lastF);
        SmartDashboard.putNumber("Intake/KickerRPM", 0);
        SmartDashboard.putBoolean("Intake/RunKicker", false);
    }
    public void dashboardPIDcontrolLoopKicker(){
        if(!DriverStation.isFMSAttached()){
            final double p = SmartDashboard.getNumber("Intake/KickerP", lastP);
            final double d = SmartDashboard.getNumber("Intake/KickerD", lastD);
            final double f = SmartDashboard.getNumber("Intake/KickerF", lastF);
            final double RPM = SmartDashboard.getNumber("Intake/KickerRPM", 0);
            final boolean run = SmartDashboard.getBoolean("Intake/RunKicker", false);

            SmartDashboard.putNumber("KickerRPM", m_kicker.getEncoder().getVelocity()/Constants.kKickerEncoderVelocityFactor);
            SmartDashboard.putNumber("KickerCurrent", m_kicker.getOutputCurrent());

            if(p != lastP || d != lastD || f != lastF || RPM != lastRPM || run != lastRun){
                lastP = p; lastD = d; lastF = f; lastRPM = RPM; lastRun = run;
                kKickerConfig.closedLoop.pid(p, 0, d);
                kKickerConfig.closedLoop.feedForward.kV(f);
                m_kicker.configure(kKickerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                if(run){
                    m_kicker.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
                }
                else{
                    m_kicker.set(0);
                }
            }
        }
    }
    

    public void dashboardPIDcontrolInitFeeder(){
        lastP = Constants.kFeederPIDF[0];
        lastD = Constants.kFeederPIDF[2];
        lastF = Constants.kFeederPIDF[3];
    
        SmartDashboard.putNumber("Intake/FeederP", lastP);
        SmartDashboard.putNumber("Intake/FeederD", lastD);
        SmartDashboard.putNumber("Intake/FeederF", lastF);
        SmartDashboard.putNumber("Intake/FeederRPM", 0);
        SmartDashboard.putBoolean("Intake/RunFeeder", false);
    }
    public void dashboardPIDcontrolLoopFeeder(){
        if(!DriverStation.isFMSAttached()){
            final double p = SmartDashboard.getNumber("Intake/FeederP", lastP);
            final double d = SmartDashboard.getNumber("Intake/FeederD", lastD);
            final double f = SmartDashboard.getNumber("Intake/FeederF", lastF);
            final double RPM = SmartDashboard.getNumber("Intake/FeederRPM", 0);
            final boolean run = SmartDashboard.getBoolean("Intake/RunFeeder", false);
    
            SmartDashboard.putNumber("FeederRPM", m_feeder.getEncoder().getVelocity()/Constants.kFeederEncoderVelocityFactor);
            SmartDashboard.putNumber("FeederCurrent", m_feeder.getOutputCurrent());

            if(p != lastP || d != lastD || f != lastF || RPM != lastRPM || run != lastRun){
                lastP = p; lastD = d; lastF = f; lastRPM = RPM; lastRun = run;
                kFeederConfig.closedLoop.pid(p, 0, d);
                kFeederConfig.closedLoop.feedForward.kV(f);
                m_feeder.configure(kFeederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                if(run){
                    m_feeder.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
                }
                else{
                    m_feeder.set(0);
                }
            }
        }
    }

	public void dashboardPIDcontrolInitIntakeArm(){
        lastP = Constants.kIntakeArmPIDF[0];
        lastD = Constants.kIntakeArmPIDF[2];
        lastF = Constants.kIntakeArmPIDF[3];
    
        SmartDashboard.putNumber("Intake/ArmP", lastP);
        SmartDashboard.putNumber("Intake/ArmD", lastD);
        SmartDashboard.putNumber("Intake/ArmF", lastF);
        SmartDashboard.putNumber("Intake/ArmPose", 0);
        SmartDashboard.putBoolean("Intake/RunArm", false);
    }
    public void dashboardPIDcontrolLoopIntakeArm(){
        if(!DriverStation.isFMSAttached()){
            final double p = SmartDashboard.getNumber("Intake/ArmP", lastP);
            final double d = SmartDashboard.getNumber("Intake/ArmD", lastD);
            final double f = SmartDashboard.getNumber("Intake/ArmF", lastF);
            final double pose = SmartDashboard.getNumber("Intake/ArmPose", 0);
            final boolean run = SmartDashboard.getBoolean("Intake/RunArm", false);
    
            SmartDashboard.putNumber("ArmPosition", m_intakeArm.getAbsoluteEncoder().getPosition() / Constants.kIntakeArmEncoderPositionFactor);
            SmartDashboard.putNumber("ArmCurrent", m_intakeArm.getOutputCurrent());

            if(p != lastP || d != lastD || f != lastF || pose != lastRPM || run != lastRun){
                lastP = p; lastD = d; lastF = f; lastRPM = pose; lastRun = run;
                kIntakeArmConfig.closedLoop.pid(p, 0, d);
                kIntakeArmConfig.closedLoop.feedForward.kV(f);
                m_intakeArm.configure(kIntakeArmConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                if(run){
                    //setArmAngle(Constants.kIntakeArmMinMax[1]);
					m_intakeArm.getClosedLoopController().setSetpoint(pose, ControlType.kPosition);
                }
                else{
                    //setArmAngle(Constants.kIntakeArmMinMax[0]);
					m_intakeArm.set(0);
                }
            }
        }
    }
}
