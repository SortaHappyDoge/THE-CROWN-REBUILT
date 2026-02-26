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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    public SparkMax m_intakeArm = new SparkMax(Constants.kIntakeArmCANid, MotorType.kBrushless);
    public SparkMaxConfig kIntakeArmConfig = new SparkMaxConfig();
    public SparkMax m_intakeMouth = new SparkMax(Constants.kIntakeMouthCANid, MotorType.kBrushless);
    public SparkMaxConfig kIntakeMouthConfig = new SparkMaxConfig();
    public SparkMax m_feeder = new SparkMax(Constants.kFeederCANid, MotorType.kBrushless);
    public SparkMaxConfig kFeederConfig = new SparkMaxConfig();
    public SparkMax m_kicker /* :3 */ = new SparkMax(Constants.kKickerCANid, MotorType.kBrushless);
    public SparkMaxConfig kKickerConfig = new SparkMaxConfig();


    public boolean isArmDeployed = false;

    public IntakeSubsystem(){
        kIntakeArmConfig.idleMode(IdleMode.kBrake);
        kIntakeArmConfig.smartCurrentLimit(40);
        kIntakeArmConfig.inverted(Constants.kIntakeArmMotorInverted);
        kIntakeArmConfig.voltageCompensation(11);
        kIntakeArmConfig.absoluteEncoder
            .inverted(Constants.kIntakeArmEncoderInverted)
            .positionConversionFactor(Constants.kIntakeArmEncoderPositionFactor)
        ;
        kIntakeArmConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(Constants.kIntakeArmPIDF[0], Constants.kIntakeArmPIDF[1], Constants.kIntakeArmPIDF[2], ClosedLoopSlot.kSlot0)
            .outputRange(-Constants.kIntakeArmMaxSpeedPercentage, Constants.kIntakeArmMaxSpeedPercentage, ClosedLoopSlot.kSlot0)
            .outputRange(-Constants.kIntakeArmMaxSpeedPercentage, Constants.kIntakeArmMaxSpeedPercentage, ClosedLoopSlot.kSlot1)
        ;
        kIntakeArmConfig.closedLoop.feedForward
            .kCos(Constants.kIntakeArmPIDF[3])
        ;
        kIntakeMouthConfig.closedLoopRampRate(Constants.kIntakeArmRampRate);
        m_intakeArm.configure(kIntakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        kIntakeMouthConfig.idleMode(IdleMode.kCoast);
        kIntakeMouthConfig.smartCurrentLimit(40);
        kIntakeMouthConfig.inverted(Constants.kIntakeMouthMotorInverted);
        kIntakeMouthConfig.voltageCompensation(11);
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


        kKickerConfig.smartCurrentLimit(40);
        kKickerConfig.inverted(Constants.kKickerMotorInverted);
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
    }


    @Override
    public void periodic(){
        //System.out.println(getArmPosition());
        //System.out.println(isArmDeployed);
        //System.out.println(m_intakeMouth.getEncoder().getVelocity());
    }

    public double getArmPosition(){
        return m_intakeArm.getAbsoluteEncoder().getPosition();
    }

    public void setMouthRPM(double RPM){
        if(RPM == 0){
            m_intakeMouth.set(0);
        }
        else{
            m_intakeMouth.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
        }
    }

    public void setArmAngle(double angle){
        m_intakeArm.getClosedLoopController().setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void forceArmDown(){
        m_intakeArm.set(Constants.kIntakeArmForceSpeedPercentage);
    }

    public FunctionalCommand deployIntake(){
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

    public FunctionalCommand retractIntake(){
        return new FunctionalCommand(
            () -> {
                setArmAngle(Constants.kIntakeArmMinMax[0]);
                setMouthRPM(0);
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

    public FunctionalCommand toggleIntake(){
        System.out.println(isArmDeployed);
        if(isArmDeployed){
            return retractIntake();
        }
        else{
            return deployIntake();
        }
    }

    public void setFeederRPM(double RPM){
        if(RPM == 0.0){
            m_feeder.set(0);
        }
        else{
            //m_feeder.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
            m_feeder.set(-1);
        }
    }

    public void setKickerRPM(double RPM){
        if(RPM == 0.0){
            m_kicker.set(0);
        }
        else{
            //m_kicker.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
            m_kicker.set(1);
        }
    }
}
