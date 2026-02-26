package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurretSubsystem extends SubsystemBase {
    private SwerveSubsystem m_swerveDrive;
    
    public SparkMax m_turretAzimuthMotor = new SparkMax(Constants.kTurretAzimuthMotorCANid, MotorType.kBrushless);
    public SparkMaxConfig kTurretAzimuthMotorConfig = new SparkMaxConfig();
    public SparkMax m_turretPitchMotor = new SparkMax(Constants.kTurretPitchMotorCANid, MotorType.kBrushless);
    public SparkMaxConfig kTurretPitchMotorConfig = new SparkMaxConfig();
    /*public SparkMax m_turretFlywheelMotor = new SparkMax(Constants.kTurretFlywheelMotorCANid, MotorType.kBrushless);
    public SparkMaxConfig kTurretFlywheelMotorConfig = new SparkMaxConfig();*/
    public TalonFX m_turretFlywheelMotor = new TalonFX(Constants.kTurretFlywheelMotorCANid);
    public TalonFXConfiguration kTurretFlywheelConfiguration = new TalonFXConfiguration();


    public double lastDesiredShooterHeading = 0.0;
    public double lastTargetOrientedShooterSpeedX = 0.0;
    public double lastTargetOrientedShooterSpeedY = 0.0;
    public double lastTimeToReachPeak = 0.0;
    public double lastTimeToDescendFromPeak = 0.0;
    public double lastShooterHeadingError = 0.0;
    public double lastRequiredProjectileSpeedX = 0.0;
    public double lastRequiredProjectileSpeedZ = 0.0;
    public double lastRequiredShooterPitch = 0.0;


    public TurretSubsystem(SwerveSubsystem m_swerveDrive){
        this.m_swerveDrive = m_swerveDrive;

        kTurretAzimuthMotorConfig.idleMode(IdleMode.kBrake);
        kTurretAzimuthMotorConfig.smartCurrentLimit(40);
        kTurretAzimuthMotorConfig.inverted(Constants.kTurretAzimuthMotorInverted);
        kTurretAzimuthMotorConfig.encoder
            .positionConversionFactor(Constants.kTurretAzimuthMotorEncoderPositionFactor)
        ;
        kTurretAzimuthMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.kTurretAzimuthMotorPID[0], Constants.kTurretAzimuthMotorPID[1], Constants.kTurretAzimuthMotorPID[2])
            .outputRange(-Constants.kTurretAzimuthMotorMaxSpeedPercentage, Constants.kTurretAzimuthMotorMaxSpeedPercentage)
        ;
        kTurretAzimuthMotorConfig.closedLoopRampRate(Constants.kTurretAzimuthMotorRampRate);
        m_turretAzimuthMotor.configure(kTurretAzimuthMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turretAzimuthMotor.getEncoder().setPosition(180);

        kTurretPitchMotorConfig.smartCurrentLimit(40);
        kTurretPitchMotorConfig.inverted(Constants.kTurretPitchMotorInverted);
        kTurretPitchMotorConfig.encoder
            //.inverted(Constants.kTurretPitchEncoderInverted)
            .positionConversionFactor(Constants.kTurretPitchEncoderPositionFactor)
        ;
        kTurretPitchMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.kTurretPitchPID[0], Constants.kTurretPitchPID[1], Constants.kTurretPitchPID[2])
            .outputRange(-Constants.kTurretPitchMaxSpeedPercentage, Constants.kTurretPitchMaxSpeedPercentage)
        ;
        kTurretPitchMotorConfig.closedLoopRampRate(Constants.kTurretPitchRampRate);
        m_turretPitchMotor.configure(kTurretPitchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        kTurretFlywheelConfiguration.Slot0.withKP(Constants.kTurretFlywheelPIDF[0]);
        kTurretFlywheelConfiguration.Slot0.withKI(Constants.kTurretFlywheelPIDF[1]);
        kTurretFlywheelConfiguration.Slot0.withKD(Constants.kTurretFlywheelPIDF[2]);
        kTurretFlywheelConfiguration.Slot0.withKV(Constants.kTurretFlywheelPIDF[3]);
        m_turretFlywheelMotor.getConfigurator().apply(kTurretFlywheelConfiguration);
    }

    @Override
    public void periodic(){
        Translation2d hubPosition;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            hubPosition = Constants.kHubPosRed;
        }
        else {
            hubPosition = Constants.kHubPosBlue;
        }

        double desiredShooterHeading = CalculateRequiredShooterHeading(m_swerveDrive.getRobotPosition(), hubPosition);
        lastDesiredShooterHeading = desiredShooterHeading;
        double targetOrientedSpeed = CalculateTargetOrientedShooterSpeed(desiredShooterHeading, m_swerveDrive.getRobotSpeedsField());
        double targetDistance = CalculateTargetDistance(m_swerveDrive.getRobotPosition(), hubPosition);
        double totalProjectileAirtime = CalculateProjectileAirTime(0.5, Constants.kDistanceToPeakProjectileHeightMapper(targetDistance), Constants.kHubHeight);
        double effectiveDistance = CalculateEffectiveDistance(
            new Translation2d(lastTargetOrientedShooterSpeedX, lastTargetOrientedShooterSpeedY),
            targetDistance, 
            totalProjectileAirtime
        );
        for(int i = 0; i < 5; i++){
            totalProjectileAirtime = CalculateProjectileAirTime(0.5, Constants.kDistanceToPeakProjectileHeightMapper(effectiveDistance), Constants.kHubHeight);
            effectiveDistance = CalculateEffectiveDistance(
                new Translation2d(lastTargetOrientedShooterSpeedX, lastTargetOrientedShooterSpeedY),
                targetDistance, 
                totalProjectileAirtime
            );
        }
        double projectileSpeed = CalculateRequiredProjectileSpeed(lastTimeToReachPeak, lastTimeToDescendFromPeak, effectiveDistance);
        
        //System.out.println("Calculated desiredShooterHeading: " + lastDesiredShooterHeading);
        /*System.out.println("Calculated peakProjectileHeight: " + Constants.kDistanceToPeakProjectileHeightMapper(targetDistance));
        System.out.println("Calculated targetDistance: " + targetDistance);
        System.out.println("Calculated totalProjectileAirtime: " + totalProjectileAirtime);
        System.out.println("Calculated effectiveDistance: " + effectiveDistance);
        System.out.println("Calculated projectileSpeed: " + projectileSpeed);
        System.out.println("Calculated desiredShooterPitch: " + lastRequiredShooterPitch);
        System.out.println("Calculated corrected shooter heading: " + (desiredShooterHeading+lastShooterHeadingError));*/
        System.out.println("Turret Heading " + m_turretAzimuthMotor.getEncoder().getPosition());
    }

    public void turnTurretTo(double angle){
        m_turretAzimuthMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
    }
    
    public double CalculateRequiredShooterHeading(Translation2d shooterPosition, Translation2d targetPosition){
        double angle = (Math.atan2(targetPosition.getY() - shooterPosition.getY(), targetPosition.getX() - shooterPosition.getX())*180.0)/Math.PI;
        if(angle < 0){
            return angle + 360;
        }
        return angle;
        //return (Math.atan2(targetPosition.getY() - shooterPosition.getY(), targetPosition.getX() - shooterPosition.getX())*180.0)/Math.PI;
    }
    /**
     * Casts the field relative speed of the shooter to parallel and perpendicular lines in the targets direction
     * @param targetHeading heading of the target from the shooter in degrees, CalculateRequiredShooterHeading's return must be passed
     * @param fieldRelativeShooterSpeeds shooter's field relative speeds in the X and Y axis in meters per second
     * @return the total speed of the shooter(must be the same as the total of fieldRelativeShooterSpeeds) in meters per second
     */
    public double CalculateTargetOrientedShooterSpeed(double targetHeading, Translation2d fieldRelativeShooterSpeeds){
        double targetOrientedShooterSpeedX = (-(fieldRelativeShooterSpeeds.getX() * Math.sin((targetHeading*Math.PI)/180.0)) + (fieldRelativeShooterSpeeds.getY() * Math.cos((targetHeading*Math.PI)/180.0)));
        double targetOrientedShooterSpeedY = (fieldRelativeShooterSpeeds.getX() * Math.cos((targetHeading*Math.PI)/180.0)) - (fieldRelativeShooterSpeeds.getY() * Math.sin((targetHeading*Math.PI)/180.0));
        
        lastTargetOrientedShooterSpeedX = targetOrientedShooterSpeedX;
        lastTargetOrientedShooterSpeedY = targetOrientedShooterSpeedY;

        return Math.hypot(targetOrientedShooterSpeedX, targetOrientedShooterSpeedY);
    }
    /**
     * 
     * @param shooterPosition
     * @param targetPosition
     * @return
     */
    public double CalculateTargetDistance(Translation2d shooterPosition, Translation2d targetPosition){
        return Math.hypot(shooterPosition.getY() - targetPosition.getY(), shooterPosition.getX() - targetPosition.getX());
    }
    /**
     * Calculates projectile airtime based on projectile's peak height and 
     * sets lastTimeToReachPeak to timeToReachPeak and lastTimeToDescendFromPeak to lastTimeToDescendFromPeak
     * @param projectileStartHeight the height at which the projectile leaves the shooter in meters
     * @param projectilePeakHeight  highest point the projectile will reach in meters
     * @param targetHeight  target height ground in meters
     * @return projectile's total air time in seconds
     */
    public double CalculateProjectileAirTime(double projectileStartHeight, double projectilePeakHeight, double targetHeight){
        double timeToReachPeak = Math.sqrt((2 * (projectilePeakHeight - projectileStartHeight)) / Constants.kGravitationalConstant);
        lastTimeToReachPeak = timeToReachPeak;
        double timeToDescendFromPeak = Math.sqrt((2 * (projectilePeakHeight - targetHeight)) / Constants.kGravitationalConstant);
        lastTimeToDescendFromPeak = timeToDescendFromPeak;
        
        return timeToReachPeak + timeToDescendFromPeak;
    }
    /*public double AdjustRequiredProjectileSpeeds(Translation2d targetOrientedShooterSpeeds, double targetDistance, double totalCalculatedAirtime){
        double requiredTargetOrientedProjectileSpeedX = lastRequiredProjectileSpeedX - targetOrientedShooterSpeeds.getX();
        double requiredTargetOrientedProjectileSpeedZ =  lastRequiredProjectileSpeedZ - targetOrientedShooterSpeeds.getY();
        lastRequiredTargetOrientedProjectileSpeedX = requiredTargetOrientedProjectileSpeedX;
        lastRequiredTargetOrientedProjectileSpeedZ = requiredTargetOrientedProjectileSpeedZ;
        double shooterHeadingError = (Math.atan2(requiredTargetOrientedProjectileSpeedZ, requiredTargetOrientedProjectileSpeedX)*180.0)/Math.PI;
        lastShooterHeadingError = shooterHeadingError;


        return Math.hypot(requiredTargetOrientedProjectileSpeedX, requiredTargetOrientedProjectileSpeedZ);
    }*/
    /**
     * TODO: Write documentation
     * @param targetOrientedShooterSpeeds
     * @param targetDistance
     * @param totalCalculatedAirtime
     * @return
     */
    public double CalculateEffectiveDistance(Translation2d targetOrientedShooterSpeeds, double targetDistance, double totalCalculatedAirtime){
        double targetErrorDistanceX = targetOrientedShooterSpeeds.getX() * totalCalculatedAirtime;
        double targetErrorDistanceY = targetOrientedShooterSpeeds.getY() * totalCalculatedAirtime;
        
        double effectiveDistance = Math.hypot(targetDistance + targetErrorDistanceY, targetErrorDistanceX);
        double shooterHeadingError = (Math.atan2(targetErrorDistanceX, targetDistance - targetErrorDistanceY)*180.0)/Math.PI;
        lastShooterHeadingError = shooterHeadingError;

        return effectiveDistance;
    }
    /**
     * Calculates the required projectile speed in order to reach the target point and 
     * sets lastRequiredProjectileSpeedX to requiredProjectileSpeedX and lastRequiredProjectileSpeedY to requiredProjectileSpeedY and
     * sets lastRequiredShooterPitch to requiredShooterPitch
     * @param timeToReachPeak
     * @param timeToDescendFromPeak
     * @param targetDistance
     * @return the total speed required for the projectile to reach the target
     */
    public double CalculateRequiredProjectileSpeed(double timeToReachPeak, double timeToDescendFromPeak, double targetDistance){
        double requiredProjectileSpeedX = targetDistance / (timeToReachPeak + timeToDescendFromPeak);
        lastRequiredProjectileSpeedX = requiredProjectileSpeedX;
        double requiredProjectileSpeedZ = timeToReachPeak * Constants.kGravitationalConstant;
        lastRequiredProjectileSpeedZ = requiredProjectileSpeedZ;
        
        double requiredShooterPitch = (Math.atan2(lastRequiredProjectileSpeedZ, lastRequiredProjectileSpeedX)*180.0)/Math.PI;
        lastRequiredShooterPitch = requiredShooterPitch;
        
        return Math.hypot(requiredProjectileSpeedX, requiredProjectileSpeedZ);
    }
    


    public void setFlywheelRPM(double RPM){
        if(RPM == 0.0){
            m_turretFlywheelMotor.set(0);
        }
        else{
            /*VelocityVoltage flywheelVelocity = new VelocityVoltage(0);
            flywheelVelocity.Velocity = RPM/60.0;
            m_turretFlywheelMotor.setControl(flywheelVelocity);
            */
            m_turretFlywheelMotor.set(-1);
        }
    }
}