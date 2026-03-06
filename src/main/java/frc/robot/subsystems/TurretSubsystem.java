package frc.robot.subsystems;

import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurretSubsystem extends SubsystemBase {
    private SwerveSubsystem m_swerveDrive;

    public SparkMax m_turretAzimuthMotor = new SparkMax(Constants.kTurretAzimuthMotorCANid, MotorType.kBrushless);
    public SparkMaxConfig kTurretAzimuthMotorConfig = new SparkMaxConfig();
    public SparkMax m_turretPitchMotor = new SparkMax(Constants.kTurretPitchMotorCANid, MotorType.kBrushless);
    public SparkMaxConfig kTurretPitchMotorConfig = new SparkMaxConfig();
    /*
     * public SparkMax m_turretFlywheelMotor = new
     * SparkMax(Constants.kTurretFlywheelMotorCANid, MotorType.kBrushless);
     * public SparkMaxConfig kTurretFlywheelMotorConfig = new SparkMaxConfig();
     */
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

    public Translation2d currentTarget = new Translation2d();
    public int currentZone;
    public double desiredFlywheelRPM = 0;
    public boolean flywheelOn = false;
    public boolean flywheelReady = false;
    public boolean isTurretActive = false;
    public boolean isTargetAcquired = false;

    public boolean isUnjamming = false;
    public boolean flywheelOverride = false;

    double lastP = 0;
    double lastD = 0;
    double lastF = 0;
    double lastRPM = 0;
    double lastFlywheelMultiplier = Constants.kFlywheelEffectiveRadius;
    double lastPosition = 0;
    boolean lastRun = false;

    public TurretSubsystem(SwerveSubsystem m_swerveDrive) {
        this.m_swerveDrive = m_swerveDrive;
        
        kTurretAzimuthMotorConfig.idleMode(IdleMode.kBrake);
        kTurretAzimuthMotorConfig.smartCurrentLimit(70);
        kTurretAzimuthMotorConfig.inverted(Constants.kTurretAzimuthMotorInverted);
        kTurretAzimuthMotorConfig.encoder
                .positionConversionFactor(Constants.kTurretAzimuthMotorEncoderPositionFactor);
        kTurretAzimuthMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Constants.kTurretAzimuthMotorPID[0], Constants.kTurretAzimuthMotorPID[1],
                        Constants.kTurretAzimuthMotorPID[2])
                .outputRange(-Constants.kTurretAzimuthMotorMaxSpeedPercentage,
                        Constants.kTurretAzimuthMotorMaxSpeedPercentage);
        kTurretAzimuthMotorConfig.closedLoopRampRate(Constants.kTurretAzimuthMotorRampRate);
        m_turretAzimuthMotor.configure(kTurretAzimuthMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turretAzimuthMotor.getEncoder().setPosition(180);

        kTurretPitchMotorConfig.smartCurrentLimit(40);
        kTurretPitchMotorConfig.inverted(Constants.kTurretPitchMotorInverted);
        kTurretPitchMotorConfig.encoder
                // .inverted(Constants.kTurretPitchEncoderInverted)
                .positionConversionFactor(Constants.kTurretPitchEncoderPositionFactor);
        kTurretPitchMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Constants.kTurretPitchPID[0], Constants.kTurretPitchPID[1], Constants.kTurretPitchPID[2])
                .outputRange(-Constants.kTurretPitchMaxSpeedPercentage, Constants.kTurretPitchMaxSpeedPercentage);
        kTurretPitchMotorConfig.closedLoopRampRate(Constants.kTurretPitchRampRate);
        m_turretPitchMotor.configure(kTurretPitchMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turretPitchMotor.getEncoder().setPosition(Constants.kTurretPitchMinMaxDegrees[1]);


        CurrentLimitsConfigs flywheelCurrentLimit = new CurrentLimitsConfigs();
        flywheelCurrentLimit.SupplyCurrentLimitEnable = true;
        flywheelCurrentLimit.SupplyCurrentLimit = 70;
        flywheelCurrentLimit.SupplyCurrentLowerLimit = 40;
        flywheelCurrentLimit.SupplyCurrentLowerTime = 1.0;
        kTurretFlywheelConfiguration.withCurrentLimits(flywheelCurrentLimit);
        
        VoltageConfigs flywheelVoltageLimit = new VoltageConfigs();
        flywheelVoltageLimit.PeakForwardVoltage = 10.0;
        flywheelVoltageLimit.PeakReverseVoltage = -10.0;
        kTurretFlywheelConfiguration.withVoltage(flywheelVoltageLimit);
        kTurretFlywheelConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        if (!Constants.kTurretFlywheelMotorInverted) {
            kTurretFlywheelConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        } else {
            kTurretFlywheelConfiguration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        }
        kTurretFlywheelConfiguration.Slot0.withKP(Constants.kTurretFlywheelPIDF[0]);
        kTurretFlywheelConfiguration.Slot0.withKI(Constants.kTurretFlywheelPIDF[1]);
        kTurretFlywheelConfiguration.Slot0.withKD(Constants.kTurretFlywheelPIDF[2]);
        kTurretFlywheelConfiguration.Slot0.withKV(Constants.kTurretFlywheelPIDF[3]);
        m_turretFlywheelMotor.getConfigurator().apply(kTurretFlywheelConfiguration);

        

        //dashboardPIDcontrolInitFlywheel();
        //dashboardPIDcontrolInitTurretPitch();
        //dashboardPIDcontrolInitTurreAzimuth();
    }

    @Override
    public void periodic() {
        boolean isAllianceBlue;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            isAllianceBlue = false;
        } else {
            isAllianceBlue = true;
        }

        Translation2d hubPosition;
        if (!isAllianceBlue) {
            hubPosition = Constants.kHubPosRed;
        } else {
            hubPosition = Constants.kHubPosBlue;
        }
        Translation2d turretFieldPosition = new Translation2d(Constants.kTurretOffset.getY(), -Constants.kTurretOffset.getX()).rotateBy(m_swerveDrive.getHeading()).plus(m_swerveDrive.getRobotPosition());
        currentZone = findRobotZone(turretFieldPosition, Constants.kRobotZones, currentZone);
        Translation2d newTarget = findTarget(currentZone, turretFieldPosition, isAllianceBlue);
        if(newTarget!=null){
            currentTarget = newTarget;
        }
        m_swerveDrive.swerveDrive.field.getObject("Current Target").setPose(new Pose2d(currentTarget, new Rotation2d()));

        double desiredShooterHeading = CalculateRequiredShooterHeading(turretFieldPosition, currentTarget);
        lastDesiredShooterHeading = desiredShooterHeading;
        double targetOrientedSpeed = CalculateTargetOrientedShooterSpeed(desiredShooterHeading,
                m_swerveDrive.getRobotSpeedsField()/*.plus(m_swerveDrive.getRobotAccelerationField().times(Constants.kTurretOnTheMoveAccelerationFeedForwardSeconds))*/
        );
        double targetDistance = CalculateTargetDistance(turretFieldPosition, currentTarget);
        
        double effectiveDistance = CalculateEffectiveDistance(
                new Translation2d(lastTargetOrientedShooterSpeedX, lastTargetOrientedShooterSpeedY),
                targetDistance,
                Constants.kProjectileFixedTotalAirtime);
        double projectileSpeed = CalculateFixedPeakProjectileHeightToProjectileSpeed(effectiveDistance);
        if(lastRequiredShooterPitch >= Constants.kTurretPitchMinMaxDegrees[1]){
            projectileSpeed = CalculateFixedPitchProjectileSpeed(Constants.kTurretPitchMinMaxDegrees[1], effectiveDistance, Constants.kHubHeight, Constants.kTurretHeight);
        }
        else if(lastRequiredShooterPitch <= Constants.kTurretPitchMinMaxDegrees[0]){
            projectileSpeed = CalculateFixedPitchProjectileSpeed(Constants.kTurretPitchMinMaxDegrees[0], effectiveDistance, Constants.kHubHeight, Constants.kTurretHeight);
        }
        
        /*System.out.println("Calculated desiredShooterHeading: " + lastDesiredShooterHeading);
        System.out.println(
                "Calculated peakProjectileHeight: " + Constants.kDistanceToPeakProjectileHeightMapper(targetDistance));
        System.out.println("Calculated targetDistance: " + targetDistance);
        System.out.println("Calculated totalProjectileAirtime: " + totalProjectileAirtime);
        System.out.println("Calculated effectiveDistance: " + effectiveDistance);
        System.out.println("Calculated projectileSpeed: " + projectileSpeed);
        System.out.println(""Calculated desiredShooterPitch: " + lastRequiredShooterPitch);
        System.out.println(Calculated corrected shooter heading: " + (desiredShooterHeading + lastShooterHeadingError));
        System.out.println("Turret Heading " + m_turretAzimuthMotor.getEncoder().getPosition());
        System.out.println("//////////////");*/
        /*System.out.println("Turret pitch: "+m_turretPitchMotor.getEncoder().getPosition());
        System.out.println("Calculated desiredShooterPitch: " + lastRequiredShooterPitch);
        System.out.println("Required projectile speed: " + projectileSpeed);
        System.out.println("Required shooter RPM: " + CalculateExitVelocityToRPM(projectileSpeed));*/
        desiredFlywheelRPM = CalculateExitVelocityToRPM(projectileSpeed);
        SmartDashboard.putNumber("Turret Pitch", m_turretPitchMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Desired Turret Pitch", lastRequiredShooterPitch);
        SmartDashboard.putNumber("Required shooter RPM", desiredFlywheelRPM);
        SmartDashboard.putNumber("Shooter RPM", m_turretFlywheelMotor.getVelocity().getValueAsDouble()*60);
        SmartDashboard.putNumber("Robot Zone", currentZone);

        updateFlywheel();
        updateTurret();
        m_swerveDrive.swerveDrive.field.getObject("Turret").setPose(new Pose2d(turretFieldPosition, Rotation2d.fromDegrees(m_turretAzimuthMotor.getEncoder().getPosition() + m_swerveDrive.getHeading().getDegrees())));
        
        
        SmartDashboard.putBoolean("TargetAcquired", isTargetAcquired);

        //dashboardPIDcontrolLoopFlywheel();
        // dashboardPIDcontrolLoopTurretPitch();
        //dashboardPIDcontrolLoopTurretAzimuth();
    }

    /*public boolean checkLineOfSight(Translation2d obstacleBottomLeft, Translation2d obstacleTopRight, Translation2d target, Translation2d origin){
        double x1 = target.getX(); double x2 = origin.getX();
        double y1 = target.getY(); double y2 = origin.getY();

        double distanceX = x2 - x1;
        double distanceY = y2 - y1;

        double tMin = 0.0;
        double tMax = 1.0;

        
        if(Math.abs(distanceX) < 1e-6){
            if(x1 < obstacleBottomLeft.getX() || x1 > obstacleTopRight.getX()){
                return true;
            }
        }
        if(distanceX != 0){
            double tx1 = (obstacleBottomLeft.getX() - x1) / distanceX;
            double tx2 = (obstacleTopRight.getX() - x1) / distanceX;
        
            double txMin = Math.min(tx1, tx2);
            double txMax = Math.max(tx1, tx2);

            tMin = Math.max(tMin, txMin);
            tMax = Math.min(tMax, txMax);
        }
        
        if(Math.abs(distanceY) < 1e-6){
            if(y1 < obstacleBottomLeft.getY() || y1 > obstacleTopRight.getY()){
                return true;
            }
        }
        if(distanceY != 0){
            double ty1 = (obstacleBottomLeft.getY() - y1) / distanceY;
            double ty2 = (obstacleTopRight.getY() - y1) / distanceY;
        
            double tyMin = Math.min(ty1, ty2);
            double tyMax = Math.max(ty1, ty2);

            tMin = Math.max(tMin, tyMin);
            tMax = Math.min(tMax, tyMax);
        }

        if(tMax >= tMin){
            return false;
        }

        return true;
    }*/

    public Translation2d findTarget(int robotZone, Translation2d robotPosition, boolean allianceIsBlue){
        Translation2d returnTarget = null;

        if(allianceIsBlue && robotZone == 0){
            return Constants.kHubPosBlue;
        }
        else if(!allianceIsBlue && robotZone == 2){
            return Constants.kHubPosRed;
        }
        else if(allianceIsBlue && robotZone!=0){
            for (Translation2d target : Constants.kBluePassTargets) {
                m_swerveDrive.swerveDrive.field.getObject("Current Target").setPose(new Pose2d(target, new Rotation2d()));

                /*if(checkLineOfSight(
                        new Translation2d(Constants.kHubPosBlue.getX() - (Constants.kHubLength/2), Constants.kHubPosBlue.getY() - (Constants.kHubWidth/2)), 
                        new Translation2d(Constants.kHubPosBlue.getX() + (Constants.kHubLength/2), Constants.kHubPosBlue.getY() + (Constants.kHubWidth/2)), 
                        target, 
                        robotPosition
                    )
                ){*/
                    if(returnTarget == null){
                        returnTarget = target;
                    }
                    else if(robotPosition.getDistance(target) < robotPosition.getDistance(returnTarget)){
                        returnTarget = target;
                    }
                //}
            }
        }
        else if(!allianceIsBlue && robotZone!=2){
            for (Translation2d target : Constants.kRedPassTargets) {
                /*if(checkLineOfSight(
                        new Translation2d(Constants.kHubPosRed.getX() - (Constants.kHubLength/2), Constants.kHubPosRed.getY() - (Constants.kHubWidth/2)), 
                        new Translation2d(Constants.kHubPosRed.getX() + (Constants.kHubLength/2), Constants.kHubPosRed.getY() + (Constants.kHubWidth/2)), 
                        target, 
                        robotPosition
                    )
                ){*/
                    if(returnTarget == null){
                        returnTarget = target;
                    }
                    else if(robotPosition.getDistance(target) < robotPosition.getDistance(returnTarget)){
                        returnTarget = target;
                    }
                //}
            }
        }

        if(returnTarget ==  null){
            isTargetAcquired = false;
        }
        else{
            isTargetAcquired = true;
        }
        return returnTarget;
    }
    public int findRobotZone(Translation2d robotPosition, Translation2d[][] zones, int lastZone){
        for(int i = 0; i < zones.length; i++){
            if(
                zones[i][1].getX() > robotPosition.getX() && robotPosition.getX() > zones[i][0].getX() &&
                zones[i][1].getY() > robotPosition.getY() && robotPosition.getY() > zones[i][0].getY()
            ){
                return i;
            }
        }
        
        return lastZone;
    }

    public double CalculateFixedPeakProjectileHeightToProjectileSpeed(double effectiveDistance){
        double horizontalProjectileSpeed = effectiveDistance/Constants.kProjectileFixedTotalAirtime;
        double requiredShooterPitch = Math.atan(Constants.kProjectileFixedVerticalSpeed/horizontalProjectileSpeed)*180/Math.PI;
        lastRequiredShooterPitch = requiredShooterPitch;

        lastRequiredProjectileSpeedX = horizontalProjectileSpeed;
        lastRequiredProjectileSpeedZ = Constants.kProjectileFixedVerticalSpeed;
        return Math.hypot(horizontalProjectileSpeed, Constants.kProjectileFixedVerticalSpeed);
    }

    public double CalculateFixedPitchProjectileSpeed(double shooterPitch, double targetDistance, double targetHeight, double startingHeight){
        double heightDifference = targetHeight - startingHeight;

        double denom = 2 * Math.pow(Math.cos(Math.toRadians(shooterPitch)), 2) * (targetDistance * Math.tan(Math.toRadians(shooterPitch)) - heightDifference);

        if(denom <= 0){
            return 0;
        }

        return Math.sqrt(Constants.kGravitationalConstant * Math.pow(targetDistance, 2) / denom);
    }

    public double CalculateRequiredShooterHeading(Translation2d shooterPosition, Translation2d targetPosition) {
        double angle = (Math.atan2(targetPosition.getY() - shooterPosition.getY(),
                targetPosition.getX() - shooterPosition.getX()) * 180.0) / Math.PI;
        if (angle < 0) {
            return angle + 360;
        }
        return angle;
        // return (Math.atan2(targetPosition.getY() - shooterPosition.getY(),
        // targetPosition.getX() - shooterPosition.getX())*180.0)/Math.PI;
    }

    /**
     * Casts the field relative speed of the shooter to parallel and perpendicular
     * lines in the targets direction
     * 
     * @param targetHeading              heading of the target from the shooter in
     *                                   degrees, CalculateRequiredShooterHeading's
     *                                   return must be passed
     * @param fieldRelativeShooterSpeeds shooter's field relative speeds in the X
     *                                   and Y axis in meters per second
     * @return the total speed of the shooter(must be the same as the total of
     *         fieldRelativeShooterSpeeds) in meters per second
     */
    public double CalculateTargetOrientedShooterSpeed(double targetHeading, Translation2d fieldRelativeShooterSpeeds/*, Translation2d fieldRelativeAccelerationSpeeds*/) {
        double targetOrientedShooterSpeedX = (-(fieldRelativeShooterSpeeds.getX()
                * Math.sin((targetHeading * Math.PI) / 180.0))
                + (fieldRelativeShooterSpeeds.getY() * Math.cos((targetHeading * Math.PI) / 180.0)));
        double targetOrientedShooterSpeedY = -(fieldRelativeShooterSpeeds.getX()
                * Math.cos((targetHeading * Math.PI) / 180.0))
                - (fieldRelativeShooterSpeeds.getY() * Math.sin((targetHeading * Math.PI) / 180.0));

        lastTargetOrientedShooterSpeedX = targetOrientedShooterSpeedX;
        lastTargetOrientedShooterSpeedY = targetOrientedShooterSpeedY;

        SmartDashboard.putNumber("TargetOriented X", targetOrientedShooterSpeedX);
        SmartDashboard.putNumber("TargetOriented Y", targetOrientedShooterSpeedY);
        return Math.hypot(targetOrientedShooterSpeedX, targetOrientedShooterSpeedY);
    }

    /**
     * 
     * @param shooterPosition
     * @param targetPosition
     * @return
     */
    public double CalculateTargetDistance(Translation2d shooterPosition, Translation2d targetPosition) {
        return Math.hypot(shooterPosition.getY() - targetPosition.getY(),
                shooterPosition.getX() - targetPosition.getX());
    }

    /**
     * Calculates projectile airtime based on projectile's peak height and
     * sets lastTimeToReachPeak to timeToReachPeak and lastTimeToDescendFromPeak to
     * lastTimeToDescendFromPeak
     * 
     * @param projectileStartHeight the height at which the projectile leaves the
     *                              shooter in meters
     * @param projectilePeakHeight  highest point the projectile will reach in
     *                              meters
     * @param targetHeight          target height ground in meters
     * @return projectile's total air time in seconds
     */
    public double CalculateProjectileAirTime(double projectileStartHeight, double projectilePeakHeight,
            double targetHeight) {
        double timeToReachPeak = Math
                .sqrt((2 * (projectilePeakHeight - projectileStartHeight)) / Constants.kGravitationalConstant);
        lastTimeToReachPeak = timeToReachPeak;
        double timeToDescendFromPeak = Math
                .sqrt((2 * (projectilePeakHeight - targetHeight)) / Constants.kGravitationalConstant);
        lastTimeToDescendFromPeak = timeToDescendFromPeak;

        return timeToReachPeak + timeToDescendFromPeak;
    }

    /*
     * public double AdjustRequiredProjectileSpeeds(Translation2d
     * targetOrientedShooterSpeeds, double targetDistance, double
     * totalCalculatedAirtime){
     * double requiredTargetOrientedProjectileSpeedX = lastRequiredProjectileSpeedX
     * - targetOrientedShooterSpeeds.getX();
     * double requiredTargetOrientedProjectileSpeedZ = lastRequiredProjectileSpeedZ
     * - targetOrientedShooterSpeeds.getY();
     * lastRequiredTargetOrientedProjectileSpeedX =
     * requiredTargetOrientedProjectileSpeedX;
     * lastRequiredTargetOrientedProjectileSpeedZ =
     * requiredTargetOrientedProjectileSpeedZ;
     * double shooterHeadingError =
     * (Math.atan2(requiredTargetOrientedProjectileSpeedZ,
     * requiredTargetOrientedProjectileSpeedX)*180.0)/Math.PI;
     * lastShooterHeadingError = shooterHeadingError;
     * 
     * 
     * return Math.hypot(requiredTargetOrientedProjectileSpeedX,
     * requiredTargetOrientedProjectileSpeedZ);
     * }
     */
    /**
     * TODO: Write documentation
     * 
     * @param targetOrientedShooterSpeeds
     * @param targetDistance
     * @param totalCalculatedAirtime
     * @return
     */
    public double CalculateEffectiveDistance(Translation2d targetOrientedShooterSpeeds, double targetDistance,
            double totalCalculatedAirtime) {
        double targetErrorDistanceX = targetOrientedShooterSpeeds.getX() * totalCalculatedAirtime;
        double targetErrorDistanceY = targetOrientedShooterSpeeds.getY() * totalCalculatedAirtime;

        double effectiveDistance = Math.hypot(targetDistance + targetErrorDistanceY, targetErrorDistanceX);
        double shooterHeadingError = (Math.atan2(targetErrorDistanceX, targetDistance - targetErrorDistanceY) * 180.0)
                / Math.PI;
        lastShooterHeadingError = shooterHeadingError;

        return effectiveDistance;
    }

    /**
     * Calculates the required projectile speed in order to reach the target point
     * and
     * sets lastRequiredProjectileSpeedX to requiredProjectileSpeedX and
     * lastRequiredProjectileSpeedY to requiredProjectileSpeedY and
     * sets lastRequiredShooterPitch to requiredShooterPitch
     * 
     * @param timeToReachPeak
     * @param timeToDescendFromPeak
     * @param targetDistance
     * @return the total speed required for the projectile to reach the target
     */
    public double CalculateRequiredProjectileSpeed(double timeToReachPeak, double timeToDescendFromPeak,
            double targetDistance) {
        double requiredProjectileSpeedX = targetDistance / (timeToReachPeak + timeToDescendFromPeak);
        lastRequiredProjectileSpeedX = requiredProjectileSpeedX;
        double requiredProjectileSpeedZ = timeToReachPeak * Constants.kGravitationalConstant;
        lastRequiredProjectileSpeedZ = requiredProjectileSpeedZ;

        double requiredShooterPitch = (Math.atan2(lastRequiredProjectileSpeedZ, lastRequiredProjectileSpeedX) * 180.0)
                / Math.PI;
        lastRequiredShooterPitch = requiredShooterPitch;

        return Math.hypot(requiredProjectileSpeedX, requiredProjectileSpeedZ);
    }

    public double CalculateExitVelocityToRPM(double exitVelocity){
        double RPM = 60 * exitVelocity/(Math.PI*2*lastFlywheelMultiplier);
        return RPM;
    }

    public void toggleFlywheel() {
        flywheelOn = !flywheelOn;
    }
    public void setFlywheelRPM(double RPM) {
        desiredFlywheelRPM = MathUtil.clamp(RPM, -Constants.kFlywheelMaxSpeedRPM, Constants.kFlywheelMaxSpeedRPM);
    }


    public void updateFlywheel() {
        if(isUnjamming){
            return;
        }

        if(flywheelOverride){
            flywheelReady = true;
            VelocityVoltage flywheelVelocity = new VelocityVoltage(0);
            flywheelVelocity.Velocity = Constants.kFlywheelOverrideRPM / 60.0;
            m_turretFlywheelMotor.setControl(flywheelVelocity);
            return;
        }

        if (desiredFlywheelRPM <= 0.001 
            || Math.abs(lastDesiredShooterHeading - lastShooterHeadingError - (m_swerveDrive.getHeading().getDegrees() % 360) - (m_turretAzimuthMotor.getEncoder().getPosition()%360)) > Constants.kTurretAzimuthMotorToleranceDegrees
            ) {
            flywheelReady = false;
        } else {
            flywheelReady = true;
        }

        if (desiredFlywheelRPM == 0.0 || !flywheelOn) {
            m_turretFlywheelMotor.set(0);
        } else {
            VelocityVoltage flywheelVelocity = new VelocityVoltage(0);
            flywheelVelocity.Velocity = desiredFlywheelRPM / 60.0;
            m_turretFlywheelMotor.setControl(flywheelVelocity);
        }
    }

    public void turnTurretTo(double angle) {
        m_turretAzimuthMotor.getClosedLoopController().setSetpoint(MathUtil.clamp(angle%360, 70, 360), ControlType.kPosition);
    }
    public void toggleTurretActive() {
        isTurretActive = !isTurretActive;
    }
    public void updateTurret() {
        if(flywheelOverride){
            turnTurretTo(180);
            setTurretPitch(Constants.kTurretPitchOverride);

            return;
        }
        if (isTurretActive) {
            turnTurretTo(lastDesiredShooterHeading - lastShooterHeadingError - m_swerveDrive.getHeading().getDegrees());
            setTurretPitch(lastRequiredShooterPitch);
        } else {
            turnTurretTo(180);
            setTurretPitch(Constants.kTurretPitchMinMaxDegrees[1]);
        }
    }

    public void setTurretPitch(double degrees){
        m_turretPitchMotor.getClosedLoopController().setSetpoint(MathUtil.clamp(degrees, Constants.kTurretPitchMinMaxDegrees[0], Constants.kTurretPitchMinMaxDegrees[1]), ControlType.kPosition);
    }

    public void dashboardPIDcontrolInitFlywheel() {
        lastP = Constants.kTurretFlywheelPIDF[0];
        lastD = Constants.kTurretFlywheelPIDF[2];
        lastF = Constants.kTurretFlywheelPIDF[3];
        
        SmartDashboard.putNumber("Shooter/FlywheelP", lastP);
        SmartDashboard.putNumber("Shooter/FlywheelD", lastD);
        SmartDashboard.putNumber("Shooter/FlywheelF", lastF);
        SmartDashboard.putNumber("Shooter/FlywheelRPM", 0);
        SmartDashboard.putNumber("Shooter/FlywheelMultiplier", lastFlywheelMultiplier);
        SmartDashboard.putBoolean("Shooter/RunFlywheel", false);
    }

    public void dashboardPIDcontrolLoopFlywheel() {
        if (!DriverStation.isFMSAttached()) {
            final double p = SmartDashboard.getNumber("Shooter/FlywheelP", lastP);
            final double d = SmartDashboard.getNumber("Shooter/FlywheelD", lastD);
            final double f = SmartDashboard.getNumber("Shooter/FlywheelF", lastF);
            final double flywheelMultiplier = SmartDashboard.getNumber("Shooter/FlywheelMultiplier", lastFlywheelMultiplier);
            final double RPM = SmartDashboard.getNumber("Shooter/FlywheelRPM", 0);
            final boolean run = SmartDashboard.getBoolean("Shooter/RunFlywheel", false);

            SmartDashboard.putNumber("FlywheelRPM", m_turretFlywheelMotor.getVelocity().getValueAsDouble() * 60
                    / Constants.kTurretFlywheelEncoderVelocityFactor);
            SmartDashboard.putNumber("FlywheelCurrent", m_turretFlywheelMotor.getStatorCurrent().getValueAsDouble());

            if (p != lastP || d != lastD || f != lastF || RPM != lastRPM || run != lastRun || flywheelMultiplier != lastFlywheelMultiplier) {
                lastP = p;
                lastD = d;
                lastF = f;
                lastRPM = RPM;
                lastRun = run;
                lastFlywheelMultiplier = flywheelMultiplier;
                kTurretFlywheelConfiguration.Slot0.kP = p;
                kTurretFlywheelConfiguration.Slot0.kD = d;
                kTurretFlywheelConfiguration.Slot0.kV = f;
                m_turretFlywheelMotor.getConfigurator().apply(kTurretFlywheelConfiguration);
                if (run) {
                    VelocityVoltage velocity = new VelocityVoltage(0);
                    velocity.Velocity = RPM / 60;
                    m_turretFlywheelMotor.setControl(velocity);
                } else {
                    m_turretFlywheelMotor.set(0);
                }
            }
        }
    }
    public void dashboardPIDcontrolInitTurreAzimuth() {
        lastP = Constants.kTurretAzimuthMotorPID[0];
        lastD = Constants.kTurretAzimuthMotorPID[2];

        SmartDashboard.putNumber("Shooter/AzimuthP", lastP);
        SmartDashboard.putNumber("Shooter/AzimuthD", lastD);
        SmartDashboard.putNumber("Shooter/AzimuthAngle", 0);
        SmartDashboard.putBoolean("Shooter/RunAzimuth", false);
    }

    public void dashboardPIDcontrolLoopTurretAzimuth() {
        if (!DriverStation.isFMSAttached()) {
            final double p = SmartDashboard.getNumber("Shooter/AzimuthP", lastP);
            final double d = SmartDashboard.getNumber("Shooter/AzimuthD", lastD);
            final double angle = SmartDashboard.getNumber("Shooter/AzimuthAngle", 0);
            final boolean run = SmartDashboard.getBoolean("Shooter/RunAzimuth", false);

            SmartDashboard.putNumber("AzimuthAngle", m_turretAzimuthMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("AzimuthCurrent", m_turretAzimuthMotor.getOutputCurrent());

            if (p != lastP || d != lastD || angle != lastPosition || run != lastRun) {
                lastP = p;
                lastD = d;
                lastPosition = angle;
                lastRun = run;
                kTurretAzimuthMotorConfig.closedLoop.pid(p, 0, d);
                m_turretAzimuthMotor.configure(kTurretAzimuthMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                if (run) {
                    m_turretAzimuthMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
                } else {
                    m_turretAzimuthMotor.set(0);
                }
            }
        }
    }
    public void dashboardPIDcontrolInitTurretPitch() {
        lastP = Constants.kTurretPitchPID[0];
        lastD = Constants.kTurretPitchPID[2];

        SmartDashboard.putNumber("Shooter/PitchP", lastP);
        SmartDashboard.putNumber("Shooter/PitchD", lastD);
        SmartDashboard.putNumber("Shooter/PitchAngle", 0);
        SmartDashboard.putBoolean("Shooter/RunPitch", false);
    }

    public void dashboardPIDcontrolLoopTurretPitch() {
        if (!DriverStation.isFMSAttached()) {
            final double p = SmartDashboard.getNumber("Shooter/PitchP", lastP);
            final double d = SmartDashboard.getNumber("Shooter/PitchD", lastD);
            final double angle = SmartDashboard.getNumber("Shooter/PitchAngle", 0);
            final boolean run = SmartDashboard.getBoolean("Shooter/RunPitch", false);

            SmartDashboard.putNumber("PitchAngle", m_turretPitchMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("PitchCurrent", m_turretPitchMotor.getOutputCurrent());

            if (p != lastP || d != lastD || angle != lastPosition || run != lastRun) {
                lastP = p;
                lastD = d;
                lastPosition = angle;
                lastRun = run;
                kTurretPitchMotorConfig.closedLoop.pid(p, 0, d);
                m_turretPitchMotor.configure(kTurretPitchMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                if (run) {
                    m_turretPitchMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
                } else {
                    m_turretPitchMotor.set(0);
                }
            }
        }
    }
}