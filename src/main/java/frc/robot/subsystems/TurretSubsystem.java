package frc.robot.subsystems;


import java.io.Serial;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurretSubsystem extends SubsystemBase {
    private SwerveSubsystem m_swerveDrive;
    //public double currentDistanceMultiplier;
    //public PIDController turretCorrectionPIDloop = new PIDController(Constants.kTurretCorrectionPID[0], Constants.kTurretCorrectionPID[1], Constants.kTurretCorrectionPID[2]);

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
    }

    @Override
    public void periodic(){
        Translation2d hubPosition;
        if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) hubPosition = Constants.kHubPosRed;
        else hubPosition = Constants.kHubPosBlue;

        double desiredShooterHeading = CalculateRequiredShooterHeading(m_swerveDrive.getRobotPosition(), hubPosition);
        double targetOrientedSpeed = CalculateTargetOrientedShooterSpeed(desiredShooterHeading, m_swerveDrive.getRobotSpeedsField());
        double targetDistance = CalculateTargetDistance(m_swerveDrive.getRobotPosition(), hubPosition);
        double totalProjectileAirtime = CalculateProjectileAirTime(0.5, Constants.kDistanceToPeakProjectileHeightMapper(targetDistance), Constants.kHubHeight);
        double effectiveDistance = CalculateEffectiveDistance(
            new Translation2d(lastTargetOrientedShooterSpeedX, lastTargetOrientedShooterSpeedY),
            targetDistance, 
            totalProjectileAirtime
        );
        double projectileSpeed = CalculateRequiredProjectileSpeed(lastTimeToReachPeak, lastTimeToDescendFromPeak, effectiveDistance);
        for(int i = 0; i < 5; i++){
            totalProjectileAirtime = CalculateProjectileAirTime(0.5, Constants.kDistanceToPeakProjectileHeightMapper(effectiveDistance), Constants.kHubHeight);
            effectiveDistance = CalculateEffectiveDistance(
                new Translation2d(lastTargetOrientedShooterSpeedX, lastTargetOrientedShooterSpeedY),
                targetDistance, 
                totalProjectileAirtime
            );
            projectileSpeed = CalculateRequiredProjectileSpeed(lastTimeToReachPeak, lastTimeToDescendFromPeak, effectiveDistance);    
        }
        System.out.println("Calculated desiredShooterHeading: " + desiredShooterHeading);
        System.out.println("Calculated peakProjectileHeight: " + Constants.kDistanceToPeakProjectileHeightMapper(targetDistance));
        System.out.println("Calculated targetDistance: " + targetDistance);
        System.out.println("Calculated totalProjectileAirtime: " + totalProjectileAirtime);
        System.out.println("Calculated effectiveDistance: " + effectiveDistance);
        System.out.println("Calculated projectileSpeed: " + projectileSpeed);
        System.out.println("Calculated desiredShooterPitch: " + lastRequiredShooterPitch);
        System.out.println("Calculated corrected shooter heading: " + (desiredShooterHeading+lastShooterHeadingError));
    }

    /*public void LogCalculations(){
        
    }*/
    
    public double CalculateRequiredShooterHeading(Translation2d shooterPosition, Translation2d targetPosition){
        return (Math.atan2(targetPosition.getY() - shooterPosition.getY(), targetPosition.getX() - shooterPosition.getX())*180.0)/Math.PI;
    }
    /**
     * Casts the field relative speed of the shooter to parallel and perpendicular lines in the targets direction
     * @param targetHeading heading of the target from the shooter in degrees, CalculateRequiredShooterHeading's return must be passed
     * @param fieldRelativeShooterSpeeds shooter's field relative speeds in the X and Y axis in meters per second
     * @return the total speed of the shooter(must be the same as the total of fieldRelativeShooterSpeeds) in meters per second
     */
    public double CalculateTargetOrientedShooterSpeed(double targetHeading, Translation2d fieldRelativeShooterSpeeds){
        double targetOrientedShooterSpeedX = (fieldRelativeShooterSpeeds.getX() * Math.cos((targetHeading*Math.PI)/180.0)) - (fieldRelativeShooterSpeeds.getY() * Math.sin((targetHeading*Math.PI)/180.0));
        double targetOrientedShooterSpeedY = (fieldRelativeShooterSpeeds.getX() * Math.sin((targetHeading*Math.PI)/180.0)) + (fieldRelativeShooterSpeeds.getY() * Math.cos((targetHeading*Math.PI)/180.0));
        
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
    public double CalculateEffectiveDistance(Translation2d targetOrientedShooterSpeeds, double targetDistance, double totalCalculatedAirtime){
        double targetErrorDistanceX = targetOrientedShooterSpeeds.getX() * totalCalculatedAirtime;
        double targetErrorDistanceY = targetOrientedShooterSpeeds.getY() * totalCalculatedAirtime;
        
        double effectiveDistance = Math.hypot(targetDistance - targetErrorDistanceY, targetErrorDistanceX);
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
        
        double requiredShooterPitch = (Math.atan(lastRequiredProjectileSpeedZ/lastRequiredProjectileSpeedX)*180.0)/Math.PI;
        lastRequiredShooterPitch = requiredShooterPitch;
        
        return Math.hypot(requiredProjectileSpeedX, requiredProjectileSpeedZ);
    }
    
}
