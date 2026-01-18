package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurretSubsystem extends SubsystemBase {
    public double currentDistanceMultiplier;


    public TurretSubsystem(SwerveSubsystem m_swerveDrive){

    }

    @Override
    public void periodic(){

    }

    /**
     * TODO: Write better comments
     * Function to find the closest distance from the currentPose to the targetPose in the targetList
     * @param currentPose
     * @param targetPose
     * @param targetList
     * @param distanceMultiplier default should be 1
     * @return returns the index of the closest point inside targetList
     */
    public int FindClosestCircle(Translation2d currentPos, Translation2d targetPos, double[] targetList, double distanceMultiplier){
        double distance = currentPos.getDistance(targetPos);
        int closestIndex = -1;
        double closestDistance = Double.POSITIVE_INFINITY;
        
        int i = 0;
        for (double d : targetList) {
            if(Math.abs(distance-(d*distanceMultiplier)) < closestDistance){
                closestDistance = (distance-(d*distanceMultiplier));
                closestIndex = i;
            }
            i++;
        }
        return closestIndex;
    }

    public Translation2d ForcedDistanceSpeed(Pose2d robotPose, Translation2d targetPosition, double targetDistance, Translation2d joystickInput){
        Translation2d shooterPosition = new Translation2d(
            Constants.kShooterOffsetDistance*Math.cos(Constants.kShooterOffsetAngle-robotPose.getRotation().getRadians()) + robotPose.getX(),
            Constants.kShooterOffsetDistance*Math.sin(Constants.kShooterOffsetAngle-robotPose.getRotation().getRadians()) + robotPose.getY()
        );
        Rotation2d angleOnCircle = Rotation2d.fromRadians(Math.atan2(
            targetPosition.getY()-shooterPosition.getY(), 
            targetPosition.getX()-shooterPosition.getX()
        ));
        /*Translation2d circleInterceptionPoint = new Translation2d(
            targetDistance*Math.cos(angleOnCircle.getRadians()) + targetPosition.getX(),
            targetDistance*Math.sin(angleOnCircle.getRadians()) + targetPosition.getY()
        );*/

        double distanceToCircle = targetDistance - Math.hypot(
            targetPosition.getX()-shooterPosition.getX(), 
            targetPosition.getY()-shooterPosition.getY()
        );

        /**
         * TODO: Implement PID loop, currently given kRobotMaxSpeed as a placeholder
         */
        Translation2d speedVectorToCircle = new Translation2d(
            MathUtil.clamp(distanceToCircle*Math.cos(angleOnCircle.getRadians())*Constants.kRobotMaxSpeed, -Constants.kRobotMaxSpeed, Constants.kRobotMaxSpeed),
            MathUtil.clamp(distanceToCircle*Math.sin(angleOnCircle.getRadians())*Constants.kRobotMaxSpeed, -Constants.kRobotMaxSpeed, Constants.kRobotMaxSpeed)
        );

        Translation2d inputSpeedVector = new Translation2d(
            joystickInput.getX()*-1*Math.sin(angleOnCircle.getRadians())*Constants.kRobotMaxSpeed,
            joystickInput.getY()*Math.cos(angleOnCircle.getRadians())*Constants.kRobotMaxSpeed
        );
        
        Translation2d totalSpeedVector = speedVectorToCircle.plus(inputSpeedVector);

        return totalSpeedVector;
    }


    public double ShooterAzimuthError(Translation2d perpendicularSpeed, Rotation2d currentHeading, double targetDistance, double timeOfArrival, int headingZone){
        double targetErrorDistance = Math.hypot(perpendicularSpeed.getX(), perpendicularSpeed.getY())*timeOfArrival;
        double totalDisplacement = Math.hypot(targetDistance, targetErrorDistance);
        Rotation2d errorAlpha = Rotation2d.fromRadians(Math.asin(targetErrorDistance/totalDisplacement));
        Rotation2d correctedHeading = currentHeading; // Initialize uncorrected

        switch (headingZone) {
            case 1:
                correctedHeading.minus(errorAlpha);
                break;
            case 2:
                correctedHeading.plus(errorAlpha);
                break;
            case 3:
                correctedHeading.minus(errorAlpha);
                break;
            case 4:
                correctedHeading.plus(errorAlpha);
                break;
            default:
                System.err.println("Invalid heading zone, must be from 1 to 4");
                return 0;
        }

        currentDistanceMultiplier = errorAlpha.getCos();
        return correctedHeading.minus(currentHeading).getDegrees();
    }
}
