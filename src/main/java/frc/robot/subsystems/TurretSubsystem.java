package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurretSubsystem extends SubsystemBase {
    private SwerveSubsystem m_swerveDrive;
    public double currentDistanceMultiplier;
    public PIDController circleApproachPIDloop = new PIDController(Constants.kCircleApproachPID[0], Constants.kCircleApproachPID[1], Constants.kCircleApproachPID[2]);
    public PIDController turretCorrectionPIDloop = new PIDController(Constants.kTurretCorrectionPID[0], Constants.kTurretCorrectionPID[1], Constants.kTurretCorrectionPID[2]);

    public TurretSubsystem(SwerveSubsystem m_swerveDrive){
        this.m_swerveDrive = m_swerveDrive;
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

    /**
     * Calculates the speed vector required to approach targetDistance plus joystickInput 
     * requested speed perpendicular to the targetDistance. This allows the robot to stay
     * at a targetDistance meters away from targetPosition no matter the input
     * @param robotPose the current robot pose
     * @param targetPosition target position we want the robot to stay targetDistance meters away from
     * @param targetDistance requested distance in meters to the target
     * @param joystickInput joystick axes used for robot drive
     * @return returns the speed vector
     */
    public Translation2d ForcedDistanceSpeed(
        Pose2d robotPose, 
        Translation2d targetPosition, 
        double targetDistance, 
        Translation2d joystickInput
    ){
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

        ///
        /// Operations for finding the approach speed to the closest circle with PID
        /// 
        Translation2d vectorToCircle = new Translation2d(
            Math.cos(angleOnCircle.getRadians()),
            Math.sin(angleOnCircle.getRadians())
        );
        Translation2d speedVectorToCircle = vectorToCircle.times(MathUtil.clamp(
            circleApproachPIDloop.calculate(distanceToCircle, 0)*Constants.kRobotMaxSpeed, 
            -Constants.kRobotMaxSpeed, 
            Constants.kRobotMaxSpeed
        ));
        ///

        ///
        /// Operations for converting joystick input to speed tangent to the circle
        ///
        Translation2d tangentVector = new Translation2d(
            -Math.sin(angleOnCircle.getRadians()),
            Math.cos(angleOnCircle.getRadians())
        );
        double joystickProjectedMagnitude = tangentVector.dot(joystickInput);
        Translation2d tangentInputSpeed = tangentVector.times(joystickProjectedMagnitude*Constants.kRobotMaxSpeed);
        ///

        Translation2d totalSpeedVector = speedVectorToCircle.plus(tangentInputSpeed);

        return totalSpeedVector;
    }


    /*public double AngleToTarget(Translation2d shooterPosition, Translation2d targetPosition){
        double beta = Math.atan2();
    }*/

    /**
     * 
     * @param perpendicularSpeed
     * @param currentHeading
     * @param targetDistance
     * @param timeOfArrival
     * @param headingZone
     * @return calculated degrees of lead
     */
    public double ShooterAzimuthLead(Translation2d perpendicularSpeed, Rotation2d currentHeading, double targetDistance, double timeOfArrival, int headingZone){
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
        
        // Debug
        System.out.println("targetErrorDistance: " + targetErrorDistance);
        System.out.println("totalDisplacement: " + totalDisplacement);
        System.out.println("errorAlpha: " + errorAlpha.getDegrees());
        System.out.println("correctedHeading: " + correctedHeading.getDegrees());
        System.out.println("currentHeading: " + currentHeading.getDegrees());
        System.out.println("currentDistanceMultiplier: " + currentDistanceMultiplier);

        //
        return correctedHeading.minus(currentHeading).getDegrees();
    }

    public Command LockRobotToTarget(Translation2d perpendicularSpeed, Rotation2d currentHeading, double targetDistance, double timeOfArrival, int headingZone){
        return new RunCommand(
            () -> {
                System.out.print("Locked on target, ");
                double azimuthError = ShooterAzimuthLead(perpendicularSpeed, currentHeading, targetDistance, timeOfArrival, headingZone); 
                System.out.println("angle error: "+azimuthError);
                m_swerveDrive.drive(m_swerveDrive.processVelocityToChassisSpeeds(
                    -MathUtil.applyDeadband(RobotContainer.driveJoystick.getRawAxis(1)*Constants.kRobotMaxSpeed, 0.02), 
                    -MathUtil.applyDeadband(RobotContainer.driveJoystick.getRawAxis(0)*Constants.kRobotMaxSpeed, 0.02), 
                    MathUtil.clamp(turretCorrectionPIDloop.calculate(azimuthError, 0)*Constants.kRobotMaxAngularSpeed, -Constants.kRobotMaxAngularSpeed, Constants.kRobotMaxAngularSpeed), 
                    currentHeading, true));
            },
            m_swerveDrive
        );
    }
}
