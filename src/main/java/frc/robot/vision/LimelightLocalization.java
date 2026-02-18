package frc.robot.vision;

import java.io.InputStreamReader;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightLocalization extends SubsystemBase {
    
    String limelightName;
    int[] validIDs = {
        1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,
        17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32
    };

    double last_yaw = 0; double last_yaw_velocity = 0;

    public LimelightLocalization(String name){
        limelightName = name;
        LimelightHelpers.SetIMUMode(name, 1);
        initializeValidIDs();
    }

    public void initializeValidIDs(){
        boolean isRed;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRed = alliance.get() == DriverStation.Alliance.Red;
        }
        else { 
            isRed = false;

        }
        

        //Alliance based id filters can be implemented. Currently all the id numbers are used.
        if(isRed){
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
        }
        else{
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
        }
    }
    public void setRobotOrientation(double yaw, double yawRate){
        LimelightHelpers.SetRobotOrientation(limelightName, 
            yaw, yawRate, 
            0, 0, 
            0, 0
        );
    }
    
    /**
     *
     * @param mt1_threshold_tagCount the minimum amount of april tags needed in view at which mt1 results can be returned
     * @param mt1_threshold_distance the maximum distance at which mt1 results can be returned
     * @param mt1_threshold_angular_velocty the maximum angular velocity at which mt1 results can be returned
     * @WARNING all of the given conditions should be met for MegaTag1 values to be returned
     * @return LimelightHelpers.PoseEstimate type. Use PoseEstimate.pose and PoseEstimate.TimeStampSeconds to feed into odometry
     */
    public LimelightHelpers.PoseEstimate getBotPoseEstimate(
        int mt1_threshold_tagCount, 
        double mt1_threshold_distance, 
        double mt1_threshold_angular_velocty
    ){
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if(mt1 == null){
            return null;
        }
        if(mt1.tagCount <= 0){
            return null;
        }
        if(
            mt1.tagCount >= mt1_threshold_tagCount && 
            mt1.avgTagDist <= mt1_threshold_distance && 
            last_yaw_velocity <= mt1_threshold_angular_velocty
        ){
            return mt1;
        }
        else{
            return mt2;
        }
    }

}
