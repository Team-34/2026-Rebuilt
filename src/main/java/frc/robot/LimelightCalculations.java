package frc.robot;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.lang.Math;
import java.lang.reflect.Array;

public class LimelightCalculations {

    Pigeon2 gyro = new Pigeon2(10);
        
    /**
     * I will not lie, I am not sure how to code this yet. 
     * The more I get experience with the subsystem, the better I can make 
     * this code.
     */
    public static double returnSetpointLimelight() {
        double txrad = Math.toRadians(LimelightHelpers.getTX(""));
        
        return txrad;
    }

    /**
     * Used for getting distance to an apriltag.
     * Will be used in conjunction with the turret or shooter subsystem. 
     * Utilizes a TZ variable (hidden from the world) that returns the variable 
     * in an unknown conversion factor, it can be multiplied by 65 to get the 
     * distance in inches.
     * @return The distance to a tag, in inches.
     */
    public static double getDistanceToTarget() {

        double[] targpose_camspace = LimelightHelpers.getTargetPose_CameraSpace("");
        
        return targpose_camspace[2] * 65;
    }

    

}
