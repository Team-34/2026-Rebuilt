package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.lang.Math;

public class LimelightCalculations extends SubsystemBase{
    
    double ta = LimelightHelpers.getTA("");


    /**
     * Calculates the base-2 logarithm of a number. I ripped this from G4G
     * so I desperately hope it works.
     * @param n The number to calculate the logarithm of.
     * @return The base-2 logarithm of the number indirectly by using log2 N = log N / log 2
     */
    public static double log2(double n) {
        double result = (Math.log(n) / Math.log(2));
        return result;
    }

    /**
     * I will not lie, I am not sure how to code this yet. 
     * 
     */
    public static double returnSetpointLimelight() {
        double txrad = Math.toRadians(LimelightHelpers.getTX(""));
        return txrad;
    }

    /**
     * Definitely copies directly from the 2024 code.
     * work smarter, not harder
     * @return Distance to the target, in an unknown value (for now). needs to be tested.
     */
    public double getDistanceToTarget() {
        return log2(2.5 / ta) - 0.3;
    }

    public void periodic() {

    }

}
