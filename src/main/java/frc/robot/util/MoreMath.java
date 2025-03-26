package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class MoreMath {


    
    private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static double ensureRange(double value, double minValue, double maxValue) {
        if (minValue > value) {
            return minValue;
        } else if (maxValue < value) {
            return maxValue;
        } else {
            return value;
        }
    }

    public static Pose2d getNearest(Pose2d pose, int[] tagIntegers) {
        return pose.nearest(getPoseList(tagIntegers));
    }

    public static List<Pose2d> getPoseList(int[] tagIntegers) {
    List<Pose2d> tags = new ArrayList<>();
    for (int i = 0; i < tagIntegers.length; i++) {
      int tagInteger = tagIntegers[i];
      Pose2d tagPose = fieldLayout.getTagPose(tagInteger).get().toPose2d();
      tags.add(tagPose);
    }
    return tags;
  }

	/** SuperNURDS Interpolation Utility - taken from SuperCORE v2025.1.0 (FRC team 3255 base robot project)
	 *
	 * @param input
	 *            - the input value used to determine the output value
	 * @param minInput
	 *            - the input value that should map to the outputAtMin value
	 * @param maxInput
	 *            - the input value that should map to the outputAtMax value
	 * @param outputAtMin
	 *            - the output value when input = minInput
	 * @param outputAtMax
	 *            - the output value when input = maxInput
	 * @return - interpolated value
	 */

     public static double interpolate(double input, double minInput, double maxInput, double outputAtMin,
     double outputAtMax) {
        double output = input;
        if (input <= minInput) {
        output = outputAtMin;
        } else if (input >= maxInput) {
        output = outputAtMax;
        } else {
        // output is somewhere between minOutput and maxOutput
         output = outputAtMin + (((input - minInput) / (maxInput - minInput)) * (outputAtMax - outputAtMin));
        }
        return output;
    }

}