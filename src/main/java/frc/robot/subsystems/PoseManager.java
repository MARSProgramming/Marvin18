package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MoreMath;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;

public class PoseManager extends SubsystemBase {
    
    private List<Pose2d> reefPoses;
    private List<Pose2d> feederPoses;
    private List<Pose2d> empty = new ArrayList<Pose2d>();
    private Alliance alliance;

    public PoseManager(Alliance dsAlliance) {
        if (dsAlliance != null) {
            alliance = dsAlliance;
        }
        reefPoses = generateList("reef");
        feederPoses = generateList("feeder");
    }

    public List<Pose2d> generateList(String listType) {
        if (listType.equals("reef")) {
            return (
                ((alliance == Alliance.Blue) ? MoreMath.getPoseList(Constants.VisionFiducials.BLUE_CORAL_TAGS) : MoreMath.getPoseList(Constants.VisionFiducials.RED_CORAL_TAGS))
            );
        }
        if (listType.equals("feeder")) {
            return (
                ((alliance == Alliance.Blue) ? MoreMath.getPoseList(Constants.VisionFiducials.BLUE_FEEDER_TAGS) : MoreMath.getPoseList(Constants.VisionFiducials.RED_FEEDER_TAGS))
            );
        }

        else {
            return empty;
        }
    }

    public Pose2d returnOptimalPose(Pose2d currentPose, String desired, int level, boolean leftSide) {
        Transform2d transform = new Transform2d();
        if (desired.equals("reef")) {
            Pose2d nearestReef = MoreMath.getNearestWithoutCreation(currentPose, reefPoses);
            if (level == 4) {
                transform = 
                (leftSide ? DynamicConstants.Transforms.LEFT_L4_TRANSFORM : DynamicConstants.Transforms.RIGHT_L4_TRANSFORM);
            } else if (level == 3) {
                transform =
                (leftSide ? DynamicConstants.Transforms.LEFT_L3_TRANSFORM : DynamicConstants.Transforms.RIGHT_L3_TRANSFORM);
            } else if (level == 2) {
                transform = 
                (leftSide ? DynamicConstants.Transforms.LEFT_L2_TRANSFORM : DynamicConstants.Transforms.RIGHT_L2_TRANSFORM);
            } else if (level == 1) {
                transform = 
                (leftSide ? DynamicConstants.Transforms.LEFT_L1_TRANSFORM : DynamicConstants.Transforms.RIGHT_L1_TRANSFORM);
            }
            return nearestReef.transformBy(transform);
        }

        if (desired.equals("algae")) {
            Pose2d nearestReef = MoreMath.getNearestWithoutCreation(currentPose, reefPoses);
            return nearestReef.transformBy(DynamicConstants.Transforms.ALGAE_TRANSFORM);
        }

        if (desired.equals("feeder")) {
            Pose2d nearestFeeder = MoreMath.getNearestWithoutCreation(currentPose, feederPoses);
            return nearestFeeder.transformBy(DynamicConstants.Transforms.FEEDER_TRANSFORM);
        }
         else {
            return currentPose;
         }
    }

    // fallback that we can put on copilot in case initializiation fails.
    public void setAlliance(Alliance all) {
        alliance = all;
    }

    @Override
    public void periodic() {
        if (alliance == Alliance.Blue) {
            SmartDashboard.putString("Pose Manager Alliance", "Blue");
        }
        if (alliance == Alliance.Red) {
            SmartDashboard.putString("Pose Manager Alliance", "Red");
        } else {
            SmartDashboard.putString("Pose Manager Alliance", "NOT FOUND!");
        }
    }


}
