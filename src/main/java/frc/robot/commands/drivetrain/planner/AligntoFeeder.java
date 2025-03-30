// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.planner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.Second;

import java.util.ArrayList;
import java.util.List;

import frc.robot.constants.DynamicConstants;
import frc.robot.constants.Constants.VisionFiducials;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AligntoFeeder extends Command {
  /** Creates a new AligntoFeeder. */
  private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private List<Pose2d> leftFeeders = getPoseList(
      List.of(VisionFiducials.RED_LEFT_FEEDER_TAG, VisionFiducials.BLUE_LEFT_FEEDER_TAG));
  private List<Pose2d> rightFeeders = getPoseList(
      List.of(VisionFiducials.RED_RIGHT_FEEDER_TAG, VisionFiducials.BLUE_RIGHT_FEEDER_TAG));
  private static Transform2d offset = new Transform2d(DynamicConstants.AlignTransforms.feederX, DynamicConstants.AlignTransforms.feederY, Rotation2d.fromDegrees(0));
  private static Transform2d slotSpacing_pos = new Transform2d(0.0, Inches.of(16).in(Meters), Rotation2d.fromDegrees(0));
  private static Transform2d slotSpacingneg = new Transform2d(0.0, Inches.of(16).in(Meters), Rotation2d.fromDegrees(0));
  private CommandSwerveDrivetrain dt;
  private Coral m_coral;
  private List<Pose2d> feederPoses;
  private Command drive; 
  private int time;

  /** Creates a new DriveCoralScorePose. */


  public AligntoFeeder(CommandSwerveDrivetrain drivetrain, Coral coral, int timeout) {
    dt = drivetrain;
    m_coral = coral;
    time = timeout;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    feederPoses = createLoadingSlots(leftFeeders, 0, 0);
    feederPoses.addAll(createLoadingSlots(rightFeeders, 0, 0));
  }

  public List<Pose2d> getPoseList(List<Integer> tagIntegers) {
    List<Pose2d> tags = new ArrayList<>();
    for (int i = 0; i < tagIntegers.size(); i++) {
      int tagInteger = tagIntegers.get(i);
      Pose2d tagPose = fieldLayout.getTagPose(tagInteger).get().toPose2d();
      tags.add(tagPose);
    }
    return tags;
  }

  public List<Pose2d> createLoadingSlots(List<Pose2d> tagPoses, int leftSlots, int rightSlots) {
    List<Pose2d> loadingSlots = new ArrayList<>();
    for (int i = 0; i < tagPoses.size(); i++) {
      Pose2d originPose = tagPoses.get(i).transformBy(offset);
      Pose2d tagPose = originPose;
      loadingSlots.add(tagPose);
      for (int l = 0; l < leftSlots; l++) {
        tagPose = tagPose.transformBy(slotSpacingneg);
        loadingSlots.add(tagPose);
      }
      tagPose = originPose;
      for (int r = 0; r < rightSlots; r++) {
        tagPose = tagPose.transformBy(slotSpacing_pos);
        loadingSlots.add(tagPose);
      }
    }
    return loadingSlots;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d goalPose = dt.getState().Pose.nearest(feederPoses);
    drive = PlannerSetpointGenerator.generateCommand(dt, goalPose, Time.ofBaseUnits(time, Second), false);
    drive.schedule();
  }

  @Override
  public void execute() {
    // No need to call generateCommand here, as the command is already scheduled in
    // initialize()

  }

  @Override
  public boolean isFinished() {
    return drive.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drive.cancel();
    }
  }
}
