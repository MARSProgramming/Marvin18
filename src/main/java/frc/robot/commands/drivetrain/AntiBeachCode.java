// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.constants.Constants.VisionFiducials;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

public class AntiBeachCode extends Command {

  private CommandSwerveDrivetrain dt;
  private double pigeonAngle;

  /** Creates a new DriveCoralScorePose. */

  public AntiBeachCode(CommandSwerveDrivetrain drivetrain) {
    dt = drivetrain;
    double pigeonAngle = 0;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //pigeonAngle = dt.getPigeon2().
  }

  @Override
  public void execute() {
    // No need to call generateCommand here, as the command is already scheduled in
    // initialize()
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
    }
  }
}