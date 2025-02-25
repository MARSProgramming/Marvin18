// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.arm.SafeArm;
import frc.robot.commands.elevator.SafeElevator;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ArmSetpointConfigs;
import frc.robot.constants.Constants.ElevatorSetpointConfigs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HopperReturn extends Command {
    Elevator m_Elevator;
    CoralArm m_CoralArm;
  /** Creates a new ArmReturn. */
  public HopperReturn(Elevator elevator, CoralArm coralArm) {
    m_Elevator = elevator;
    m_CoralArm = coralArm;
    addRequirements(m_Elevator, m_CoralArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_CoralArm.getPosition() < 0.05){
      m_Elevator.setRotations(0.0);
      m_CoralArm.runVolt(-1.0);
    }
    else if (m_Elevator.isSafe()) {
      m_Elevator.setRotations(Constants.ElevatorSetpointConfigs.ELEVATOR_SCORE_POSITION);
      m_CoralArm.setPosition(0.0);
    }
    else {
      m_Elevator.setRotations(Constants.ElevatorSetpointConfigs.ELEVATOR_SCORE_POSITION);
      if (m_CoralArm.getPosition() > Constants.ArmSetpointConfigs.ARM_SCORE_POSITION) {
       m_CoralArm.setPosition(Constants.ArmSetpointConfigs.ARM_SCORE_POSITION);}
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_Elevator.isNear(0.0)) {
      m_Elevator.setRotations(0.0);
      m_CoralArm.setPosition(0.0);
    }
    else {
      m_Elevator.stopMotor();
      m_CoralArm.stopMotor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return m_Elevator.isNear(0.0);
  }
}
