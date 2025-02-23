// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.constants.Constants.ArmSetpointConfigs;
import frc.robot.constants.Constants.ElevatorSetpointConfigs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmElevatorGroup extends SequentialCommandGroup {
  private Elevator m_Elevator;
  private CoralArm m_CoralArm;
  private double mTargetElevatorPosition;
  private double mTargetArmPosition;


  /** Creates a new SafeMoveArm. */
  public ArmElevatorGroup(Elevator elevator, CoralArm coralArm, double elevtarg, double armtarg) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = elevator;
    m_CoralArm = coralArm;
    addRequirements(m_Elevator, m_CoralArm);
    mTargetElevatorPosition = elevtarg;
    mTargetArmPosition = armtarg;
    
    if (m_Elevator.getPositionNormal() > ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
        addCommands(
            m_CoralArm.setMotionMagicPosition(() -> mTargetArmPosition).
            andThen(m_Elevator.setMotionMagicPosition(() -> mTargetElevatorPosition))
            );
        }

    else if (m_Elevator.getPositionNormal() < ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
        addCommands(
            m_Elevator.setMotionMagicPosition(() -> ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION).
            andThen(m_CoralArm.setMotionMagicPosition(() -> mTargetArmPosition))
            .andThen(m_Elevator.setMotionMagicPosition(() -> mTargetElevatorPosition))
            .andThen(m_CoralArm.setMotionMagicPosition(() -> mTargetArmPosition))             
            );
        }
    
  }
/*
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_CoralArm.isNear(ArmSetpointConfigs.ARM_REVERSE_LIMIT)) {
      if (m_Elevator.getPositionNormal() > ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
       new ArmSetpoint(m_CoralArm, mTargetArmPosition).andThen(new ElevatorSetpoint(m_Elevator, mTargetElevatorPosition));
      }
      if (m_Elevator.getPositionNormal() < ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
        new ElevatorSetpoint(m_Elevator, ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION).
        andThen(new ArmSetpoint(m_CoralArm, mTargetArmPosition)).
        andThen(new ElevatorSetpoint(m_Elevator, mTargetElevatorPosition)); 
      }

    } else if (m_CoralArm.isNear(ArmSetpointConfigs.ARM_REVERSE_LIMIT)) {
      if (mTargetElevatorPosition > ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
        new ArmSetpoint(m_CoralArm, mTargetArmPosition).andThen(new ElevatorSetpoint(m_Elevator, mTargetElevatorPosition));
      }
      if (mTargetElevatorPosition < ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
        new ElevatorSetpoint(m_Elevator, ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION).
        andThen(new ArmSetpoint(m_CoralArm, mTargetArmPosition)).
        andThen(new ElevatorSetpoint(m_Elevator, mTargetElevatorPosition));        
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CoralArm.runVoltage(0);
    m_Elevator.runVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_Elevator.isNear(mTargetElevatorPosition) && m_CoralArm.isNear(mTargetArmPosition));
  }


   /* 
  public Command setTest(double position, Elevator elevator) {
    return new Command() {
        @Override
        public void execute() {
            if (elevator.getPositionNormal() > 6) {
                arm.setControl(motionMagicRequest.withPosition(position));
            } else
                return;

        }

        @Override
        public void end(boolean interrupted) {
            arm.set(0);
        }

        @Override
        public boolean isFinished() {
            if (arm.getPosition().getValueAsDouble() > (position - 0.01)
                    && arm.getPosition().getValueAsDouble() < (position + 0.01)) {
                return true;
            }
            return false;
        }
    };
}  
  
  */
}
