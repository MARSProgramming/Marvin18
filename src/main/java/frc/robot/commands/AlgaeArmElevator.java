// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
//Import Command Class
import edu.wpi.first.wpilibj2.command.Command;
// Import Subsystems
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
// Import Constants
import frc.robot.constants.Constants.ArmSetpointConfigs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeArmElevator extends Command {
    Elevator m_Elevator;
    CoralArm m_CoralArm;
    double m_elevatorsetpoint;
  /** Creates a new ArmReturn. */
  public AlgaeArmElevator(Elevator elevator, CoralArm coralArm, double elevatorSetpoint) {
    m_Elevator = elevator;
    m_CoralArm = coralArm;
    m_elevatorsetpoint = elevatorSetpoint;
    addRequirements(m_Elevator, m_CoralArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_CoralArm.isForward()) {
      m_Elevator.setRotations(m_elevatorsetpoint);
    }
    else if(m_CoralArm.isZero()) {
      m_Elevator.setRotations(m_elevatorsetpoint);
      m_CoralArm.runVolt(-1.0);
    }
    else {
      m_CoralArm.setPosition(ArmSetpointConfigs.ARM_SAFE_POSITION + 0.05);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_Elevator.isNear(m_elevatorsetpoint)) {
      m_Elevator.setRotations(m_elevatorsetpoint);
    }
    else {
      m_Elevator.stopMotorHold();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return m_Elevator.isNear(m_elevatorsetpoint);
  }
}
