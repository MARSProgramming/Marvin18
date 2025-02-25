package frc.robot.commands.elevator;

import frc.robot.constants.Constants;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class SafeElevator extends Command {
  Elevator m_Elevator;
  CoralArm m_CoralArm;
  double mTargetElevatorPosition;

  public SafeElevator(Elevator elevator, CoralArm arm, double position) {
    m_Elevator = elevator;
    m_CoralArm = arm;
    mTargetElevatorPosition = position;

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    if (m_Elevator.getPositionNormal() > Constants.ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
      m_Elevator.setRotations(mTargetElevatorPosition);
    }
    else if (m_CoralArm.isSafe()) {
      m_Elevator.setRotations(mTargetElevatorPosition);
    } else
      m_Elevator.stopMotorHold();
  }

  @Override
  public void end(boolean interrupted) {
    m_Elevator.stopMotorHold();
  }

  @Override
  public boolean isFinished() {
    return m_Elevator.isNear(mTargetElevatorPosition);
  }

}
