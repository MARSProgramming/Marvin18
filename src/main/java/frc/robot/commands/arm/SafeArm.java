package frc.robot.commands.arm;

import frc.robot.constants.Constants;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class SafeArm extends Command {
    Elevator m_Elevator;
    CoralArm m_CoralArm;
    double mTargetArmPosition;

    public SafeArm(Elevator elevator, CoralArm arm, double position) {
        m_Elevator = elevator;
        m_CoralArm = arm;
        mTargetArmPosition = position;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (m_Elevator.getPositionNormal() > Constants.ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
            m_CoralArm.setPosition(mTargetArmPosition);
        } else if (m_CoralArm.getPosition() > Constants.ArmSetpointConfigs.ARM_SAFE_POSITION
                && mTargetArmPosition > Constants.ArmSetpointConfigs.ARM_SAFE_POSITION) {
            m_CoralArm.setPosition(mTargetArmPosition);
        } else {
            m_CoralArm.stopMotorHold();}
    }

    @Override
    public void end(boolean interrupted) {
        if (m_CoralArm.isNear(mTargetArmPosition)) {
            m_CoralArm.setPosition(mTargetArmPosition);
        }
        else {
            m_CoralArm.stopMotorHold();}
    }

    @Override
    public boolean isFinished() {
        return m_CoralArm.isNear(mTargetArmPosition);
    }

}
