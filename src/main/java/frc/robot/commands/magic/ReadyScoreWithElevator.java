package frc.robot.commands.magic;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.IntegratedAlignWithTermination;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MagicManager;
import frc.robot.subsystems.PoseManager;

public class ReadyScoreWithElevator extends ParallelCommandGroup {
    Elevator mElevator;
    CommandSwerveDrivetrain mDrivetrain;
    PoseManager mPoseManager;
    MagicManager mMagicManager;
    boolean mLeft;
    DoubleSupplier mX, mY, mRot;

    public ReadyScoreWithElevator(Elevator elev, CommandSwerveDrivetrain dt, PoseManager pm, MagicManager mm, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, boolean left) {
        mElevator = elev;
        mDrivetrain = dt;
        mPoseManager = pm;
        mMagicManager = mm;
        mX = x;
        mY = y;
        mRot = r;
        mLeft = left;
        addCommands(
            new ReadyScore(mm, elev, dt, pm,  mX, mY,  mRot, left),
            mElevator.goToSelectedMagicCommand(mm)
        );
    }
}
