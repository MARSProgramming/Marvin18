package frc.robot.commands.magic;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MagicManager;
import frc.robot.subsystems.PoseManager;
import frc.robot.util.MoreMath;

public class ReadyScore extends Command {
    MagicManager mMagic;
    Elevator mElevator;
    PoseManager mPoseManager;
    CommandSwerveDrivetrain mDt;
    DoubleSupplier mX, mY, mRot;


    Pose2d pos;
    int lev;
    boolean left;
    boolean runMagic;



    public ReadyScore(MagicManager magic, Elevator elev, CommandSwerveDrivetrain dt, PoseManager pm, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, boolean leftside) {
        mPoseManager = pm;
        mMagic = magic;
        mElevator = elev;
        mDt = dt;
        mX = x;
        mY = y;
        mRot = r;
        left = leftside;
    }

    @Override
    public void initialize() {
        lev = mMagic.getLevel();
        pos = mPoseManager.returnOptimalPose(mDt.getState().Pose, "reef", lev, left);
        runMagic = mMagic.getMagic();
    }

    @Override
    public void execute() {
        double elevatorHeightMultiplier = MoreMath.interpolate(
            mElevator.getPositionNormal(),
            0.0, DynamicConstants.ElevatorSetpoints.elevMax,
            1.0, Constants.AlignmentConstants.kMinimumElevatorMultiplier);

        double transMultiplier = Constants.AlignmentConstants.TeleoperatedMaximumVelocity.in(Units.MetersPerSecond) * elevatorHeightMultiplier;

        LinearVelocity xVelocity = Units.MetersPerSecond.of(-mX.getAsDouble() * transMultiplier);
        LinearVelocity yVelocity = Units.MetersPerSecond.of(-mY.getAsDouble() * transMultiplier);
        AngularVelocity rVelocity = Units.RadiansPerSecond
        .of(-mRot.getAsDouble() * Constants.AlignmentConstants.kMaximumRotSpeed.in(Units.RadiansPerSecond)
            * elevatorHeightMultiplier);
        Distance distFromReef = Units.Meters.of(mDt.getState().Pose.getTranslation().getDistance(pos.getTranslation()));
        mDt.ReefAlign2(distFromReef, pos, left, lev, xVelocity, yVelocity, rVelocity, elevatorHeightMultiplier, distFromReef);

        if (runMagic) {
            if (lev == 4) {
                mElevator.setRotations(DynamicConstants.ElevatorSetpoints.elevL4);
            }
            if (lev == 3) {
                mElevator.setRotations(DynamicConstants.ElevatorSetpoints.elevL3);
            }
            if (lev == 2) {
                mElevator.setRotations(DynamicConstants.ElevatorSetpoints.elevL2);
            }
            if (lev == 1) {
            // add "slow zero" before testing
                mElevator.setRotations(DynamicConstants.ElevatorSetpoints.elevL1);
            }
        }
    }


    @Override
    public void end(boolean interrupted) {
        // "Do nothing"
        mDt.drive(new ChassisSpeeds());
        if (runMagic) {
            if (mElevator.getPositionNormal() != 0 || (!mElevator.getLimit())) {
                mElevator.stopMotorHold();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
