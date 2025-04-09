package frc.robot.commands.drivetrain;

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
import frc.robot.subsystems.PoseManager;
import frc.robot.util.MoreMath;

public class OptimizedIntegratedAlign extends Command {
    Elevator mElevator;
    PoseManager mPoseManager;
    Pose2d closest;
    CommandSwerveDrivetrain mDt;
    DoubleSupplier mX, mY, mRot;
    int level;
    boolean left;
    String desire;

    public OptimizedIntegratedAlign(String type, PoseManager pm, Elevator elev, CommandSwerveDrivetrain dt, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, boolean leftside) {
        desire = type;
        mElevator = elev;
        mDt = dt;
        mPoseManager = pm;
        mX = x;
        mY = y;
        mRot = r;
        left = leftside;

        addRequirements(dt); // Don't require elevator, we only want to look at its position.
    }

    @Override
    public void initialize() {
        closest = mPoseManager.returnOptimalPose(mDt.getState().Pose, desire, level, left);
    }

    @Override
    public void execute() {
        // the first parameter should be fed in meters (some measure of the position converted to meters)
        double elevatorHeightMultiplier = MoreMath.interpolate(
            mElevator.getPositionNormal(),
            0.0, DynamicConstants.ElevatorSetpoints.elevMax,
            1.0, Constants.AlignmentConstants.kMinimumElevatorMultiplier);

        double transMultiplier = Constants.AlignmentConstants.TeleoperatedMaximumVelocity.in(Units.MetersPerSecond) * elevatorHeightMultiplier;

    // figure out signs later, right now im just going off how the CTRE designates forward/backward movement and rotation which
    // is negative of driver input
        LinearVelocity xVelocity = Units.MetersPerSecond.of(-mX.getAsDouble() * transMultiplier);
        LinearVelocity yVelocity = Units.MetersPerSecond.of(-mY.getAsDouble() * transMultiplier);
        AngularVelocity rVelocity = Units.RadiansPerSecond
        .of(-mRot.getAsDouble() * Constants.AlignmentConstants.kMaximumRotSpeed.in(Units.RadiansPerSecond)
            * elevatorHeightMultiplier);
        Distance distFromReef = Units.Meters.of(mDt.getState().Pose.getTranslation().getDistance(closest.getTranslation()));
        mDt.ReefAlign2(distFromReef, closest, left, level, xVelocity, yVelocity, rVelocity, elevatorHeightMultiplier, distFromReef);
    }

    @Override
    public void end(boolean interrupted) {
        // "Do nothing"
        mDt.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
