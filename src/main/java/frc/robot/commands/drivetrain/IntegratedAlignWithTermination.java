package frc.robot.commands.drivetrain;

import java.lang.constant.DynamicCallSiteDesc;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.util.MoreMath;

public class IntegratedAlignWithTermination extends Command {
    Elevator mElevator;
    CommandSwerveDrivetrain mDt;
    DoubleSupplier mX, mY, mRot;
    int level;
    boolean right;

    public IntegratedAlignWithTermination(Elevator elev, CommandSwerveDrivetrain dt, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, int lev, boolean rightside) {
        mElevator = elev;
        mDt = dt;
        mX = x;
        mY = y;
        mRot = r;
        level = lev;
        right = rightside;

        addRequirements(dt); // Don't require elevator, we only want to look at its position.
    }

    @Override
    public void initialize() {

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

        mDt.integratedReefAlignment(
            right, 
            mElevator.selectedLevel, 
            xVelocity, 
            yVelocity, 
            rVelocity, 
            elevatorHeightMultiplier, 
            Constants.AlignmentConstants.kMinimumXYAlignDistance);
    }

    @Override
    public void end(boolean interrupted) {
        // "Do nothing"
        mDt.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return mDt.isAligned();
    }
}
