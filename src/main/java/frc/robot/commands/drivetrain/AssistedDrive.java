package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AssistedDrive extends Command {
    CommandSwerveDrivetrain mdt;
    DoubleSupplier mX, mY, mRot;
    double PigeonRoll, PigeonPitch;
    double vX, vY;
    double kTiltMulti;

    public AssistedDrive(CommandSwerveDrivetrain drive) {
        mdt = drive;
        addRequirements(drive);
    }

    // TODO: Let's dynamically scale the vX and vY based on tilt amount. like, vX = (TiltMulti * minimum) where tiltmulti scales off of pigeon reading
    // https://ctre.download/files/user-manual/Pigeon2%20User's%20Guide.pdf
    // page 10
    @Override
    public void initialize() {
        vX = 0;
        vY = 0;
        kTiltMulti = 1; // Maximum tilt multiplier. 
        // Describing Pitch and Roll
        // We need to put these periodically on the dashboard to measure the threshold at which we must trigger
        // overtake of driver control
        // Conventions according to pigeon documentation and wpilib (but we can test to make sure)
        // under the assumption we drive up onto the ball, we want to drive backwards based on our tilt. 
        // When pitch is positive, we are tilting to the right, so apply a swerve request to drive to the right.
        // When pitch is negative, we are tilting to the left, so apply a swerve request to drive to the left.
        // when roll is positive, we are tilting forward, so apply a swerve request to drive forward.
        // when roll is negative, we are tilting backward, so apply a swerve request to drive backward.
    }

    @Override
    public void execute() {
        PigeonRoll = mdt.getPigeon2().getRoll().getValueAsDouble();
        PigeonPitch = mdt.getPigeon2().getPitch().getValueAsDouble();
        // what should these values be?
        boolean safe = (Math.abs(PigeonRoll) < 10 && Math.abs(PigeonPitch) < 10);

        // if the command scheduler works correctly, we will basically drive off
        // as fast as possibly and driver control should return immediately. 
        if (safe) {
            mdt.fieldDrive(mX, mY, mRot, true);
        } else {
            mdt.drive(new ChassisSpeeds());
        }
    }

    @Override
    public void end(boolean interrupted) {
        mdt.drive(new ChassisSpeeds());
    }
}
