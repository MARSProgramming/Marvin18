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

    public AssistedDrive(CommandSwerveDrivetrain drive) {
        mdt = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
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
            // but what speed do we drive at? 
            if (PigeonRoll < 0) { // so if roll is less than 0, we are tilting backward. Drive backward, robot relative. 
                vX = -1;
                // is 1 meter per second fast enough? Does this work?
            }
            if (PigeonRoll > 0) {
                vX = 1;
            }
            if (PigeonPitch < 0) {
                vY = 1;
            }
            if (PigeonPitch > 0) {
                vY = -1;
            }
            // this logic is intended to account for scenarios where we drive a corner into an algae
            // but it may be too aggressive and cause us to flip or something
            // I don't think we want to introduce complex angular calculations because this should work for
            // most use cases. 
            mdt.drive(new ChassisSpeeds(
                vX, vY, 0
            ));
        }
    }

    @Override
    public void end(boolean interrupted) {
        mdt.drive(new ChassisSpeeds());
    }
}
