// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController Pilot = new CommandXboxController(0);
    private final CommandXboxController test = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Hopper mCoral_Hopper = new Hopper();
    public final Algae m_algae = new Algae();
    public final Elevator m_elevator = new Elevator();
    public final CoralArm m_coralArm = new CoralArm();

    public RobotContainer() {
      configureBindings();
    }

    public void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                robotDrive.withVelocityX(deadband((-Pilot.getLeftY() * 0.5) * MaxSpeed, 0.1)) // Drive forward with negative Y (forward)
                    .withVelocityY(deadband(-Pilot.getLeftX() * 0.5 * MaxSpeed, 0.1)) // Drive left with negative X (left)
                    .withRotationalRate(deadband((-Pilot.getRightX() * 0.5) * MaxAngularRate, 0.1)) // Drive counterclockwise with negative X (left)
            )
        );

        Pilot.povUp().whileTrue(m_coralArm.ArmPosVoltage(3));
        Pilot.leftTrigger().whileTrue(m_elevator.runVoltage(-1));
        Pilot.rightTrigger().whileTrue(m_elevator.runVoltage(1));
        Pilot.leftBumper().whileTrue(m_coralArm.runVoltage(0.5));
        Pilot.rightBumper().whileTrue(m_coralArm.runVoltage(-0.5));
  
        Pilot.x().whileTrue(mCoral_Hopper.runIntake(0.1));
        Pilot.b().whileTrue(mCoral_Hopper.runIntake(-0.1));
        Pilot.y().whileTrue(mCoral_Hopper.runCoralAgitator(0.1));
        Pilot.a().whileTrue(mCoral_Hopper.runCoralAgitator(-0.1));

      //  Pilot.a().whileTrue(drivetrain.applyRequest(() -> brake));
     //   Pilot.b().whileTrue(drivetrain.applyRequest(() ->
     //       point.withModuleDirection(new Rotation2d(-Pilot.getLeftY(), -Pilot.getLeftX()))
     //   ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
     //   Pilot.back().and(Pilot.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
     //   Pilot.back().and(Pilot.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      //  Pilot.start().and(Pilot.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      //  Pilot.start().and(Pilot.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
       // Pilot.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

      //  Pilot.leftTrigger().whileTrue(mCoral_Hopper.runVoltageUntilIRReading(1));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void configureTestBindings() {
      // Elevator Test Bindings
    //  test.a().whileTrue(mCoral_Hopper.runAgitator(0.1));
    //  test.x().whileTrue(m_algae.runAlgaeWheels(0.1));
      test.leftTrigger().whileTrue(m_elevator.runVoltage(1));
      test.rightTrigger().whileTrue(m_elevator.runVoltage(-1));
      test.leftBumper().whileTrue(m_coralArm.runVoltage(0.5));
      test.rightBumper().whileTrue(m_coralArm.runVoltage(-0.5));

    //  test.x().whileTrue(mCoral_Hopper.runIntake(0.1));
      test.x().whileTrue(m_algae.runAlgaeWheels(0.9));
      test.y().whileTrue(m_algae.runAlgaeWheels(-0.8));
     // test.y().whileTrue(mCoral_Hopper.runCoralAgitator(0.1));
      test.a().whileTrue(mCoral_Hopper.runCoralAgitator(-0.1));

    }


    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
    
          }
        } else {
          return 0.0;
        }
      }
}
