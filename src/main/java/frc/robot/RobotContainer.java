
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.opencv.video.TrackerDaSiamRPN_Params;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.auto.PathfindToPose;
import frc.robot.commands.ArmElevatorGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralCamera;
import frc.robot.subsystems.DrivetrainTelemetry;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.PathfindingSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.testing.ElevatorSysid;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController Pilot = new CommandXboxController(0);
    private final CommandXboxController Copilot = new CommandXboxController(1);
    private final CommandXboxController test = new CommandXboxController(2);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    public final Hopper mCoral_Hopper = new Hopper();
    public final Algae m_algae = new Algae();
    public final Elevator m_elevator = new Elevator();
    public final CoralArm m_coralArm = new CoralArm();

   public final PhotonVision mReef = new PhotonVision(drivetrain, "reef_cam", PoseStrategy.LOWEST_AMBIGUITY, new Transform3d(Inches.of(9.15), Inches.of(9.5), Inches.of(7.16), new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(90))));
   //public final PhotonVision mCoral = new PhotonVision(drivetrain, "feeder_cam", PoseStrategy.AVERAGE_BEST_TARGETS, new Transform3d(Inches.of(1.48), Inches.of(-10.31), Inches.of(17.54), new Rotation3d(Degrees.of(30), Degrees.of(0), Degrees.of(-93))));
    public final DrivetrainTelemetry m_Telemetry = new DrivetrainTelemetry(drivetrain, mReef);

    public RobotContainer() {
      configureBindings();
    }



    public void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(deadband(-Pilot.getLeftY(), 0.1)* 0.5 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(deadband(-Pilot.getLeftX(), 0.1) * 0.5 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(deadband(-Pilot.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // OLD DRIVE COMMAND
        /*
        Pilot.leftTrigger().whileTrue(m_elevator.runVoltage(-1));
        Pilot.rightTrigger().whileTrue(m_elevator.runVoltage(1));
       // Pilot.leftBumper().whileTrue(m_coralArm.runVoltage(0.5));
       // Pilot.rightBumper().whileTrue(m_coralArm.runVoltage(-0.5));
     //  Pilot.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
      // Pilot.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        Pilot.x().whileTrue(mCoral_Hopper.runIntake(0.8));
        Pilot.y().whileTrue(mCoral_Hopper.runIntake(-0.8));
        Pilot.a().onTrue(drivetrain.seedCentric());
        Pilot.leftBumper().whileTrue(m_algae.intake());
        Pilot.rightBumper().whileTrue(m_algae.outtake());
 */   

        Pilot.leftBumper().whileTrue(m_algae.intake());
        Pilot.rightBumper().whileTrue(m_algae.outtake());
        Pilot.povUp().whileTrue(new ArmElevatorGroup(m_elevator, m_coralArm, 4.47, 0.268));
        Pilot.povDown().whileTrue(new ArmElevatorGroup(m_elevator, m_coralArm, 0.77, 0.3));
      //  Pilot.x().whileTrue(new PathfindToPose(drivetrain));
        Pilot.rightTrigger().whileTrue(mCoral_Hopper.runIntakeUntilIR(-1));
        Pilot.leftTrigger().whileTrue(mCoral_Hopper.runIntake(1));
        Pilot.povRight().whileTrue(m_elevator.runVoltage(2));
        Pilot.povLeft().whileTrue(m_elevator.runVoltage(-2));
        Pilot.x().whileTrue(m_coralArm.runVoltage(-0.5));
        Pilot.y().whileTrue(m_coralArm.runVoltage(0.5));
        Pilot.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        Copilot.leftTrigger().whileTrue(
          AutoBuilder.pathfindToPose(
            new Pose2d(14.08, 2.24, Rotation2d.fromDegrees(30)),
            new PathConstraints(
              1.0, 1.0,
              edu.wpi.first.math.util.Units.degreesToRadians(360), edu.wpi.first.math.util.Units.degreesToRadians(540)
            ),
            0
          )          
        );
        /*
        Pilot.b().whileTrue(AutoBuilder.pathfindToPose(
          new Pose2d(13.95, 2.77, Rotation2d.fromDegrees(35)),
          new PathConstraints(
            0.6, 0.5,
            edu.wpi.first.math.util.Units.degreesToRadians(30), edu.wpi.first.math.util.Units.degreesToRadians(540)
          ),
          0
        )); */

/*
        Pilot.povLeft().whileTrue(m_elevator.setMotionMagicPosition(()-> 3));
        Pilot.povRight().whileTrue(m_coralArm.setMotionMagicPosition(() -> DynamicConstants.ArmSetpoints.armTestPos));
        Pilot.povUp().whileTrue(m_coralArm.setMotionMagicPosition(() -> 0.0));
        Pilot.povDown().whileTrue(m_elevator.setMotionMagicPosition(() -> 0.0));
        Pilot.leftTrigger().whileTrue(mCoral_Hopper.runIntake(0.8));
        Pilot.rightTrigger().whileTrue(mCoral_Hopper.runIntake(-1));
        Copilot.povUp().whileTrue(new ArmElevatorGroup(m_elevator, m_coralArm, 3, 0.3));
        Copilot.rightTrigger().whileTrue(m_elevator.runVoltage(2));
        Copilot.leftTrigger().whileTrue(m_elevator.runVoltage(-2));
        Copilot.leftBumper().whileTrue(m_elevator.SafeHopperReturn(m_coralArm).alongWith(m_coralArm.setTest(0, m_elevator)));
        Copilot.rightBumper().whileTrue(m_coralArm.runVoltage(-0.5));
        Copilot.x().whileTrue(m_coralArm.runVoltage(-1));
 */
      //  Pilot.rightBumper().onTrue(m_coralArm.ArmPosVoltage(3));
       // Pilot.leftBumper().onTrue(m_coralArm.ArmPosVoltage(1));

      //  Pilot.a().whileTrue(drivetrain.applyRequest(() -> brake));
     //   Pilot.b().whileTrue(drivetrain.applyRequest(() ->
     //       point.withModuleDirection(new Rotation2d(-Pilot.getLeftY(), -Pilot.getLeftX()))
     //   ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
     Pilot.back().and(Pilot.y()).whileTrue(m_elevator.sysIdDynamic(Direction.kForward));
      Pilot.back().and(Pilot.x()).whileTrue(m_elevator.sysIdDynamic(Direction.kReverse));
      Pilot.start().and(Pilot.y()).whileTrue(m_elevator.sysIdQuasistatic(Direction.kForward));
       Pilot.start().and(Pilot.x()).whileTrue(m_elevator.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        drivetrain.registerTelemetry(logger::telemeterize);

      //  Pilot.leftTrigger().whileTrue(mCoral_Hopper.runVoltageUntilIRReading(1));
    }

    public Command getAutonomousCommand() {
      return new Command() {
        
      };
      //  return new PathfindingCommand(null, null, null, null, null, null, null, null)
    }

    public void configureTestBindings() {
      // Elevator Test Bindings
    //  test.a().whileTrue(mCoral_Hopper.runAgitator(0.1));
    //  test.x().whileTrue(m_algae.runAlgaeWheels(0.1));
     // test.leftTrigger().whileTrue(m_elevator.runVoltage(1));
    //  test.rightTrigger().whileTrue(m_elevator.runVoltage(-1));
    //  test.leftBumper().whileTrue(m_coralArm.runVoltage(0.5));
   //   test.rightBumper().whileTrue(m_coralArm.runVoltage(-0.5));
      test.x().whileTrue(drivetrain.driveToPose());

    //  test.x().whileTrue(mCoral_Hopper.runIntake(0.1));
      //test.x().whileTrue(m_algae.intake());
      test.y().whileTrue(m_algae.outtake());
     // test.y().whileTrue(mCoral_Hopper.runCoralAgitator(0.1));
      test.a().whileTrue(mCoral_Hopper.runCoralAgitator(-0.1));

      test.povLeft().whileTrue(m_algae.intake());

      test.povRight().whileTrue(m_algae.outtake());

      test.leftBumper().whileTrue(new ElevatorSetpoint(m_elevator, 5, test.leftBumper().getAsBoolean()));
     // test.rightBumper().whileTrue(new ElevatorSetpoint(m_elevator, 5));


      
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
