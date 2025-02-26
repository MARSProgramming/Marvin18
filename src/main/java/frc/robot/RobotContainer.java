
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
import frc.robot.commands.HopperReturn;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.CoralArmElevator;
import frc.robot.commands.arm.SafeArm;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Agitator;
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
    public final Agitator m_Agitator = new Agitator();



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
        //Bumper and Trigger Controls
        Pilot.leftBumper().onTrue(m_algae.intake());
        Pilot.rightBumper().whileTrue(m_algae.outtake());
        Pilot.rightTrigger().whileTrue(mCoral_Hopper.runIntakeUntilIR(-1).alongWith(m_Agitator.runAgitatorWhenReading(2)));
        Pilot.leftTrigger().whileTrue(mCoral_Hopper.runIntake(1));

        //POV Controls
        Pilot.povLeft().whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityX(-0.02 * MaxSpeed).withVelocityY(0)));
        Pilot.povRight().whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityX(0.02 * MaxSpeed).withVelocityY(0)));
        Pilot.povUp().whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityY(0.02 * MaxSpeed).withVelocityX(0)));
        Pilot.povDown().whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityY(-0.02 * MaxSpeed).withVelocityX(0)));

        //Face Button Controls
        Pilot.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //Pilot.x().whileTrue(null); // Save for pose driving.
        //Pilot.b().whileTrue(null); // Save for pose driving.
        //Pilot.a().whileTrue(null); // Save for climbing.


        ///Copilot
        //Copilot.povLeft().whileTrue(m_coralArm.safeArmMovement((m_coralArm.getPosition() - .01), m_elevator));
        //Copilot.povRight().whileTrue(m_coralArm.safeArmMovement((m_coralArm.getPosition() + .01), m_elevator));
        Copilot.povUp().whileTrue(m_elevator.setMotionMagicPositionDB(m_elevator.getPositionNormal() + 0.1)); // fix
        Copilot.povDown().whileTrue(m_elevator.setMotionMagicPositionDB(m_elevator.getPositionNormal() - 0.1)); // fix
        Copilot.povUp().and(Copilot.leftBumper()).whileTrue(m_elevator.runVoltage(2));
        Copilot.povDown().and(Copilot.leftBumper()).whileTrue(m_elevator.runVoltage(-2));
        Copilot.povLeft().and(Copilot.leftBumper()).whileTrue(m_coralArm.runVoltage(-1));
        Copilot.povRight().and(Copilot.leftBumper()).whileTrue(m_coralArm.runVoltage(1));

        //Copilot.leftTrigger().onTrue(null); // Save for feeder selection
       // Copilot.rightTrigger().onTrue(null); // Save for feeder selection


        //Face Button Controls Height selection


        Copilot.y().whileTrue();
        Copilot.x().whileTrue();
        Copilot.b().whileTrue();
        Copilot.a().whileTrue(new HopperReturn(m_elevator, m_coralArm));
        Copilot.y().and(Copilot.rightBumper()).whileTrue(new SafeElevator(m_elevator, m_coralArm, Constants.ElevatorSetpointConfigs.ELEVATOR_ALGAE_TOP_SETPOINT));
        Copilot.b().and(Copilot.rightBumper()).whileTrue(new SafeElevator(m_elevator, m_coralArm, Constants.ElevatorSetpointConfigs.ELEVATOR_ALGAE_GROUND_SETPOINT));
        Copilot.x().and(Copilot.rightBumper()).whileTrue(new SafeElevator(m_elevator, m_coralArm, Constants.ElevatorSetpointConfigs.ELEVATOR_ALGAE_BOT_SETPOINT));
        Copilot.a().and(Copilot.rightBumper()).whileTrue(new SafeElevator(m_elevator, m_coralArm, Constants.ElevatorSetpointConfigs.ELEVATOR_ALGAE_TEE_SETPOINT));

      //   Copilot.leftTrigger().whileTrue(
      //     AutoBuilder.pathfindToPose(
      //       new Pose2d(14.08, 2.24, Rotation2d.fromDegrees(30)),
      //       new PathConstraints(
      //         1.0, 1.0,
      //         edu.wpi.first.math.util.Units.degreesToRadians(360), edu.wpi.first.math.util.Units.degreesToRadians(540)
      //       ),
      //       0
      //     )          
      // );


      drivetrain.registerTelemetry(logger::telemeterize);
    }


    public Command getAutonomousCommand() {
      return new Command() {
        
      };
    //   //  return new PathfindingCommand(null, null, null, null, null, null, null, null)
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
     // test.a().whileTrue(mCoral_Hopper.runCoralAgitator(-0.1));

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
