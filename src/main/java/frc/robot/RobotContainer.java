
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorAlgaeComand;
import frc.robot.commands.drivetrain.CenterAlign;
import frc.robot.commands.drivetrain.FeederAlign;
import frc.robot.commands.drivetrain.IntegratedAlign;
import frc.robot.commands.drivetrain.IntegratedAlignWithTermination;
import frc.robot.commands.drivetrain.planner.AlgaeAlign;
import frc.robot.commands.drivetrain.planner.AligntoFeeder;
import frc.robot.commands.drivetrain.planner.DriveCoralScorePose;
import frc.robot.commands.drivetrain.planner.PoseSelector;
import frc.robot.commands.drivetrain.planner.PoseSelector2;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.DrivetrainTelemetry;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntegratedVision;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDSection;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
          SteerRequestType.MotionMagicExpo); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
          SteerRequestType.MotionMagicExpo); // Use Closed-loop control for drive motors at low speeds
  
  private final SwerveRequest.ApplyRobotSpeeds robot = new SwerveRequest.ApplyRobotSpeeds();

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle();

  private final SwerveRequest.ApplyRobotSpeeds doNothing = new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds());

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController Pilot = new CommandXboxController(0);
  private final CommandXboxController Copilot = new CommandXboxController(1);
  private final CommandXboxController test = new CommandXboxController(2);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SendableChooser<Command> autoChooser;

  public final Timer m_timer = new Timer();


  public final Algae m_algae = new Algae();
  public final Elevator m_elevator = new Elevator();
  public final Coral m_coral = new Coral();
  public final Vision reef_vision = new Vision(Constants.Vision.reefCameraName, Constants.Vision.reefRobotToCam);
  public final Vision feeder_vision = new Vision(Constants.Vision.feederCameraName, Constants.Vision.feederRobotToCam);

  public final IntegratedVision integVis = new IntegratedVision(drivetrain);
 public final LED led = new LED(40);
  public final DrivetrainTelemetry m_Telemetry = new DrivetrainTelemetry(drivetrain);
  private final Trigger readyToPlaceCoral = new Trigger(() -> (DriverStation.isTeleop() && drivetrain.isAligned()));
  private final Trigger algaeWarning = new Trigger(() -> drivetrain.notSafe());
  private final Trigger hasCoralTrigger = new Trigger(() -> m_coral.hasCoral());



  public RobotContainer() {
    drivetrain.setStateStdDevs(VecBuilder.fill(0.01, 0.01, Math.toRadians(5)));
    configureBindings();
    configureLEDTriggers();

    NamedCommands.registerCommand("Nearest Tag Align Left",
        new DriveCoralScorePose(drivetrain,
            new Transform2d(DynamicConstants.AlignTransforms.LeftXL4, DynamicConstants.AlignTransforms.LeftYL4,
                Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.LeftRot)), 3));
    NamedCommands.registerCommand("Nearest Tag Align Center",
        new DriveCoralScorePose(drivetrain, new Transform2d(DynamicConstants.AlignTransforms.CentX,
            DynamicConstants.AlignTransforms.CentY, Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.CentRot)), 1));
    NamedCommands.registerCommand("Nearest Tag Align Right",
        new DriveCoralScorePose(drivetrain,
            new Transform2d(DynamicConstants.AlignTransforms.RightXL4, DynamicConstants.AlignTransforms.RightYL4,
                Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.RightRot)), 2));
    NamedCommands.registerCommand("Deploy Algae",
                m_algae.intakeWithStop().withTimeout(0.6));

    NamedCommands.registerCommand("Feeder Align", new AligntoFeeder(drivetrain, m_coral, 5));
    NamedCommands.registerCommand("Elevator Setpoint L1",
        m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL1));
    NamedCommands.registerCommand("Elevator Setpoint L2",
        m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL2));
    NamedCommands.registerCommand("Elevator Setpoint L3",
        m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL3));
    NamedCommands.registerCommand("Elevator Setpoint L4",
        m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL4).withTimeout(2));
    NamedCommands.registerCommand("Elevator Setpoint Algae Ground",
        m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeGround));
    NamedCommands.registerCommand("Elevator Setpoint Algae Processor",
        m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeTee));
    NamedCommands.registerCommand("Elevator Setpoint Algae Bottom",
        m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeBot));
    NamedCommands.registerCommand("Elevator Setpoint Algae Top",
        m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeTop));
    NamedCommands.registerCommand("Zero Elevator", m_elevator.zeroElevatorCommand().withTimeout(2)); // ensure that the
                                                                                                     // robot stops
                                                                                                     // running elevator
                                                                                                     // down if limit
                                                                                                     // isn't read.
    NamedCommands.registerCommand("Score", m_coral.runIntake(1).withTimeout(0.5));
    NamedCommands.registerCommand("Score Faster", m_coral.runIntake(1).withTimeout(0.3));
    NamedCommands.registerCommand("Passive Intake", m_coral.coralCheck());
    NamedCommands.registerCommand("Alt Right Side Align", new IntegratedAlignWithTermination(m_elevator, drivetrain, () -> 5, () -> 5,  () -> Constants.AlignmentConstants.kMaximumRotSpeed.baseUnitMagnitude(), 4, false).withTimeout(2));
    NamedCommands.registerCommand("Alt Left Side Align", new IntegratedAlignWithTermination(m_elevator, drivetrain, () -> 5, () -> 5,  () -> Constants.AlignmentConstants.kMaximumRotSpeed.baseUnitMagnitude(), 4, true).withTimeout(2));

    configureLEDTriggers();

    // Set the robot state standard deviation.
    // This sets trust in built-in robot odometry.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  public void configureBindings() {

    readyToPlaceCoral.onTrue(Commands.runOnce(
      () -> Pilot.setRumble(RumbleType.kBothRumble, 0.4)).withTimeout(1).alongWith(led.setStrobeAnimationCommand(255, 165, 0, .2).withTimeout(.6).andThen(led.setLEDColorCommand(255, 0, 0)).withTimeout(2)))
      .onFalse(Commands.runOnce(
          () -> Pilot.setRumble(RumbleType.kBothRumble, 0)));

    algaeWarning.onTrue(drivetrain.DoNothingForTime(1.5));

    
      
    m_coral.setDefaultCommand(m_coral.runIntake(-0.2));
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(deadband(-Pilot.getLeftY(), 0.1) * 0.7 * MaxSpeed) // Drive
                                                                                                             // forward
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // Y (up)
            .withVelocityY(deadband(-Pilot.getLeftX(), 0.1) * 0.7 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(deadband(-Pilot.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
        ));
    // Bumper and Trigger Controls
    Pilot.leftBumper().whileTrue(new ElevatorAlgaeComand(m_elevator, m_algae));
    Pilot.rightBumper().whileTrue(m_algae.outtake());
    Pilot.rightTrigger().whileTrue(m_coral.runIntake(1));
    Pilot.leftTrigger().onTrue(m_elevator.zeroElevatorCommand());


    // POV Controls
    Pilot.povLeft()
        .whileTrue(drivetrain.applyRequest(
            () -> robotDrive.withVelocityX(-DynamicConstants.Drive.leftRight * MaxSpeed).withVelocityY(0)));
    Pilot.povRight()
        .whileTrue(drivetrain.applyRequest(
            () -> robotDrive.withVelocityX(DynamicConstants.Drive.leftRight * MaxSpeed).withVelocityY(0)));
    Pilot.povUp()
        .whileTrue(drivetrain.applyRequest(
            () -> robotDrive.withVelocityY(DynamicConstants.Drive.forwardBackward * MaxSpeed).withVelocityX(0)));
    Pilot.povDown()
        .whileTrue(drivetrain.applyRequest(
            () -> robotDrive.withVelocityY(-DynamicConstants.Drive.forwardBackward * MaxSpeed).withVelocityX(0)));

    // Face Button Controls
    Pilot.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    Pilot.a().whileTrue(new AlgaeAlign(m_elevator, drivetrain, 
    () -> deadband(-Pilot.getLeftY(), 0.1) * 0.7 * MaxSpeed, 
    () -> deadband(-Pilot.getLeftX(), 0.1) * 0.7 * MaxSpeed, 
   () -> deadband(-Pilot.getRightX(), 0.1) * MaxAngularRate));
    //Pilot.y().whileTrue(new DriveCoralScorePose(
    //    drivetrain, new Transform2d(DynamicConstants.AlignTransforms.CentX, DynamicConstants.AlignTransforms.CentY,
     //       Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.CentRot)), 10));


     Pilot.y().whileTrue(new CenterAlign(m_elevator, drivetrain, 
     () -> deadband(-Pilot.getLeftY(), 0.1) * 0.7 * MaxSpeed, 
     () -> deadband(-Pilot.getLeftX(), 0.1) * 0.7 * MaxSpeed, 
    () -> deadband(-Pilot.getRightX(), 0.1) * MaxAngularRate));
         // Alternative bindings
    // Pilot.x().whileTrue(poseSelector);
    // Pilot.b().onTrue(m_elevator.goToSelectedPointCommand());

    Pilot.b().whileTrue(new IntegratedAlign(m_elevator, drivetrain, 
    () -> deadband(-Pilot.getLeftY(), 0.1) * 0.7 * MaxSpeed, 
    () -> deadband(-Pilot.getLeftX(), 0.1) * 0.7 * MaxSpeed, 
   () -> deadband(-Pilot.getRightX(), 0.1) * MaxAngularRate, 4, false));
    

    Pilot.x().whileTrue(new IntegratedAlign(m_elevator, drivetrain, 
    () -> deadband(-Pilot.getLeftY(), 0.1) * 0.7 * MaxSpeed, 
   () -> deadband(-Pilot.getLeftX(), 0.1) * 0.7 * MaxSpeed, 
    () -> deadband(-Pilot.getRightX(), 0.1) * MaxAngularRate, 4, true));
    
    /// Copilot
    /// Elevator and drive controls
    Copilot.povUp().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeTop));
    Copilot.povDown()
        .onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeGround));
    Copilot.povLeft().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeTee));
    Copilot.povRight()
        .onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeBot));
    Copilot.leftBumper().onTrue(drivetrain.setSide(0));
    Copilot.rightBumper().onTrue(drivetrain.setSide(1));
    Copilot.rightTrigger().onTrue(m_elevator.goToSelectedPointCommand());

    Copilot.start().whileTrue(m_elevator.climbingCommand());
    Copilot.back().whileTrue(m_elevator.setServoCommand(0));

    // Face Button Controls Height selection

    Copilot.a().onTrue(m_elevator.zeroElevatorCommand()); // Save for height selection
    Copilot.b().onTrue(m_elevator.setLevel(3));
    Copilot.x().onTrue(m_elevator.setLevel(2));
    Copilot.y().onTrue(m_elevator.setLevel(4));


    Copilot.leftStick().onTrue(m_elevator.advanceRotationsCommand(-0.1));
    Copilot.rightStick().onTrue(m_elevator.advanceRotationsCommand(0.1));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log

    Pilot.back().and(Pilot.y()).whileTrue(m_elevator.sysIdDynamic(Direction.kForward));
    Pilot.back().and(Pilot.x()).whileTrue(m_elevator.sysIdDynamic(Direction.kReverse));
    Pilot.start().and(Pilot.y()).whileTrue(m_elevator.sysIdQuasistatic(Direction.kForward));
    Pilot.start().and(Pilot.x()).whileTrue(m_elevator.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press

    drivetrain.registerTelemetry(logger::telemeterize);

    // Pilot.leftTrigger().whileTrue(mCoral_Hopper.runVoltageUntilIRReading(1));
  }

  public Command configureBindingsCommand() {
    double currTimeSeconds = Timer.getFPGATimestamp();
    return new Command() {
      @Override
      public void initialize() {
        configureBindings();
      }

      @Override
      public boolean isFinished() {
        return (Timer.getFPGATimestamp() - currTimeSeconds > .5);
      }
    };
  }

  public Command getAutonomousCommand() {
   // m_coral.setDefaultCommand(m_coral.runIntake(-0.2));
    return autoChooser.getSelected();
    // return new Command() {
    //
    // };
    // return new PathfindingCommand(null, null, null, null, null, null, null, null)
  }

  public void configureTestBindings() {

    test.leftTrigger().whileTrue(m_algae.intake());

    test.rightTrigger().whileTrue(m_algae.outtake());

    test.povDown().whileTrue(m_elevator.setServoCommand(0));
    test.povLeft().whileTrue(m_elevator.setServoCommand(0.5));
    test.povUp().whileTrue(m_elevator.setServoCommand(1));
    // test.povRight().whileTrue(m_elevator.setServo(45));

    test.leftBumper().whileTrue(m_elevator.testVoltageCommand(-2));
    test.rightBumper().whileTrue(m_elevator.testVoltageCommand(2));
    test.a().whileTrue(m_coral.runIntake(0.5));
    test.b().whileTrue(m_coral.runIntake(-0.5));

    // test.rightBumper().whileTrue(new ElevatorSetpoint(m_elevator, 5));

  }

  private void configureLEDTriggers() {
    //pilot
      //coral score - green
      Pilot.rightTrigger().onTrue(led.setLEDColorCommand(0, 255, 0).withTimeout(.6).andThen(led.setLEDColorCommand(255, 0, 0)).withTimeout(2));
      // Feeder align: Turn purple
      Pilot.a().whileTrue(led.setLEDColorCommand(184, 0, 185));
      // Reef align - turn yellow
      Pilot.b().whileTrue(led.setLEDColorCommand(255, 165, 0)); 
      // Reef align - turn yellow
      Pilot.x().whileTrue(led.setLEDColorCommand(255, 165, 0));
      // Algae align - turn blue 
      Pilot.y().whileTrue(led.setLEDColorCommand(0, 0, 255)); 

    //copilot
      // L2 - turn "yellow"
      Copilot.x().whileTrue(led.setLEDColorCommand(128, 255, 0));
      // L3 - turn "green"
      Copilot.b().whileTrue(led.setLEDColorCommand(255, 127, 0));
      // L4 - turn "orange"
      Copilot.y().whileTrue(led.setLEDColorCommand(255, 255, 0));
      //processor - green
      Copilot.povLeft().whileTrue(led.setLEDColorCommand(0, 255, 0));
      //Algae1 - strobe green
      Copilot.povRight().onTrue(led.setStrobeAnimationCommand(0, 255, 0, .4).andThen(new WaitCommand(1)).andThen(led.setLEDColorCommand(255, 0, 0)));
      // climbing - strobe green
      Copilot.start().onTrue(led.setStrobeAnimationCommand(0, 0, 255, .4).andThen(new WaitCommand(1)).andThen(led.setLEDColorCommand(255, 0, 0)));
      //Elevator to setpoint - turn red
      Copilot.rightTrigger().whileTrue(led.setLEDColorCommand(255, 0, 0));
  
   //has coral - flash white
   hasCoralTrigger.onTrue(led.setStrobeAnimationCommand(255, 255, 255, .2).withTimeout(1).andThen(led.setLEDColorCommand(255, 0, 0)).withTimeout(2));

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
