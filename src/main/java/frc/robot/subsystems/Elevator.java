package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.constants.Constants;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.Constants.ElevatorSetpointConfigs;
import frc.robot.constants.DynamicConstants;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.DynamicConstants.ElevatorSetpoints;

public class Elevator extends SubsystemBase {
  private TalonFX master; // right SIDE MOTOR
  private TalonFX follower; // left SIDE MOTOR
  Servo servo = new Servo(Constants.PWM_IDS.SERVO);
  private double positionCoefficient = 1 / 12;
  private final VoltageOut sysIdVoltage = new VoltageOut(0);
  Time sysIdTimeout = Time.ofBaseUnits(5, Units.Second);
  // Velocity<VoltageUnit> rampRate = Velocity.ofBaseUnits(1.0,
  // VelocityUnit.combine(Units.Volt.getBaseUnit(), 1.0));
  Velocity<VoltageUnit> rampRate = Velocity.ofBaseUnits(0.3, VelocityUnit.combine(Volt, Second));

  private DigitalInput climbLimit;
  private double mostRecentTarget; // in rotations, converted using position coefficient.

  public boolean elevatorZeroInProgress = false;
  public boolean elevatorZeroed = false;
  public double deadzoneDist = 1.0 / Constants.ElevatorSetpointConfigs.INCHES_PER_ROTATION; // the deadzone distance, in adjusted motor rotations
  VoltageOut voltageRequest;
  PositionVoltage positionVoltageRequest;
  MotionMagicVoltage motionRequest;

  public Angle currentElevatorRightPos;
  public Angle currentElevatorLeftPos;
  public Distance elevatorRotationsInInchesMulti;
  public Integer selectedLevel = 4;

  public Elevator() {

    // Motors
    follower = new TalonFX(Constants.CAN_IDS.ELEVATOR.ELEVATOR_FOLLOWER, "CAN-2"); // left SIDE MOTOR
    master = new TalonFX(Constants.CAN_IDS.ELEVATOR.ELEVATOR_MASTER, "CAN-2"); // RIGHT SIDE MOTOR
    climbLimit = new DigitalInput(Constants.DIO_IDS.CLIMB_LIMIT);
    // Elevator Speed/Target Configs
    mostRecentTarget = 0; // configure units before testing - get in terms of encoder positions
    voltageRequest = new VoltageOut(0);
    motionRequest = new MotionMagicVoltage(0);
    positionVoltageRequest = new PositionVoltage(0);
    currentElevatorLeftPos = Rotations.of(0); // Follower
    currentElevatorRightPos = Rotations.of(0); // Master
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    

    motionRequest.EnableFOC = true;
    voltageRequest.EnableFOC = true;
    positionVoltageRequest.EnableFOC = true;

    masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ElevatorSetpointConfigs.ELEVATOR_FORWARD_LIMIT;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ElevatorSetpointConfigs.ELEVATOR_REVERSE_LIMIT;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // TESTING ONLY
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TESTING ONLY
    // masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    masterConfig.CurrentLimits.SupplyCurrentLimit = 54;
    masterConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    masterConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

    masterConfig.CurrentLimits.StatorCurrentLimit = 80;
    masterConfig.Feedback.SensorToMechanismRatio = 12;
    masterConfig.Voltage.PeakForwardVoltage = 16;
    masterConfig.Voltage.PeakReverseVoltage = -16;

    masterConfig.Slot0.kP = 18;
    masterConfig.Slot0.kI = 2;
    masterConfig.Slot0.kD = 2;
    masterConfig.Slot0.kG = 0.3;
    masterConfig.Slot0.kS = 0.1;
    masterConfig.MotionMagic.MotionMagicCruiseVelocity = 35;
    masterConfig.MotionMagic.MotionMagicAcceleration = 70;

    masterConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    master.getConfigurator().apply(masterConfig);
    follower.getConfigurator().apply(masterConfig);
    // master.getConfigurator().apply(new
    // MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    follower.setControl(new Follower(master.getDeviceID(), true));
    // leftLimit = new DigitalInput(0);
    // rightLimit = new DigitalInput(0);

    master.setPosition(0.0); // resets elevator encoders to 0
    follower.setPosition(0.0);

  }

  /// Methods to Move motor

  public void ratchetLock(double position) {
    servo.set(position);
  }

  public void setRotations(Double rotations) {
    master.setControl(motionRequest.withPosition(rotations));
  }

  public void advanceRotations(double rotations) {
    if (!getLimit())
      master.setControl(motionRequest.withPosition(getPositionNormal() + rotations));
  }

  public void runVoltageRequest(double voltage) {
    master.setControl(voltageRequest.withOutput(voltage));
  }

  /// Methods to Stop motor
  public void stopMotorHold() {
    master.setControl(motionRequest.withPosition(master.getPosition().getValueAsDouble()));
  }

  public void stopMotor() {
    master.set(0);
  }

  /// Methods to get positions/states
  public double getLastDesiredPosition() {
    return mostRecentTarget;
  }

  public double getAdjustedRotations() {
    return master.getPosition().getValueAsDouble() * positionCoefficient;
  }

  public double getPositionNormal() {
    return master.getPosition().getValueAsDouble();
  }

  /// Boolean Methods
  public boolean getLimit() {
    return !climbLimit.get();
  }

  public boolean isAtSetpoint() {
    double offset = getAdjustedRotations() - getLastDesiredPosition();
    if (offset > 0) {
      return ((offset - deadzoneDist) > 0);
    }
    if (offset < 0) {
      return ((offset + deadzoneDist) > 0);
    } else {
      return false;
    }
  }

  public boolean isNear(double rotations) {
    return Math.abs(getPositionNormal() - rotations) < ElevatorSetpointConfigs.ELEVATOR_DEADZONE_DIST;
  }

  public AngularVelocity getSpinVelocity() {
    return master.getRotorVelocity().getValue();
  }

  public boolean isStopped() {
    return getSpinVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setNeutral() {
    master.setControl(new NeutralOut());
    follower.setControl(new NeutralOut());
  }

  public boolean isPosNearZero() {
    return Math.abs(getAdjustedRotations()) < Constants.ElevatorSetpointConfigs.ELEVATOR_ROTATIONS_DEADZONE;
  }

  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    master.getConfigurator().apply(masterConfig);
    follower.getConfigurator().apply(masterConfig);
  }

  public void resetSensorPosition(Distance setpoint) {
    master.setPosition(setpoint.in(Inches));
    follower.setPosition(setpoint.in(Inches));
  }

  ///// Commands

  public Command advanceRotationsCommand(double rotations) {
    return runOnce(() -> {
        ratchetLock(0);
        advanceRotations(rotations);});
  }

  public Command setMotionMagicPositionCommand(double rotations) {
    return runEnd(() -> {
      ratchetLock(0);
      setRotations(rotations);
    }, () -> {
      stopMotorHold();
    }).until(
        () -> isNear(rotations));
  }

  public Command goToSelectedPointCommand() {
    return runEnd(() -> {
    if (selectedLevel == 2) {
      setRotations(ElevatorSetpoints.elevL2);
    }
    if (selectedLevel == 3) {
      setRotations(ElevatorSetpoints.elevL3);
    }
    if (selectedLevel == 4) {
      setRotations(ElevatorSetpoints.elevL4);
    }
    else zeroElevatorCommand();
  }, () -> {
    stopMotorHold();
  }).until(
    () -> isNear(selectedLevel == 2 ? ElevatorSetpoints.elevL2 : 
                 selectedLevel == 3 ? ElevatorSetpoints.elevL3 : 
                 selectedLevel == 4 ? ElevatorSetpoints.elevL4 : 0));
  }

  
  public Command setLevel(int level) {
    return runOnce(() -> {
      selectedLevel = level;
    });
  }

  public Command runVoltageJoystickCommand(double percentage) {
    double minimumVoltage = 0.5;
    double kV = 3;
    return runEnd(() -> {
      ratchetLock(0);
      if (percentage > 0){
        runVoltageRequest(percentage * kV + minimumVoltage);
      } else if (percentage < 0){
        runVoltageRequest(percentage * kV - minimumVoltage);
      } else {
        stopMotorHold();
      }
    }, () -> {
      stopMotorHold();
    });
  }
  
  public Command runVoltageCommand(double voltage) {
    return runEnd(() -> {
      ratchetLock(0);
      runVoltageRequest(voltage);
    }, () -> {
      stopMotorHold();
    });
  }

  public Command testVoltageCommand(double voltage) {
    return runEnd(() -> {
      runVoltageRequest(voltage);
    }, () -> {
      master.set(0);
    });
  }

  public Command resetPositionsCommand(double position) {
    return runEnd(() -> {
      master.setControl(new DutyCycleOut(0));
      master.setPosition(position);
    }, () -> master.setPosition(position));
  }

  public Command setServoCommand(double position) {
    return runEnd(() -> {
      ratchetLock(position);
    }, () -> {
      ratchetLock(position);
    });
  }

  public Command climbingCommand() {
    return runEnd(() -> {
      ratchetLock(0.5);
      master.setControl(voltageRequest.withOutput(DynamicConstants.ElevatorSetpoints.elevClimbVoltage));
    }, () -> {
      stopMotor();
    }).until(
        () -> getLimit());
  }

  public Command zeroElevatorCommand() {
    return runEnd(() -> {
      ratchetLock(0);
      if(getPositionNormal() > 1.5) {
      setRotations(0.0);}
      else  {
        runVoltageRequest(-6);
      }
    }, () -> {
      stopMotor();
    }).until(
        () -> getLimit()
        ).withTimeout(3); // three second timeout
  }

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          rampRate, // Use default ramp rate (1 V/s)
          Volts.of(1), // Reduce dynamic step voltage to 4 to prevent brownout
          sysIdTimeout, // Use default timeout (10 s)
          // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("elevatorState", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> master.setControl(sysIdVoltage.withOutput(volts.in(Volts))),
          null,
          this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentElevatorLeftPos = Units.Rotations.of(follower.getPosition().getValueAsDouble());
    currentElevatorRightPos = Units.Rotations.of(master.getPosition().getValueAsDouble());

    // assume follower just maintains master values. follower returns position as
    // negative because it is inverted

    SmartDashboard.putNumber("Elevator/CLO", master.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Output", master.get());
    SmartDashboard.putNumber("Elevator/Inverted", master.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Current", master.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/AdjustedPosition",
        master.getPosition().getValueAsDouble() * positionCoefficient);
    SmartDashboard.putNumber("Elevator/TruePosition", master.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator/LimitDIO", getLimit());
    SmartDashboard.putNumber("Cimb Servo", servo.getPosition());
    SmartDashboard.putNumber("Get Voltage5V", RobotController.getVoltage5V());
    SmartDashboard.putNumber("Get Voltage3V3", RobotController.getVoltage3V3());
    SmartDashboard.putNumber("Get Voltage6V", RobotController.getVoltage6V());

    if (getLimit() && (getPositionNormal() != 0)) {
      master.setPosition(0);
    }
  }
}