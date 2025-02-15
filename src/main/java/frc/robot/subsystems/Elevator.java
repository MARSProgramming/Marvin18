package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;

public class Elevator extends SubsystemBase {
    private TalonFX master; // right SIDE MOTOR
    private TalonFX follower; // left SIDE MOTOR
    private double positionCoefficient = 1.0/12.0;
    private boolean softLimitEnabled;
    private DigitalInput leftLimitSwitch;
    private DigitalInput rightLimitSwitch;
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

    public Elevator() {

        // Motors
        follower = new TalonFX(Constants.CAN_IDS.ELEVATOR.ELEVATOR_FOLLOWER, "CAN-2"); // left SIDE MOTOR
        master = new TalonFX(Constants.CAN_IDS.ELEVATOR.ELEVATOR_MASTER, "CAN-2"); // RIGHT SIDE MOTOR
        leftLimitSwitch = new DigitalInput(4);
        rightLimitSwitch = new DigitalInput(0);
        // Elevator Speed/Target Configs 
        mostRecentTarget = 0; // configure units before testing - get in terms of encoder positions
        voltageRequest = new VoltageOut(0);
        motionRequest = new MotionMagicVoltage(0);
        positionVoltageRequest = new PositionVoltage(0);
        currentElevatorLeftPos = Rotations.of(0); // Follower
        currentElevatorRightPos = Rotations.of(0); // Master
        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ElevatorSetpointConfigs.ELEVATOR_FORWARD_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ElevatorSetpointConfigs.ELEVATOR_REVERSE_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // TESTING ONLY
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TESTING ONLY
      //  masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimit = 20;
        masterConfig.CurrentLimits.StatorCurrentLimit =  60;
        masterConfig.Voltage.PeakForwardVoltage = 4;
        masterConfig.Voltage.PeakReverseVoltage = -4;

        masterConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        masterConfig.Slot0 = new Slot0Configs().withKP(.8).withKI(.3).withKD(0);
        master.getConfigurator().apply(masterConfig);
        follower.getConfigurator().apply(masterConfig);
        //master.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        follower.setControl(new Follower(master.getDeviceID(), true));
      //  leftLimit = new DigitalInput(0);
      //rightLimit = new DigitalInput(0);
        
        master.setPosition(0.0); // resets elevator encoders to 0
        follower.setPosition(0.0);
    }

    public void setMotionMagicPosition(double rotations) {
      // must pass in a value that is converted to positioncoefficient modified rotations
      master.setControl(motionRequest.withPosition(rotations)); // Should be a MotionMagicVoltage input, TUNED FROM A CONSTRUCTOR. See SuperNURDs code.
      follower.setControl(new Follower(master.getDeviceID(), true));
      mostRecentTarget = rotations * positionCoefficient;
    }

    public double getLastDesiredPosition() {
      return mostRecentTarget;
    }

    public double getAdjustedRotations() {
    return master.getPosition().getValueAsDouble() * positionCoefficient;
  }

  public boolean getLeftLimit() {
    return !leftLimitSwitch.get();
  }

  public boolean getRightLimit() {
    return !rightLimitSwitch.get();
  }

  public boolean isAtSetpoint() {
    double offset = getAdjustedRotations() - getLastDesiredPosition();
    if (offset > 0) {
      return ((offset - deadzoneDist) > 0);
      }

    if (offset < 0) {
      return ((offset + deadzoneDist) > 0);
    }
    else {
      return false;
    }
  }

  public AngularVelocity getSpinVelocity() {
    return master.getRotorVelocity().getValue();
  }

  public Command elevatorPositionVoltage(double position) {
    return runEnd(() -> {
      master.setControl(positionVoltageRequest.withPosition(0));
    }, () -> {
      master.set(0);
    });
  }

  public boolean isStopped() {
    return getSpinVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setNeutral() {
    master.setControl(new NeutralOut());
    follower.setControl(new NeutralOut());
  }

  public Command runVoltage(double voltage) {
        return runEnd(() -> {
            master.setControl(voltageRequest.withOutput(voltage));
        }, () -> {
            master.set(0);
        });
    }

    public Command runVoltageRequest(double voltage) {
      return runEnd(() -> {
          master.setControl(voltageRequest.withOutput(0));
      }, () -> {
        master.set(0);
      });
  }

    public Command resetPositions(double position) {
      return runEnd(() -> {
          master.setControl(new DutyCycleOut(0));
          master.setPosition(position);
      }, () -> 
      master.setPosition(position));
  }

    public Command zeroElevatorWithLimit() {
        return runEnd(() -> {
          setMotionMagicPosition(0);
        }, () -> {
          if (leftLimitSwitch.get() || rightLimitSwitch.get()) {
            master.set(0);
          }
        }).until(() -> leftLimitSwitch.get() || rightLimitSwitch.get());
    }

    public boolean isPosNearZero() {
      if (Math.abs(getAdjustedRotations()) < Constants.ElevatorSetpointConfigs.ELEVATOR_ROTATIONS_DEADZONE) {
        return true;
      } else {
        return false;
      }
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
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentElevatorLeftPos = Units.Rotations.of(follower.getPosition().getValueAsDouble());
    currentElevatorRightPos = Units.Rotations.of(master.getPosition().getValueAsDouble());

    // assume follower just maintains master values. follower returns position as negative because it is inverted

    SmartDashboard.putNumber("Elevator/CLO", master.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Output", master.get());
    SmartDashboard.putNumber("Elevator/Inverted", master.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Current", master.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/AdjustedPosition", master.getPosition().getValueAsDouble() * positionCoefficient);
    SmartDashboard.putNumber("Elevator/TruePosition", master.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator/LeftLimitDIO", getLeftLimit());
    SmartDashboard.putBoolean("Elevator/RightLimitDIO", getRightLimit());

  }
}
