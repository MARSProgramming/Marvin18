package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ElevatorTesting extends SubsystemBase {

    private TalonFX master; // RIGHT SIDE MOTOR
    private TalonFX follower; // LEFT SIDE MOTOR
    private double positionCoefficient = 1.0/12.0;
    private boolean softLimitEnabled;
    private DigitalInput leftLimit;
    private DigitalInput rightLimit;
    private Distance mostRecentTarget;

    private Servo ratchet; // RIGHT SIDE RATCHET

    public boolean elevatorZeroInProgress = false;
    public boolean elevatorZeroed = false;
    public Distance deadzoneDist = Units.Inches.of(1);
    VoltageOut voltageRequest = new VoltageOut(0);

    public Distance currentElevatorRightPos;
    public Distance currentElevatorLeftPos;

    public ElevatorTesting() {

        // CLimb Ratchet
        ratchet = new Servo(9);

        // Motors
        follower = new TalonFX(Constants.CAN_IDS.ELEVATOR.ELEVATOR_FOLLOWER, "CAN-2"); // left SIDE MOTOR
        master = new TalonFX(Constants.CAN_IDS.ELEVATOR.ELEVATOR_MASTER, "CAN-2"); // RIGHT SIDE MOTOR

        // Elevator Speed/Target Configs 
        mostRecentTarget = Units.Inches.of(0); // configure units before testing - get in terms of encoder positions
        VoltageOut voltageRequest = new VoltageOut(0);
        
        currentElevatorLeftPos = Inches.of(0); // Follower
        currentElevatorRightPos = Inches.of(0); // Master

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimit = 20;
        masterConfig.CurrentLimits.StatorCurrentLimit =  60;
        masterConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);



        master.getConfigurator().apply(masterConfig);
        follower.getConfigurator().apply(masterConfig);
        follower.setControl(new Follower(master.getDeviceID(), true));
      //  leftLimit = new DigitalInput(0);
      //rightLimit = new DigitalInput(0);
        
        master.setPosition(0); // resets elevator encoders to 0
    }



    public Distance getElevatorPosition() {
    return Units.Inches.of(master.getPosition().getValueAsDouble());
  }

  public boolean isAtSetpoint() {
    return (getElevatorPosition()
        .compareTo(getLastDesiredPosition().minus(deadzoneDist)) > 0) &&
        getElevatorPosition().compareTo(getLastDesiredPosition().plus(deadzoneDist)) < 0;
  }

  public AngularVelocity getSpinVelocity() {
    return master.getRotorVelocity().getValue();
  }

  public Distance getLastDesiredPosition() {
    return mostRecentTarget;
  }

  public boolean isStopped() {
    return getSpinVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setPosition(Distance height) {
    master.setControl(new MotionMagicVoltage(height.in(Units.Inches))); // Should be a MotionMagicVoltage input, TUNED FROM A CONSTRUCTOR. See SuperNURDs code.
    follower.setControl(new Follower(master.getDeviceID(), true));
    mostRecentTarget = height;
  }

  public void setNeutral() {
    master.setControl(new NeutralOut());
    follower.setControl(new NeutralOut());
  }

  public void setVoltage(double voltage) {
    master.setControl(voltageRequest.withOutput(voltage));
    follower.setControl(new Follower(master.getDeviceID(), true));
  }

  public Command runVoltage(double voltage) {
        return runEnd(() -> {
            master.setVoltage(voltage);
            follower.setVoltage(voltage);
        }, () -> {
            master.set(0);
            follower.set(0);
        });
    }

  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    master.getConfigurator().apply(masterConfig);
    follower.getConfigurator().apply(masterConfig);
  }

  public void resetSensorPosition(Distance setpoint) {
    master.setPosition(setpoint.in(Inches));
    follower.setPosition(setpoint.in(Inches));
  }

  public Command lockClimber(double degrees) {
        return runOnce(() -> {ratchet.setAngle(degrees);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentElevatorLeftPos = Units.Inches.of(follower.getPosition().getValueAsDouble());
    currentElevatorRightPos = Units.Inches.of(master.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Elevator/Left/CLO", follower.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Output", follower.get());
    SmartDashboard.putNumber("Elevator/Left/Inverted", follower.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Current", follower.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Elevator/Right/CLO", master.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Output", master.get());
    SmartDashboard.putNumber("Elevator/Right/Inverted", master.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Current", master.getSupplyCurrent().getValueAsDouble());
  }
}
