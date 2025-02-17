
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.RobotContainer;

public class CoralArm extends SubsystemBase {
    private TalonFX arm;
    private TalonSRX bucketMotor;
    private double positionCoefficient = 1/48;
    PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    VoltageOut voltageRequest = new VoltageOut(0);
    MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private DigitalInput armlimit;


    public CoralArm() {
        arm = new TalonFX(Constants.CAN_IDS.CORAL_MECHANISM.CORAL_BUCKET_ROTATE);
        bucketMotor = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.AGITATOR);
        armlimit = new DigitalInput(Constants.DIO_IDS.CORAL_ARM_LIMIT);

        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        bucketMotor.configFactoryDefault();

        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ArmSetpointConfigs.ARM_FORWARD_LIMIT;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ArmSetpointConfigs.ARM_REVERSE_LIMIT;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // TESTING ONLY
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TESTING ONLY
        
     //   masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.Feedback.SensorToMechanismRatio = 48;
        armConfig.CurrentLimits.SupplyCurrentLimit = 20;
        armConfig.CurrentLimits.StatorCurrentLimit =  60;
        armConfig.Voltage.PeakForwardVoltage = 16;
        armConfig.Voltage.PeakReverseVoltage = -16;
        armConfig.MotionMagic.MotionMagicCruiseVelocity = 0.2;
        armConfig.MotionMagic.MotionMagicAcceleration = 3;
        armConfig.Slot0.kP = 110;
        armConfig.Slot0.kI = 12;
        armConfig.Slot0.kD = 15;
        armConfig.Slot0.kS = 0;
        armConfig.Slot0.kG = 0;
        arm.getConfigurator().apply(armConfig);
        arm.setPosition(0);
    }

    public Command setMotionMagicPosition(DoubleSupplier position) {
        return runEnd(() -> {
            arm.setControl(motionMagicRequest.withPosition(position.getAsDouble()));
        }, () -> {
            arm.set(0);
        });
    }

    public Command ArmPosVoltage(double position) {
        return runEnd(() -> {
            arm.setControl(positionVoltageRequest.withPosition(position));
        }, () -> {
            arm.set(0);
        });
    }

    public double getPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            arm.setControl(voltageRequest.withOutput(voltage));
        }, () -> {
            arm.set(0);
        });
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // assume follower just maintains master values. follower returns position as negative because it is inverted

    SmartDashboard.putNumber("Arm/CLO", arm.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Output", arm.get());
    SmartDashboard.putNumber("Arm/Inverted", arm.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Current", arm.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Arm/AdjustedPosition", arm.getPosition().getValueAsDouble() * positionCoefficient);
    SmartDashboard.putNumber("Arm/TruePosition", arm.getPosition().getValueAsDouble());

    SmartDashboard.putBoolean("Arm/DIO", !armlimit.get());

  }

}
