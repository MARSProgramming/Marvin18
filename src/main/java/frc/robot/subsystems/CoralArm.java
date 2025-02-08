package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class CoralArm extends SubsystemBase {
    private TalonFX arm;
    private TalonSRX bucketMotor;
    private double positionCoefficient = 1/12;


    public CoralArm() {
        arm = new TalonFX(Constants.CAN_IDS.CORAL_MECHANISM.CORAL_BUCKET_ROTATE);
        bucketMotor = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.AGITATOR);

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
        armConfig.CurrentLimits.SupplyCurrentLimit = 20;
        armConfig.CurrentLimits.StatorCurrentLimit =  60;

        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);

        arm.getConfigurator().apply(armConfig);
    }



    public Command runArmVoltage(double voltage) {
        return runEnd(() -> {
            arm.setVoltage(voltage);
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
  }

}
