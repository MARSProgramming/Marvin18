// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Algae extends SubsystemBase {
    

    TalonSRX algae = new TalonSRX(Constants.CAN_IDS.ALGAE_MECHANISM.ALGAE_MECH_MC);
  /** Creates a new Algae object. */
  public Algae() {
    algae.configFactoryDefault();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("", getBucketBreakReading());
  }

  public void setPercentage(double percent) {
    algae.set(TalonSRXControlMode.PercentOutput, percent);
}

public Command intake() {
    return runEnd(() -> {
        algae.set(TalonSRXControlMode.PercentOutput, DynamicConstants.Algae.intakePercent);

    },
    () -> {
        algae.set(TalonSRXControlMode.PercentOutput, 0); 
    });
}

public Command outtake() {
  return runEnd(() -> {
      algae.set(TalonSRXControlMode.PercentOutput, DynamicConstants.Algae.outtakePercent);

  },
  () -> {
      algae.set(TalonSRXControlMode.PercentOutput, 0);
  });
}
}

