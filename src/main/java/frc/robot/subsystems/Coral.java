// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  TalonSRX coral = new TalonSRX(12);
  /** Creates a new Intake. */
  public Coral() {
    coral.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public Command runIntake(double percentOut) {
    return runEnd(() -> {
     coral.set(TalonSRXControlMode.PercentOutput, percentOut);
    },
    () -> {
        coral.set(TalonSRXControlMode.PercentOutput, 0);
    });
}
}
