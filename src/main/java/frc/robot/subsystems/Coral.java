// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import edu.wpi.first.wpilibj.AnalogInput;

public class Coral extends SubsystemBase {
  TalonSRX coral = new TalonSRX(12);
  private AnalogInput IR = new AnalogInput(1);
  /** Creates a new Intake. */
  public Coral() {
    coral.configFactoryDefault();
    IR.setOversampleBits(4);
    IR.setAverageBits(4);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IR", getIRReading());
    SmartDashboard.putBoolean("Has Coral", hasCoral());
    SmartDashboard.putNumber("Coral Output %", coral.getMotorOutputPercent());
    SmartDashboard.putNumber("Coral Stator Current", coral.getStatorCurrent());
    // This method will be called once per scheduler run
  }

  public double getIRReading(){
    return IR.getAverageVoltage();
 }
 
 private int integ = 0;
 public boolean hasCoral() {
   if (getIRReading() > DynamicConstants.IRThresholds.IRthreshold) {
    integ++;
   } else {
    integ = 0;
   }
   return integ >=5;
}

public Command coralCheck() {
  return runEnd(() -> {
    coral.set(TalonSRXControlMode.PercentOutput, -0.25);
  }, () -> {
    coral.set(TalonSRXControlMode.PercentOutput, -0.25);
  }).until(() -> hasCoral());
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
