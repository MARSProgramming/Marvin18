// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;

public class Agitator extends SubsystemBase {
  private AnalogInput agitatorIR;

  TalonSRX agitator;
  /** Creates a new Agitator. */
  public Agitator() {
        agitatorIR = new AnalogInput(2);
    agitator = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.AGITATOR);

    agitator.configFactoryDefault();
    agitatorIR.setAverageBits(4);
    agitatorIR.setOversampleBits(4);
  }

  @Override
  public void periodic() {
        SmartDashboard.putNumber("CoralIR", getAgitatorIRreading());
    // This method will be called once per scheduler run
  }

  public double getAgitatorIRreading(){
    return agitatorIR.getAverageVoltage();
 }

   public void setDutyCycle(double dc) {
    agitator.set(TalonSRXControlMode.PercentOutput, dc);
}

public Command runAgitatorWhenReading(double voltage) {
    return runEnd(() -> {
        if(getAgitatorIRreading() < DynamicConstants.IRThresholds.coralIRthreshold){
            agitator.set(TalonSRXControlMode.PercentOutput, voltage);
        }
        else{
            new WaitCommand(2.0);
            agitator.set(TalonSRXControlMode.PercentOutput, 0);
        }
    },
    () -> {
        agitator.set(TalonSRXControlMode.PercentOutput, 0);
    });
}

public Command runCoralAgitator(double percentOut) {
  return runEnd(() -> {
      agitator.set(TalonSRXControlMode.PercentOutput, percentOut);
  },
  () -> {
      agitator.set(TalonSRXControlMode.PercentOutput, 0);
  });
}
}
