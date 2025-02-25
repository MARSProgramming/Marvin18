// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralArm;

public class Hopper extends SubsystemBase {
    private AnalogInput bucketIR;

    TalonSRX coralIntake;
  /** Creates a new Coral_Hopper. */
  public Hopper() {
    bucketIR = new AnalogInput(1);
    coralIntake = new TalonSRX(Constants.CAN_IDS.CORAL_MECHANISM.CORAL_INTAKE);

    coralIntake.configFactoryDefault();
    bucketIR.setOversampleBits(4);
    bucketIR.setAverageBits(4);


  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("BucketIR", getBucketIRReading());
  //  runAgitatorWhenReading(1);
    //SmartDashboard.putBoolean("", getBucketIRReading());
      // assume follower just maintains master values. follower returns position as negative because it is inverted
      SmartDashboard.putNumber("Bucket/VoltageOut", coralIntake.getMotorOutputVoltage());
  }

  public void setDutyCycle(double dc) {
    coralIntake.set(TalonSRXControlMode.PercentOutput, dc);
}


 public double getBucketIRReading(){
    return bucketIR.getAverageVoltage();
 }
 

private int integ = 0;
public boolean hasCoral() {
   if (getBucketIRReading() > DynamicConstants.IRThresholds.bucketIRthreshold) {
    integ++;
   } else {
    integ = 0;
   }
   return integ >=5;
}


public Command runIntake(double percentOut) {
    return runEnd(() -> {
     coralIntake.set(TalonSRXControlMode.PercentOutput, percentOut);
    },
    () -> {
        coralIntake.set(TalonSRXControlMode.PercentOutput, 0);
    });
}

public Command runIntakeSafe(double percentOut, CoralArm coralArm) {
    return runEnd(() -> {
        if(coralArm.getPosition() > 0.1)
            coralIntake.set(TalonSRXControlMode.PercentOutput, percentOut);
        else return;
    },
    () -> {
        coralIntake.set(TalonSRXControlMode.PercentOutput, 0);
    });
}


public Command runIntakeUntilIR(double percentOut) {
    return runEnd(() -> {
     coralIntake.set(TalonSRXControlMode.PercentOutput, percentOut);
    },
    () -> {
        coralIntake.set(TalonSRXControlMode.PercentOutput, 0);
    }).until(() -> hasCoral());
}
}
