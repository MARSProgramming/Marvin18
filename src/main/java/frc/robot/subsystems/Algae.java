// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.Constants;
// import frc.robot.constants.DynamicConstants;
// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Algae extends SubsystemBase {
    

    SparkMax algae = new SparkMax(Constants.CAN_IDS.ALGAE_MECHANISM.ALGAE_MECH_MC, MotorType.kBrushless);
  /** Creates a new Algae object. */
  public Algae() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    config.inverted(true);
    algae.configure(config, null, null);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("", getBucketBreakReading());
    SmartDashboard.putNumber("algae Current", algae.getOutputCurrent());
    SmartDashboard.putNumber("algae Voltage", algae.getBusVoltage());
    SmartDashboard.putNumber("algae temps", algae.getMotorTemperature());

  }

  public void setPercentage(double percent) {
    algae.set(percent);
}

public Command intake() {
    return runEnd(() -> {
        //setPercentage(DynamicConstants.Algae.intakePercent);
        algae.set(1);
    },
    () -> {
      setPercentage(0.5);
 
    });
}

public Command intakeWithStop() {
  return runEnd(() -> {
      //setPercentage(DynamicConstants.Algae.intakePercent);
      algae.set(1);
  },
  () -> {
    algae.set(0);

  });
}

public Command outtake() {
  return runEnd(() -> {
  //  setPercentage(DynamicConstants.Algae.outtakePercent);
  algae.set(-1);
  },
  () -> {
    setPercentage(0);
  });
}
}

