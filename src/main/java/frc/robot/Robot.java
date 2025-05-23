// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.DynamicConstants.Drive;
import frc.robot.util.Elastic;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Alliance m_currentAlliance;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_currentAlliance = DriverStation.getAlliance().get();

  }

  @Override
  public void robotInit() {
   // DynamicConstants.periodic();
   // w
   m_robotContainer.led.setRainbowAnimationCommand().withTimeout(5);
    //m_robotContainer.m_elevator.setServoCommand(0).schedule();
    
    SmartDashboard.putData("Update Constants", m_robotContainer.configureBindingsCommand());
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(false);
    
    // Elastic.selectTab("Autonomous"); // Removed as the class cannot be resolved
  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Robot Voltage", RobotController.getBatteryVoltage());
    
  }

  @Override
  public void disabledInit() {
   // m_robotContainer.leds.defaultCommand().schedule();
  }


  @Override
  public void disabledPeriodic() {
    m_robotContainer.led.setRainbowAnimationCommand();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Elastic.selectTab("Teleoperated"); // Removed as the class cannot be resolved
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.configureTestBindings();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
