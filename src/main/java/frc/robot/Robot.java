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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDSegment;

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
   // DynamicConstants.init();
   // DynamicConstants.periodic();
    PathfindingCommand.warmupCommand().schedule();
    m_robotContainer.m_elevator.setServoCommand(0).schedule();

    SmartDashboard.putData("Update Constants", m_robotContainer.configureBindingsCommand());
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);

    LEDSegment.MainStrip.fullClear();
    LEDSegment.MainStrip.setColor(LED.yellow);
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    LEDSegment.MainStrip.fullClear();
    LEDSegment.MainStrip.setColor(LED.purple);

  }


  @Override
  public void disabledPeriodic() {
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
    LEDSegment.MainStrip.fullClear();
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
    LEDSegment.MainStrip.fullClear();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    LEDSegment.MainStrip.fullClear();
    LEDSegment.MainStrip.setColor(LED.red);
  }

  @Override
  public void teleopPeriodic() {
    //LEDSegment.MainStrip.setFadeAnimation(LED.red, 0.5);

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
