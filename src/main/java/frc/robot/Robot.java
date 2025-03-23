// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.DynamicConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

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
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  //  DynamicConstants.periodic();

   // var feederEst = m_robotContainer.feeder_vision.getEstimatedGlobalPose();
  // feederEst.ifPresent(
  //     estF -> {
          // Change our trust in the measurement based on the tags we can see
   //     var estStdDevsF = m_robotContainer.feeder_vision.getEstimationStdDevs();

     //  m_robotContainer.drivetrain.addVisionMeasurement(
     //      estF.estimatedPose.toPose2d(), estF.timestampSeconds, estStdDevsF);
     // });

    var reefEst = m_robotContainer.reef_vision.getEstimatedGlobalPose();
    reefEst.ifPresent(
        estR -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevsR = m_robotContainer.reef_vision.getEstimationStdDevs();

          m_robotContainer.drivetrain.addVisionMeasurement(
              estR.estimatedPose.toPose2d(), estR.timestampSeconds, estStdDevsR);
              Logger.recordOutput(
                "Reef Pose", estR.estimatedPose);
        });

    
  }

  @Override
  public void disabledInit() {
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
