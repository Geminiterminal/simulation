// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
    private DoubleLogEntry batteryVoltageLog;

  @Override
  public void robotInit() {
    // Start WPILib data logging before constructing RobotContainer so SysIdRoutineLog
    // entries and dashboard choosers attach to the active .wpilog immediately.
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();

    batteryVoltageLog = new DoubleLogEntry(
        DataLogManager.getLog(),
        "/Battery/Voltage"
    );
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
        batteryVoltageLog.append(RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Stop autonomous when teleop starts
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // SAFETY: Always retract climber when teleop begins.
    // This ensures the robot is never in a raised state
    // when the driver takes control after autonomous.
    //R Climber.getInstance().retract();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {
    // Runs after CommandScheduler.run() (so after every subsystem's
    // simulationPeriodic): aggregate current draws into the battery model.
    m_robotContainer.simulationPeriodic();
  }
}