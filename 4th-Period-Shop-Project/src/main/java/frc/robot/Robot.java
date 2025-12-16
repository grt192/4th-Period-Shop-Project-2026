// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Map;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // Match timer variables
  private double matchStartTime;
  private static final double MATCH_TIME = 150.0; // 2:30 in seconds
  private final ShuffleboardTab tab = Shuffleboard.getTab("Telemetry");
  private final ShuffleboardTab settingsTab = Shuffleboard.getTab("Settings");

  private final GenericEntry timerEntry = tab.add("Match Timer", "2:30")
      .withWidget(BuiltInWidgets.kTextView)
      .withProperties(Map.of("text color", "#FFFFFF"))
      .withPosition(0, 0)
      .withSize(3, 2)
      .getEntry();

  private final GenericEntry batteryVoltageEntry = tab.add("Battery Voltage", 0.0)
      .withWidget(BuiltInWidgets.kVoltageView)
      .withProperties(Map.of(
          "min", 0.0,
          "max", 13.0,
          "center", 12.0,
          "orientation", "HORIZONTAL",
          "number of tick marks", 5
      ))
      .withPosition(3, 0)
      .withSize(3, 2)
      .getEntry();

  // Tunable settings
  private final GenericEntry servoOpenPosEntry = settingsTab.add("Servo Open Position", Constants.ServoConstants.OPEN_POSITION)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0.0, "max", 180.0))
      .withPosition(0, 0)
      .withSize(3, 1)
      .getEntry();

  private final GenericEntry intakeSpeedEntry = settingsTab.add("Intake Speed", Constants.ServoConstants.INTAKE_SPEED)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0.0, "max", 0.1))
      .withPosition(0, 1)
      .withSize(3, 1)
      .getEntry();

  private final GenericEntry rampRateEntry = settingsTab.add("Ramp Rate (sec)", Constants.DriveConstants.openLoopRampRate.in(edu.wpi.first.units.Units.Seconds))
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0.0, "max", 0.5))
      .withPosition(0, 2)
      .withSize(3, 1)
      .getEntry();

  private final GenericEntry pivotSpeedEntry = settingsTab.add("Pivot Speed", Constants.PivotConstants.PIVOT_MANUAL_SPEED)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0.0, "max", 0.2))
      .withPosition(0, 3)
      .withSize(3, 1)
      .getEntry();

  public Robot() {
    m_robotContainer = new RobotContainer();

    // Setup driver camera
    setupCamera();
  }

  private void setupCamera() {
    try {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(320, 240);
      camera.setFPS(15);

      tab.add("Driver Camera", camera)
          .withWidget(BuiltInWidgets.kCameraStream)
          .withPosition(6, 0)
          .withSize(4, 5);
    } catch (Exception e) {
      System.err.println("Failed to start camera: " + e.getMessage());
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    updateMatchTimer();
    updateBatteryVoltage();
    updateTunableSettings();
  }

  private void updateMatchTimer() {
    if (DriverStation.isEnabled()) {
      double elapsed = DriverStation.getMatchTime();

      if (elapsed >= 0) {
        int minutes = (int) (elapsed / 60);
        int seconds = (int) (elapsed % 60);
        String timeString = String.format("%d:%02d", minutes, seconds);

        // Change color to red when 10 seconds or less
        if (elapsed <= 10.0) {
          timerEntry.setString("<!DOCTYPE html><html><body style='color:red;font-size:24px;'>" + timeString + "</body></html>");
        } else {
          timerEntry.setString(timeString);
        }
      }
    } else {
      timerEntry.setString("2:30");
    }
  }

  private void updateBatteryVoltage() {
    batteryVoltageEntry.setDouble(RobotController.getBatteryVoltage());
  }

  private void updateTunableSettings() {
    // Update servo open position
    m_robotContainer.getServoSubsystem().openPosition = servoOpenPosEntry.getDouble(Constants.ServoConstants.OPEN_POSITION);

    // Update intake speed
    m_robotContainer.getServoSubsystem().intakeSpeed = intakeSpeedEntry.getDouble(Constants.ServoConstants.INTAKE_SPEED);

    // Update ramp rate
    double rampRate = rampRateEntry.getDouble(Constants.DriveConstants.openLoopRampRate.in(edu.wpi.first.units.Units.Seconds));
    m_robotContainer.getDriveSubsystem().setRampRate(rampRate);

    // Update pivot speed
    m_robotContainer.getPivotSubsystem().manualSpeed = pivotSpeedEntry.getDouble(Constants.PivotConstants.PIVOT_MANUAL_SPEED);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

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
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
