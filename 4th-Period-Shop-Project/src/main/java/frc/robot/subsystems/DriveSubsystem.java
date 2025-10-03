// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Left side motors
  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(DriveConstants.LEFT_FRONT_MOTOR_ID);
  private final WPI_TalonSRX leftBack = new WPI_TalonSRX(DriveConstants.LEFT_BACK_MOTOR_ID);

  // Right side motors
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveConstants.RIGHT_FRONT_MOTOR_ID);
  private final WPI_TalonSRX rightBack = new WPI_TalonSRX(DriveConstants.RIGHT_BACK_MOTOR_ID);

  // Differential drive for arcade control
  private final DifferentialDrive drive;

  public DriveSubsystem() {
    // Configure left motors
    configureMotor(leftFront);
    configureMotor(leftBack);
    leftBack.follow(leftFront);

    // Configure right motors
    configureMotor(rightFront);
    configureMotor(rightBack);
    rightBack.follow(rightFront);

    // Set right side inverted (motors face opposite direction)
    rightFront.setInverted(true);
    rightBack.setInverted(true);

    // Create differential drive with front motors as leaders
    drive = new DifferentialDrive(leftFront, rightFront);
    drive.setSafetyEnabled(false);
  }

  /**
   * Configures a motor with PID, current limits, and ramp rates
   */
  private void configureMotor(WPI_TalonSRX motor) {
    // Factory reset
    motor.configFactoryDefault();

    // Configure PID
    motor.config_kP(DriveConstants.PID_SLOT, DriveConstants.kP, DriveConstants.TIMEOUT_MS);
    motor.config_kI(DriveConstants.PID_SLOT, DriveConstants.kI, DriveConstants.TIMEOUT_MS);
    motor.config_kD(DriveConstants.PID_SLOT, DriveConstants.kD, DriveConstants.TIMEOUT_MS);
    motor.config_kF(DriveConstants.PID_SLOT, DriveConstants.kF, DriveConstants.TIMEOUT_MS);

    // Configure current limiting
    motor.configPeakCurrentLimit(DriveConstants.TRIGGER_THRESHOLD_CURRENT, DriveConstants.TIMEOUT_MS);
    motor.configPeakCurrentDuration((int)(DriveConstants.TRIGGER_THRESHOLD_TIME * 1000), DriveConstants.TIMEOUT_MS);
    motor.configContinuousCurrentLimit(DriveConstants.CURRENT_LIMIT, DriveConstants.TIMEOUT_MS);
    motor.enableCurrentLimit(true);

    // Configure ramp rates
    motor.configOpenloopRamp(DriveConstants.OPEN_LOOP_RAMP, DriveConstants.TIMEOUT_MS);
    motor.configClosedloopRamp(DriveConstants.CLOSED_LOOP_RAMP, DriveConstants.TIMEOUT_MS);

    // Set neutral mode to brake
    motor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Arcade drive method
   * @param forward Forward/backward speed (-1.0 to 1.0)
   * @param rotation Rotation speed (-1.0 to 1.0)
   */
  public void arcadeDrive(double forward, double rotation) {
    drive.arcadeDrive(forward, rotation);
  }

  /**
   * Stops all drive motors
   */
  public void stop() {
    drive.stopMotor();
  }

  /**
   * Gets the left encoder position
   */
  public double getLeftEncoderPosition() {
    return leftFront.getSelectedSensorPosition();
  }

  /**
   * Gets the right encoder position
   */
  public double getRightEncoderPosition() {
    return rightFront.getSelectedSensorPosition();
  }

  /**
   * Gets the left encoder velocity
   */
  public double getLeftEncoderVelocity() {
    return leftFront.getSelectedSensorVelocity();
  }

  /**
   * Gets the right encoder velocity
   */
  public double getRightEncoderVelocity() {
    return rightFront.getSelectedSensorVelocity();
  }

  /**
   * Resets both encoders to zero
   */
  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with telemetry
    SmartDashboard.putNumber("Drive/Left Encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("Drive/Right Encoder", getRightEncoderPosition());
    SmartDashboard.putNumber("Drive/Left Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Drive/Right Velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Drive/Left Current", leftFront.getStatorCurrent());
    SmartDashboard.putNumber("Drive/Right Current", rightFront.getStatorCurrent());
  }
}
