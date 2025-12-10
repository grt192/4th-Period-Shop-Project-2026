// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
  // Kraken motors
  private final TalonFX leftKraken = new TalonFX(PivotConstants.PIVOT_MOTOR_LEFT_ID, "can");
  private final TalonFX rightKraken = new TalonFX(PivotConstants.PIVOT_MOTOR_RIGHT_ID, "can");

  // Encoder
  private final CANcoder canCoder = new CANcoder(PivotConstants.ENCODER_ID);

  // Duty cycle (percent output) control
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

  //Position based voltage control with PID
  private final PositionVoltage positionControl = new PositionVoltage(0);

  public PivotSubsystem() {
    configMotors();
    configEncoder();
  }

  /**
   * Configures both pivot motors with PID, brake mode, & follower setup.
   * The left motor is the leader, and the right motor follows AFTER being inversed because they are 
   * directly positioned opposite to each other.
   */
  private void configMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID Config for position control
    config.Slot0.kP = PivotConstants.PIVOT_P;
    config.Slot0.kI = PivotConstants.PIVOT_I;
    config.Slot0.kD = PivotConstants.PIVOT_D;
    config.Slot0.kV = PivotConstants.PIVOT_F; // Feedforward gain (accounts for forces)

    // Voltage and current limits
    config.Voltage.PeakForwardVoltage = PivotConstants.PIVOT_MAX_VOLTAGE;
    config.Voltage.PeakReverseVoltage = -PivotConstants.PIVOT_MAX_VOLTAGE;
    config.CurrentLimits.StatorCurrentLimit = PivotConstants.PIVOT_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = PivotConstants.PIVOT_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Break mode holds position when stopped
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Config left motor with counter-clockwise positive
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftKraken.getConfigurator().apply(config);

    // Configure right motor opposite of left
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightKraken.getConfigurator().apply(config);

    // Right motor follows the left motor
    rightKraken.setControl(new Follower(PivotConstants.PIVOT_MOTOR_LEFT_ID, true));
  }

  //  Config encoder - boots at 0 degrees
  private void configEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    canCoder.getConfigurator().apply(config);
    // Initialize encoder to 0 on boot
    canCoder.setPosition(0.0);
  }

   /**
   * Converts encoder rotations to degrees by multiplying by 360
   * @return Current angle in degrees
   */
  public double getAngleDegrees() {
    return canCoder.getPosition().getValueAsDouble() * 360.0;
  }


  /**
   * @return Absolute encoder position in rotations (0.0 to 1.0)
   */
  public double getAbsolutePosition() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }


  public void setAngle(double angleDegrees) {
    angleDegrees = Math.max(PivotConstants.MIN_ANGLE,
                           Math.min(PivotConstants.MAX_ANGLE, angleDegrees));

    // Encoder is on the pivot point (not affected by gear ratio)
    // Motor must rotate more due to gear ratio (motor_rotations = encoder_rotations * gear_ratio)
    double encoderRotations = angleDegrees / 360.0;
    double motorRotations = encoderRotations * PivotConstants.GEAR_RATIO;

    leftKraken.setControl(positionControl.withPosition(motorRotations));
  }

 /**
   * Manually controls the pivot at a specific speed
   * Implements soft limits based on encoder: prevents movement beyond 0° to 90° range
   *
   * @param speed Desired speed from -1.0 to 1.0
   */
  public void setManualSpeed(double speed) {
    double currentAngle = getAngleDegrees();

    // Soft limits: stop if encoder exceeds angle range (0° to 90°)
    if (currentAngle >= PivotConstants.MAX_ANGLE && speed > 0) {
      speed = 0;
    }
    if (currentAngle <= PivotConstants.MIN_ANGLE && speed < 0) {
      speed = 0;
    }
    // Command duty cycle to leader motor (leftKraken)
    leftKraken.setControl(dutyCycleControl.withOutput(speed));
  }

   // Set motor output to zero & stop all movement
  public void stop() {
    leftKraken.setControl(dutyCycleControl.withOutput(0));
  }

  //  Resets the encoder position to zero.
  public void zeroEncoder() {
    canCoder.setPosition(0.0);
  }
}