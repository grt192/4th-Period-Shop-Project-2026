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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
  // Kraken motors
  private final TalonFX leftKraken = new TalonFX(PivotConstants.PIVOT_MOTOR_LEFT_ID);
  private final TalonFX rightKraken = new TalonFX(PivotConstants.PIVOT_MOTOR_RIGHT_ID);

  // Limit Switches
  private final DigitalInput topLimitSwitch = new DigitalInput(PivotConstants.MAX_LIMIT_SWITCH_ID);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(PivotConstants.MIN_LIMIT_SWITCH_ID);

  // Encoder
  private final CANcoder canCoder = new CANcoder(PivotConstants.ENCODER_ID);
  
  // Duty cycle (percent output) control
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

  //Position based voltage control with PID 
  private final PositionVoltage positionControl = new PositionVoltage(0);

  // Track previous limit switch states
  private boolean previousBottomLimitState = false;
  private boolean previousTopLimitState = false;

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

  //  Config encoder (default settings)
  private void configEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    canCoder.getConfigurator().apply(config);
  }

   /**
   * Gets the current pivot angle in degrees
   * & converts from encoder rotations to degrees by dividing by the gear ratio
   * 
   * @return Current angle in degrees
   */
  public double getAngleDegrees() {
    return canCoder.getPosition().getValueAsDouble() * 360.0 / PivotConstants.GEAR_RATIO;
  }


  /**
   * @return Absolute encoder position in rotations (0.0 to 1.0)
   */
  public double getAbsolutePosition() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean isAtTopLimit() {
    // Prevent moving up if at top limit
    return !topLimitSwitch.get();
  }

  public boolean isAtBottomLimit() {
    // Prevent moving down if at bottom limit
    return !bottomLimitSwitch.get(); 
  }

  public void setAngle(double angleDegrees) {
    angleDegrees = Math.max(PivotConstants.MIN_ANGLE,
                           Math.min(PivotConstants.MAX_ANGLE, angleDegrees));

    double motorRotations = (angleDegrees * PivotConstants.GEAR_RATIO) / 360.0;

    leftKraken.setControl(positionControl.withPosition(motorRotations));
  }

 /**
   * Manually controls the pivot at a specific speed
   * Prevents pivot from moving past top & bottom limits
   * 
   * @param speed Desired speed from -1.0 to 1.0 
   */
  public void setManualSpeed(double speed) {
    if (isAtTopLimit() && speed > 0) {
      speed = 0;
    }
    if (isAtBottomLimit() && speed < 0) {
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

  // Set encoder position to the max angle
  public void setEncoderToMax() {
    double maxRotations = (PivotConstants.MAX_ANGLE * PivotConstants.GEAR_RATIO) / 360.0;
    canCoder.setPosition(maxRotations);
  }

  @Override
  public void periodic() {
    // Gets current state of top & bottom limit switches
    boolean topLimit = isAtTopLimit();
    boolean bottomLimit = isAtBottomLimit();

    if (bottomLimit && !previousBottomLimitState) {
      // Zero the encoder if bottom limit switch is just pressed
      zeroEncoder();
    }
    previousBottomLimitState = bottomLimit;

    if (topLimit && !previousTopLimitState) {
      // Set encoder to max if top limit switch is just pressed
      setEncoderToMax();
    }
    previousTopLimitState = topLimit;
  }
}