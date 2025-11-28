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

  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  private final PositionVoltage positionControl = new PositionVoltage(0);

  // Track previous limit switch states
  private boolean previousBottomLimitState = false;
  private boolean previousTopLimitState = false;

  public PivotSubsystem() {
    configMotors();
    configEncoder();
  }

  private void configMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID Config
    config.Slot0.kP = PivotConstants.PIVOT_P;
    config.Slot0.kI = PivotConstants.PIVOT_I;
    config.Slot0.kD = PivotConstants.PIVOT_D;
    config.Slot0.kV = PivotConstants.PIVOT_F;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftKraken.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightKraken.getConfigurator().apply(config);
    rightKraken.setControl(new Follower(PivotConstants.PIVOT_MOTOR_LEFT_ID, true));
  }

  private void configEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    canCoder.getConfigurator().apply(config);
  }

  public double getAngleDegrees() {
    return canCoder.getPosition().getValueAsDouble() * 360.0 / PivotConstants.GEAR_RATIO;
  }


  public double getAbsolutePosition() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean isAtTopLimit() {
    return !topLimitSwitch.get();
  }

  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get(); 
  }

  public void setAngle(double angleDegrees) {
    angleDegrees = Math.max(PivotConstants.MIN_ANGLE,
                           Math.min(PivotConstants.MAX_ANGLE, angleDegrees));

    double motorRotations = (angleDegrees * PivotConstants.GEAR_RATIO) / 360.0;

    leftKraken.setControl(positionControl.withPosition(motorRotations));
  }

  public void setManualSpeed(double speed) {
    if (isAtTopLimit() && speed > 0) {
      speed = 0;
    }
    if (isAtBottomLimit() && speed < 0) {
      speed = 0;
    }

    leftKraken.setControl(dutyCycleControl.withOutput(speed));
  }

  public void stop() {
    leftKraken.setControl(dutyCycleControl.withOutput(0));
  }

  public void zeroEncoder() {
    canCoder.setPosition(0.0);
  }

  public void setEncoderToMax() {
    double maxRotations = (PivotConstants.MAX_ANGLE * PivotConstants.GEAR_RATIO) / 360.0;
    canCoder.setPosition(maxRotations);
  }

  @Override
  public void periodic() {
    boolean topLimit = isAtTopLimit();
    boolean bottomLimit = isAtBottomLimit();

    if (bottomLimit && !previousBottomLimitState) {
      zeroEncoder();
    }
    previousBottomLimitState = bottomLimit;

    if (topLimit && !previousTopLimitState) {
      setEncoderToMax();
    }
    previousTopLimitState = topLimit;
  }
}