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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;


public class PivotSubsystem extends SubsystemBase {
  // Motors (Kraken X60)
  private final TalonFX leftKraken = new TalonFX(PivotConstants.LEFT_KRAKEN_ID);
  private final TalonFX rightKraken = new TalonFX(PivotConstants.RIGHT_KRAKEN_ID);

  // Encoder (WCP ThroughBore Encoder Powered by CANcoder)
  private final CANcoder canCoder = new CANcoder(PivotConstants.THROUGHBORE_ENCODER_ID);

  // Limit Switches
  private final DigitalInput topLimitSwitch = new DigitalInput(PivotConstants.TOP_LIMIT_SWITCH_DIO);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(PivotConstants.BOTTOM_LIMIT_SWITCH_DIO);

  // Control requests
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  private final PositionVoltage positionControl = new PositionVoltage(0);

  // Track previous limit switch states 
  private boolean previousBottomLimitState = false;
  private boolean previousTopLimitState = false;

  // NetworkTables for AdvantageScope logging
  private final NetworkTable pivotTable;
  private final DoublePublisher anglePublisher;
  private final DoublePublisher absolutePositionPublisher;
  private final DoublePublisher leftMotorCurrentPublisher;
  private final DoublePublisher rightMotorCurrentPublisher;
  private final DoublePublisher leftMotorTempPublisher;
  private final DoublePublisher rightMotorTempPublisher;
  private final DoublePublisher leftMotorVoltagePublisher;
  private final DoublePublisher rightMotorVoltagePublisher;
  private final DoublePublisher targetAnglePublisher;
  private final StringPublisher statusPublisher;

  public PivotSubsystem() {
    configureMotors();
    configureEncoder();

    // Initialize NetworkTables publishers for logging
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    pivotTable = inst.getTable("Pivot");

    anglePublisher = pivotTable.getDoubleTopic("AngleDegrees").publish();
    absolutePositionPublisher = pivotTable.getDoubleTopic("AbsolutePosition").publish();
    leftMotorCurrentPublisher = pivotTable.getDoubleTopic("LeftMotorCurrent").publish();
    rightMotorCurrentPublisher = pivotTable.getDoubleTopic("RightMotorCurrent").publish();
    leftMotorTempPublisher = pivotTable.getDoubleTopic("LeftMotorTemp").publish();
    rightMotorTempPublisher = pivotTable.getDoubleTopic("RightMotorTemp").publish();
    leftMotorVoltagePublisher = pivotTable.getDoubleTopic("LeftMotorVoltage").publish();
    rightMotorVoltagePublisher = pivotTable.getDoubleTopic("RightMotorVoltage").publish();
    targetAnglePublisher = pivotTable.getDoubleTopic("TargetAngle").publish();
    statusPublisher = pivotTable.getStringTopic("Status").publish();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID Configuration
    config.Slot0.kP = PivotConstants.kP;
    config.Slot0.kI = PivotConstants.kI;
    config.Slot0.kD = PivotConstants.kD;
    config.Slot0.kV = PivotConstants.kV;
    config.Slot0.kG = PivotConstants.kG;

    // Gravity compensation
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion Magic configuration
    config.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = PivotConstants.MAX_ACCELERATION;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = PivotConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Neutral mode
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Invert right motor (motors face each other)
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Apply configuration to left motor
    leftKraken.getConfigurator().apply(config);

    // Change invert for right motor and apply
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightKraken.getConfigurator().apply(config);

    // Make right motor follow left motor
    rightKraken.setControl(new Follower(PivotConstants.LEFT_KRAKEN_ID, true));
  }

  private void configureEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    // Configure sensor settings (adjust direction if needed based on  mounting)
    // By default, CANcoder measures in rotations

    // Apply configuration
    canCoder.getConfigurator().apply(config);
  }

  /**
   * Gets the current pivot angle in degrees from the CANcoder
   */
  public double getAngleDegrees() {
    // Get position from CANcoder (in rotations) and convert to degrees
    // Accounting for gear ratio
    return canCoder.getPosition().getValueAsDouble() * 360.0 / PivotConstants.GEAR_RATIO;
  }

  /**
   * Gets the absolute position from the CANcoder in rotations
   */
  public double getAbsolutePosition() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Check if top limit switch is pressed
   */
  public boolean isAtTopLimit() {
    return !topLimitSwitch.get(); // Limit switches are normally open, active low
  }

  /**
   * Check if bottom limit switch is pressed
   */
  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get(); // Limit switches are normally open, active low
  }

  /**
   * Set pivot to a specific angle using closed-loop control
   * @param angleDegrees Target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    // Clamp angle to limits
    angleDegrees = Math.max(PivotConstants.MIN_ANGLE_DEGREES,
                           Math.min(PivotConstants.MAX_ANGLE_DEGREES, angleDegrees));

    // Convert degrees to motor rotations
    double motorRotations = (angleDegrees * PivotConstants.GEAR_RATIO) / 360.0;

    // Calculate gravity feedforward position offset (horizontal position in rotations)
    double horizontalRotations = (PivotConstants.HORIZONTAL_ANGLE_DEGREES * PivotConstants.GEAR_RATIO) / 360.0;

    // Use PositionVoltage with gravity feedforward
    leftKraken.setControl(positionControl
        .withPosition(motorRotations)
        .withFeedForward(0) // Additional arbitrary feedforward if needed
        .withSlot(PivotConstants.PID_SLOT));

    // Log target angle
    targetAnglePublisher.set(angleDegrees);
    DataLogManager.log("Pivot setpoint: " + angleDegrees + " degrees");
  }

  /**
   * Manual control of pivot with limit switch protection
   * @param speed Percent output (-1.0 to 1.0), positive = up, negative = down
   */
  public void setManualSpeed(double speed) {
    // Limit switch protection
    if (isAtTopLimit() && speed > 0) {
      speed = 0;
    }
    if (isAtBottomLimit() && speed < 0) {
      speed = 0;
    }

    leftKraken.setControl(dutyCycleControl.withOutput(speed));
  }

  /**
   * Stop the pivot
   */
  public void stop() {
    leftKraken.setControl(dutyCycleControl.withOutput(0));
  }

  /**
   * Zero the CANcoder position to 0 degrees
   * This should be called when the pivot is at the bottom limit switch
   */
  public void zeroEncoder() {
    canCoder.setPosition(0.0);
    DataLogManager.log("Pivot encoder zeroed at bottom limit");
  }

  /**
   * Set the CANcoder position to the maximum angle
   * This should be called when the pivot is at the top limit switch
   */
  public void setEncoderToMax() {
    // Convert max angle to rotations for CANcoder
    double maxRotations = (PivotConstants.MAX_ANGLE_DEGREES * PivotConstants.GEAR_RATIO) / 360.0;
    canCoder.setPosition(maxRotations);
    DataLogManager.log("Pivot encoder set to max (" + PivotConstants.MAX_ANGLE_DEGREES + "°) at top limit");
  }

  /**
   * Get motor temperature
   */
  public double getLeftMotorTemp() {
    return leftKraken.getDeviceTemp().getValueAsDouble();
  }

  public double getRightMotorTemp() {
    return rightKraken.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Get motor currents
   */
  public double getLeftMotorCurrent() {
    return leftKraken.getSupplyCurrent().getValueAsDouble();
  }

  public double getRightMotorCurrent() {
    return rightKraken.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // Get current values
    double currentAngle = getAngleDegrees();
    double absolutePos = getAbsolutePosition();
    double leftCurrent = getLeftMotorCurrent();
    double rightCurrent = getRightMotorCurrent();
    double leftTemp = getLeftMotorTemp();
    double rightTemp = getRightMotorTemp();
    double leftVoltage = leftKraken.getMotorVoltage().getValueAsDouble();
    double rightVoltage = rightKraken.getMotorVoltage().getValueAsDouble();
    boolean topLimit = isAtTopLimit();
    boolean bottomLimit = isAtBottomLimit();

    // Check for bottom limit switch press and auto-zero encoder
    if (bottomLimit && !previousBottomLimitState) {
      // Rising edge detection - limit switch just got pressed
      zeroEncoder();
      String status = "Encoder zeroed at bottom limit (0°)";
      SmartDashboard.putString("Pivot/Status", status);
      statusPublisher.set(status);
    }
    previousBottomLimitState = bottomLimit;

    // Check for top limit switch press and set to max position
    if (topLimit && !previousTopLimitState) {
      // Rising edge detection - limit switch just got pressed
      setEncoderToMax();
      String status = "Encoder set to max at top limit (" + PivotConstants.MAX_ANGLE_DEGREES + "°)";
      SmartDashboard.putString("Pivot/Status", status);
      statusPublisher.set(status);
    }
    previousTopLimitState = topLimit;

    // Publish to NetworkTables for AdvantageScope
    anglePublisher.set(currentAngle);
    absolutePositionPublisher.set(absolutePos);
    leftMotorCurrentPublisher.set(leftCurrent);
    rightMotorCurrentPublisher.set(rightCurrent);
    leftMotorTempPublisher.set(leftTemp);
    rightMotorTempPublisher.set(rightTemp);
    leftMotorVoltagePublisher.set(leftVoltage);
    rightMotorVoltagePublisher.set(rightVoltage);

    // Update SmartDashboard with telemetry
    SmartDashboard.putNumber("Pivot/Angle (deg)", currentAngle);
    SmartDashboard.putNumber("Pivot/Absolute Position", absolutePos);
    SmartDashboard.putBoolean("Pivot/Top Limit", topLimit);
    SmartDashboard.putBoolean("Pivot/Bottom Limit", bottomLimit);
    SmartDashboard.putNumber("Pivot/Left Motor Temp", leftTemp);
    SmartDashboard.putNumber("Pivot/Right Motor Temp", rightTemp);
    SmartDashboard.putNumber("Pivot/Left Motor Current", leftCurrent);
    SmartDashboard.putNumber("Pivot/Right Motor Current", rightCurrent);
    SmartDashboard.putNumber("Pivot/Left Motor Voltage", leftVoltage);
    SmartDashboard.putNumber("Pivot/Right Motor Voltage", rightVoltage);
  }
}
