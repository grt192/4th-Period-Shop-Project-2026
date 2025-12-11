// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    //MOTOR CAN IDs
    public static final int left_front_motor_id = 3; //UPDATE
    public static final int left_back_motor_id = 10;

    public static final int right_front_motor_id = 7;
    public static final int right_back_motor_id = 9; //UPDATE

    // Joystick deadband to prevent stick drift
    public static final double DEADBAND = 0.02;
  }
public static final class PivotConstants {

  // Motors
  public static final int PIVOT_MOTOR_LEFT_ID = 4;
  public static final int PIVOT_MOTOR_RIGHT_ID = 2;

  // PID constants for TorqueCurrentFOC with 14:1 gear ratio
  public static final double PIVOT_P = 0.1;  // Proportional gain (torque in amps per rotation error)
  public static final double PIVOT_I = 0.0;  // Integral
  public static final double PIVOT_D = 0.0;  // Derivative
  public static final double PIVOT_KG = 0.0;  // Gravity feedforward (torque in amps to counteract gravity)
  public static final double PIVOT_KS = 0.0;  // Static friction feedforward (torque in amps to overcome static friction)

  public static final int ENCODER_ID = 0;

  // Speed and current limits
  public static final double PIVOT_MANUAL_SPEED = 0.1;  // Manual control speed (0.0 to 1.0)
  public static final double PIVOT_MAX_CURRENT = 80.0;   // Maximum torque current (amps)
  public static final double PIVOT_CURRENT_LIMIT = 80.0; // Stator current limit (amps)
  public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 40.0; // Supply current limit (amps)

  // Angle limits (in degrees)
  public static final double MIN_ANGLE = 0.0;   // Minimum angle / stowed position
  public static final double MAX_ANGLE = 90.0;  // Maximum angle / fully raised position

  public static final double GEAR_RATIO = 14; // 14 motor rotations = 1 output rotation

  // Preset positions (in degrees)
  public static final double MID_POSITION = 45.0;  // Mid position
  public static final double POSITION_TOLERANCE = 1.5;
}
public static final class PneumaticsConstants {
  public static final int pneumaticID = 1;        // Solenoid channel on PCM (0-7)
  public static final int pneumaticCANId = 10;    // PCM CAN ID (commonly 10-19)
}

public static final class ServoConstants {
  public static final int SERVO_ID = 9;  // PWM channel (0-9), changed from 0 to avoid DIO conflict

  // Servo positions: 0° = 0.0, 120° = 0.67 (assuming 180° servo range)
  public static final double HOME_POSITION = 0.0;  // 0 degrees - boot position
  public static final double OPEN_POSITION = 0.67; // 120 degrees counterclockwise

  public static final double MAX_POSITION = 1.0;
  public static final double MIN_POSITION = 0.0;
  public static final double INTAKE_TOLERANCE = 0.05;

  public static final double INTAKE_SPEED = 0.02;

}
  
  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}