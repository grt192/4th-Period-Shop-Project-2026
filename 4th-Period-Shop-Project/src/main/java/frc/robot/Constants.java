// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    //MOTOR CAN IDs
    public static final int left_front_motor_id = 2;
    public static final int left_back_motor_id = 1;

    public static final int right_front_motor_id = 18;
    public static final int right_back_motor_id = 4;

    // Joystick deadband to prevent stick drift
    public static final double DEADBAND = 0.02;
  }
public static final class PivotConstants {

  // Motors
  public static final int PIVOT_MOTOR_LEFT_ID = 5;
  public static final int PIVOT_MOTOR_RIGHT_ID = 6;

  // PID constants for 15lb seesaw with 14:1 gear ratio
  public static final double PIVOT_P = 0.5;    // Proportional gain - increase if response is too slow
  public static final double PIVOT_I = 0.0;    // Integral - keep at 0 initially
  public static final double PIVOT_D = 0.05;   // Derivative - helps reduce oscillation
  public static final double PIVOT_F = 0.05;   // Feedforward/gravity compensation - tune this for holding position

  public static final int ENCODER_ID = 0;

  // Speed and current limits
  public static final double PIVOT_MANUAL_SPEED = 0.1;  // Manual control speed (0.0 to 1.0)
  public static final double PIVOT_MAX_VOLTAGE = 12.0;   // Maximum voltage output
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