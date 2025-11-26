// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    // CAN IDs
    public static final int LEFT_FRONT_MOTOR_ID = 2; 
    public static final int LEFT_BACK_MOTOR_ID = 1;
    
    public static final int RIGHT_FRONT_MOTOR_ID = 18;
    public static final int RIGHT_BACK_MOTOR_ID = 4;

    // PID Constants
    public static final int PID_SLOT = 0;
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;

    // Motion Profile Constants
    public static final int TIMEOUT_MS = 30;
    public static final double OPEN_LOOP_RAMP = 0.25; // seconds from 0 to full throttle
    public static final double CLOSED_LOOP_RAMP = 0.0;

    // Current Limits
    public static final int CURRENT_LIMIT = 40; // amps
    public static final int TRIGGER_THRESHOLD_CURRENT = 45; // amps
    public static final double TRIGGER_THRESHOLD_TIME = 0.5; // seconds
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class PivotConstants {
    // CAN IDs
    public static final int LEFT_KRAKEN_ID = 5;
    public static final int RIGHT_KRAKEN_ID = 6;
    public static final int THROUGHBORE_ENCODER_ID = 7;

    // Digital I/O
    public static final int TOP_LIMIT_SWITCH_DIO = 0;
    public static final int BOTTOM_LIMIT_SWITCH_DIO = 1;

    // Gear Ratio (motor rotations : pivot rotations)
    public static final double GEAR_RATIO = 1.0; // TODO: Update with actual gear ratio

    // Position Limits (in degrees)
    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;

    // PID Constants
    public static final int PID_SLOT = 0;
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0; // Velocity feedforward
    public static final double kG = 0.0; // Gravity feedforward (TODO: Tune this value)

    // Gravity Compensation
    // Horizontal angle in degrees (where maximum gravity compensation is needed)
    public static final double HORIZONTAL_ANGLE_DEGREES = 0.0; // TODO: Set to horizontal position

    // Motion Magic / Profile Constants
    public static final double MAX_VELOCITY = 1.0; // rotations per second
    public static final double MAX_ACCELERATION = 2.0; 

    // Current Limits
    public static final int CURRENT_LIMIT = 40; // amps
    public static final int TRIGGER_THRESHOLD_CURRENT = 60; // amps
    public static final double TRIGGER_THRESHOLD_TIME = 0.1; // seconds

    // Manual Control (Torque Current)
    public static final double MANUAL_TORQUE_CURRENT = 20.0; // Amps for manual control (TODO: Tune this value)

    // Preset Positions (in degrees)
    public static final double POSITION_1 = 15.0; // TODO: Update with actual position
    public static final double POSITION_2 = 45.0; // TODO: Update with actual position
    public static final double POSITION_3 = 75.0; // TODO: Update with actual position

    // Position tolerance (degrees)
    public static final double POSITION_TOLERANCE = 2.0;
  }
}
