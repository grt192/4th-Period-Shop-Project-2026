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
}
