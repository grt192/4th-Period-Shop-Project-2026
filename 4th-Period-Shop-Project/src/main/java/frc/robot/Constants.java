// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

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
    public static final double GEAR_RATIO = 64.0; // 64:1 reduction

    // Position Limits (in degrees)
    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 180.0;

    // PID Constants
    public static final int PID_SLOT = 0;
    public static final double kP = 24.0;  // Proportional gain - aggressive for position control
    public static final double kI = 0.0;   // Integral gain - usually not needed with good kP and kG
    public static final double kD = 1.5;   // Derivative gain - adds damping to prevent oscillation
    public static final double kV = 0.12;  // Velocity feedforward - volts per (rotation/sec)
    public static final double kG = 0.25;  // Gravity feedforward - volts to hold arm horizontal (~3A * 12V / 144A-stall)

    // Gravity Compensation
    // Horizontal angle in degrees (where maximum gravity compensation is needed)
    public static final double HORIZONTAL_ANGLE_DEGREES = 0.0; // TODO: Set to horizontal position

    // Motion Magic / Profile Constants
    public static final double MAX_VELOCITY = 2.5; // rotations per second (reasonable for arm movement)
    public static final double MAX_ACCELERATION = 5.0; // rotations per second² (smooth but responsive) 

    // Current Limits
    public static final int CURRENT_LIMIT = 40; // amps
    public static final int TRIGGER_THRESHOLD_CURRENT = 60; // amps
    public static final double TRIGGER_THRESHOLD_TIME = 0.1; // seconds

    // Manual Control (Torque Current)
    public static final double MANUAL_TORQUE_CURRENT = 30.0; // Amps for manual control (responsive but safe)

    // Preset Positions (in degrees)
    public static final double POSITION_1 = -30.0; // Low position
    public static final double POSITION_2 = 0.0;   // Horizontal position
    public static final double POSITION_3 = 30.0;  // High position

    // Position tolerance (degrees)
    public static final double POSITION_TOLERANCE = 2.0;

    // Simulation Parameters
    public static final double ARM_MASS_KG = 8.0;  // Total mass of arm mechanism (TODO: Measure actual mass)
    public static final double ARM_LENGTH_METERS = Units.inchesToMeters(2.0); // Pivot to center of mass (TODO: Measure actual length)

    // Moment of inertia - calculated from mass and length (or use CAD value for better accuracy)
    public static final double ARM_MOI_KG_M2 = SingleJointedArmSim.estimateMOI(
        ARM_LENGTH_METERS, ARM_MASS_KG
    ); // kg⋅m² - TODO: Replace with CAD-calculated value if available
  }
}
