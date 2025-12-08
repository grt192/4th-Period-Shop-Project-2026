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
  }
public static final class PivotConstants {

  // Motors
  public static final int PIVOT_MOTOR_LEFT_ID = 5;  // Changed from 1 to avoid conflict with left_back drive motor
  public static final int PIVOT_MOTOR_RIGHT_ID = 6; // Changed from 2 to avoid conflict with left_front drive motor

  // PID constants

  public static final double PIVOT_P = 0.01; 
  public static final double PIVOT_I = 0.0; 
  public static final double PIVOT_D = 0.0;
  public static final double PIVOT_F = 0.0;

  public static final int ENCODER_ID = 0;
  public static final int CANDI_ID = 11; // CANdi (CANdleDigitalInput) CAN ID for limit switches
  public static final double PIVOT_MANUAL_SPEED = 0.15;  // Slow speed for manual control

  public static final double MAX_ANGLE = 45.0;
  public static final double MIN_ANGLE = 0.0;

  public static final double GEAR_RATIO = 14.0; // 14 motor rotations = 1 output rotation

  // Pivot Positions (in degrees)
  public static final double MAX_POSITION = 45.0;     // Fully raised position
  public static final double INTAKE_POSITION = 15.0;  // Low position for intake
  public static final double MIN_POSITION = 0.0;      // Fully lowered/stowed position
  public static final double POSITION_TOLERANCE = 1.5;
}
public static final class PneumaticsConstants {
  public static final int pneumaticID = 0;        // Solenoid channel on PCM (0-7)
  public static final int pneumaticCANId = 10;    // PCM CAN ID (commonly 10-19)
}

public static final class ServoConstants {
  public static final int SERVO_ID = 9;  // PWM channel (0-9), changed from 0 to avoid DIO conflict

  // to update opem/closed positions
  public static final double CLOSED_POSITION = 0.0; 
  public static final double OPEN_POSITION = 0.8; 
  
  public static final double MAX_POSITION = 1.0; 
  public static final double MIN_POSITION = 0.0;
  public static final double INTAKE_TOLERANCE = 0.05; // 9 degrees

  public static final double INTAKE_SPEED = 0.02;

  public static final double SERVO_POSITION = 0; //to update 

  
}
  
  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}