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
public static final class PivotConstants {

  // Motors
  public static final int PIVOT_MOTOR_LEFT_ID = 1;  //placeholder
  public static final int PIVOT_MOTOR_RIGHT_ID = 2; //placeholder

  // Limit switches
  public static final int MIN_LIMIT_SWITCH_ID = 0;  //placeholder
  public static final int MAX_LIMIT_SWITCH_ID = 1; //placeholder

  // PID constants

  public static final double PIVOT_P = 0.01; 
  public static final double PIVOT_I = 0.0; 
  public static final double PIVOT_D = 0.0;
  public static final double PIVOT_F = 0.0;

  public static final int ENCODER_ID = 0;
  public static final double PIVOT_SPEED = 0.3;
  
  public static final double MAX_ANGLE = 0.0;  
  public static final double MIN_ANGLE = 90.0;   
  
  public static final double GEAR_RATIO = 0.8 ; // update with real value

  // Pivot Positions (UPDATE PLACEHOLDERS!!)
  public static final double MAX_POSITION = 0;
  public static final double INTAKE_POSITION = 0;
  public static final double MIN_POSITION = 0;
  public static final double POSITION_TOLERANCE = 1.5;
}
public static final class PneumaticsConstants {
  public static final int pneumaticID = 0;
  public static final int pneumaticCANId = 0;
}

public static final class IntakeConstants {
  public static final int SERVO_ID = 0;

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
