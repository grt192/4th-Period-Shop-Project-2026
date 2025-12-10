// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TankDriveSubsystem extends SubsystemBase {

  // motors
  private final SparkMax leftFront  = new SparkMax(DriveConstants.left_front_motor_id, MotorType.kBrushless);
  private final SparkMax leftBack   = new SparkMax(DriveConstants.left_back_motor_id, MotorType.kBrushless);
  private final SparkMax rightFront = new SparkMax(DriveConstants.right_front_motor_id, MotorType.kBrushless);
  private final SparkMax rightBack  = new SparkMax(DriveConstants.right_back_motor_id, MotorType.kBrushless);

  // WPILib DifferentialDrive for tank/arcade drive
  private final DifferentialDrive drive;

  public TankDriveSubsystem() {
    // config left
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kCoast);
    leftConfig.inverted(true);  // Invert left side
    leftFront.configure(leftConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    leftBack.configure(leftConfig.follow(leftFront), SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

    // config right
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kCoast);
    rightConfig.inverted(false);  // Right side not inverted
    rightFront.configure(rightConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    rightBack.configure(rightConfig.follow(rightFront), SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

    // Initialize DifferentialDrive with leader motors
    drive = new DifferentialDrive(leftFront, rightFront);
    drive.setDeadband(DriveConstants.DEADBAND);
  }

  /**
   * Arcade drive using WPILib's DifferentialDrive
   * @param forward Forward/backward speed (typically left stick Y)
   * @param rotation Rotation speed (typically right stick X)
   */
  public void arcadeDrive(double forward, double rotation) {
    drive.arcadeDrive(forward, rotation);
  }

  /**
   * Tank drive using WPILib's DifferentialDrive
   * @param leftSpeed Left side speed
   * @param rightSpeed Right side speed
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  //Set all motors to Brake / Coast mode

  public void setBrakeMode(boolean brake) {
    IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
    setIdleMode(leftFront, leftBack, mode);
    setIdleMode(rightFront, rightBack, mode);
  }

  // helper (reduces repitition)
  private void setIdleMode(SparkMax m1, SparkMax m2, IdleMode mode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(mode);
    m1.configure(config, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
    m2.configure(config, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
  }
}