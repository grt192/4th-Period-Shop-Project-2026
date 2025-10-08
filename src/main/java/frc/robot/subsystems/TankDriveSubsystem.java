// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TankDriveSubsystem extends SubsystemBase {

  // motors
  private final WPI_TalonSRX leftFront  = new WPI_TalonSRX(DriveConstants.left_front_motor_id);
  private final WPI_TalonSRX leftBack   = new WPI_TalonSRX(DriveConstants.left_back_motor_id);
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveConstants.right_front_motor_id);
  private final WPI_TalonSRX rightBack  = new WPI_TalonSRX(DriveConstants.right_back_motor_id);

  public TankDriveSubsystem() {
    // config left
    leftBack.follow(leftFront);
    setNeutralMode(leftFront, leftBack, NeutralMode.Brake);

    // config right
    rightBack.follow(rightFront);
    setNeutralMode(rightFront, rightBack, NeutralMode.Coast);
  }

  /**
   * Set motor speeds for left and right sides.
   *
   * @param leftSpeed  Motor speed (-1.0 to 1.0).
   * @param rightSpeed Motor speed (-1.0 to 1.0).
   */
  public void setMotors(double leftSpeed, double rightSpeed) {
    leftFront.set(-leftSpeed); // invert left side
    rightFront.set(rightSpeed);
  }

  /**
   * Alternate drive mode using two joystick axes
   */
  public void altDrive(double leftAxis, double rightAxis) {
    if (leftAxis == 0 && rightAxis != 0) {
      // Spin in place
      setMotors(rightAxis, -rightAxis);
    } else if (leftAxis != 0 && rightAxis == 0) {
      // Move straight
      setMotors(leftAxis, leftAxis);
    } else {
      // mix
      setMotors((leftAxis - rightAxis) / 2.0, (leftAxis + rightAxis) / 2.0);
    }
  }

  //Set all motors to Brake / Coast mode

  public void setBrakeMode(boolean brake) {
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    setNeutralMode(leftFront, leftBack, mode);
    setNeutralMode(rightFront, rightBack, mode);
  }

  // helper (reduces repitition)
  private void setNeutralMode(WPI_TalonSRX m1, WPI_TalonSRX m2, NeutralMode mode) {
    m1.setNeutralMode(mode);
    m2.setNeutralMode(mode);
  }
}
