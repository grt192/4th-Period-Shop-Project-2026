// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends Command {
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier forwardSpeed;
  private final DoubleSupplier rotationSpeed;

  /**
   * Creates a new ArcadeDrive command.
   *
   * @param driveSubsystem The drive subsystem
   * @param forwardSpeed Supplier for forward/backward speed
   * @param rotationSpeed Supplier for rotation speed
   */
  public ArcadeDrive(DriveSubsystem driveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier rotationSpeed) {
    this.driveSubsystem = driveSubsystem;
    this.forwardSpeed = forwardSpeed;
    this.rotationSpeed = rotationSpeed;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    driveSubsystem.arcadeDrive(forwardSpeed.getAsDouble(), rotationSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
