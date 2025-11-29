// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StopperServo;

public class ManualServo extends Command {
  private final StopperServo intakeSubsystem;
  private final double speed;

  public ManualServo(StopperServo intakeSubsystem, double speed) {
    this.intakeSubsystem = intakeSubsystem;
    this.speed = speed;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSubsystem.move(speed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}