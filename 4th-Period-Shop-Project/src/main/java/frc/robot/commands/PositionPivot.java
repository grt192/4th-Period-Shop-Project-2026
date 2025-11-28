// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PositionPivot extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final double targetAngle;

  public PositionPivot(PivotSubsystem pivotSubsystem, double targetAngle) {
    this.pivotSubsystem = pivotSubsystem;
    this.targetAngle = targetAngle;
    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {
    pivotSubsystem.setAngle(targetAngle);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isWithinTolerance();
  }
  public boolean isWithinTolerance() {
    double positionError = Math.abs(pivotSubsystem.getAngleDegrees() - targetAngle);
    return positionError < PivotConstants.POSITION_TOLERANCE;
  }
}