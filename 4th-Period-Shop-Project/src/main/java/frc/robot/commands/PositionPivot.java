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

  /**
   * Creates a new PositionPivot command.
   * 
   * @param pivotSubsystem The pivot subsystem to control
   * @param targetAngle The desired angle in degrees for the pivot to move to
   */
  public PositionPivot(PivotSubsystem pivotSubsystem, double targetAngle) {
    this.pivotSubsystem = pivotSubsystem;
    this.targetAngle = targetAngle;
    addRequirements(pivotSubsystem);
  }


   // Commands the pivot to move to the target angle with PID control
  public void initialize() {
  }

  @Override
  public void execute() {
    pivotSubsystem.setAngle(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
  }
  /**
   * Finishes when the pivot is within set tolerance of the target angle
   * @return True if pivot reaches target position within the configured tolerance
   */
  @Override
  public boolean isFinished() {
    return isWithinTolerance();
  }

  /**
   * Checks if the current pivot angle is within acceptable tolerance of target
   * @return True if position error < configured tolerance
   */
  public boolean isWithinTolerance() {
    double positionError = Math.abs(pivotSubsystem.getAngleDegrees() - targetAngle);
    return positionError < PivotConstants.POSITION_TOLERANCE;
  }
}