// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class SetPivotPosition extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final double targetAngle;

  /**
   * Creates a new SetPivotPosition command.
   *
   * @param pivotSubsystem The pivot subsystem
   * @param targetAngle Target angle in degrees
   */
  public SetPivotPosition(PivotSubsystem pivotSubsystem, double targetAngle) {
    this.pivotSubsystem = pivotSubsystem;
    this.targetAngle = targetAngle;

    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {
    pivotSubsystem.setAngle(targetAngle);
    DataLogManager.log("SetPivotPosition command started: target=" + targetAngle + "°");
  }

  @Override
  public void execute() {
    // PID control is handled by the motor controller
  }

  @Override
  public void end(boolean interrupted) {
    // Keep holding position (don't stop)
    if (interrupted) {
      DataLogManager.log("SetPivotPosition command interrupted at " + pivotSubsystem.getAngleDegrees() + "°");
    } else {
      DataLogManager.log("SetPivotPosition command finished at target " + targetAngle + "°");
    }
  }

  @Override
  public boolean isFinished() {
    // Command finishes when pivot reaches target position
    return Math.abs(pivotSubsystem.getAngleDegrees() - targetAngle) < PivotConstants.POSITION_TOLERANCE;
  }
}
