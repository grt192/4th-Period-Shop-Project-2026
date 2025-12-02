// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PivotSubsystem;

/**
 * Instant command to set pivot to a target angle.
 * Motor controller maintains position with closed-loop control.
 * Returns control to default command (manual) immediately.
 */
public class SetPivotPosition extends InstantCommand {
  /**
   * Creates a new SetPivotPosition command.
   *
   * @param pivotSubsystem The pivot subsystem
   * @param targetAngle Target angle in degrees
   */
  public SetPivotPosition(PivotSubsystem pivotSubsystem, double targetAngle) {
    super(
      () -> {
        pivotSubsystem.setAngle(targetAngle);
        DataLogManager.log("SetPivotPosition: target=" + targetAngle + "°");
      },
      pivotSubsystem
    );
  }
}
