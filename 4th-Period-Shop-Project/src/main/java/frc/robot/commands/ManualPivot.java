// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import java.util.function.DoubleSupplier;

public class ManualPivot extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final DoubleSupplier speedSupplier;

  /**
   * Creates a new ManualPivot command.
   *
   * @param pivotSubsystem The pivot subsystem
   * @param speedSupplier Supplier for pivot speed (positive = up, negative = down)
   */
  public ManualPivot(PivotSubsystem pivotSubsystem, DoubleSupplier speedSupplier) {
    this.pivotSubsystem = pivotSubsystem;
    this.speedSupplier = speedSupplier;

    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setManualSpeed(speedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
