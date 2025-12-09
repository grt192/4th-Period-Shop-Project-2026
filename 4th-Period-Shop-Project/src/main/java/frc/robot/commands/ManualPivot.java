// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import java.util.function.DoubleSupplier;

public class ManualPivot extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final DoubleSupplier speedSupplier;

   /**
   * Creates a new ManualPivot command
   * 
   * @param pivotSubsystem Pivot subsystem
   * @param speedSupplier A supplier returning the desired speed (-1.0 to 1.0), in this case its R2 and L2
   *                  
   */
  public ManualPivot(PivotSubsystem pivotSubsystem, DoubleSupplier speedSupplier) {
    this.pivotSubsystem = pivotSubsystem;
    this.speedSupplier = speedSupplier;
    addRequirements(pivotSubsystem);
  }

  // Add interruption behavior to override any and all other commands usiong the pivot subsystem
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Get the current speed value from R2 and L2
    double speedValue = speedSupplier.getAsDouble();
    pivotSubsystem.setManualSpeed(speedValue * PivotConstants.MANUAL_PIVOT_SPEED);

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