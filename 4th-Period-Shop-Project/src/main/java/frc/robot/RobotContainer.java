// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // Controllers
  private final CommandPS5Controller driverController = new CommandPS5Controller(OIConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {}

  private void configureDefaultCommands() {
    // Set arcade drive as default command
    // Left stick Y-axis for forward/backward, right stick X-axis for rotation
    driveSubsystem.setDefaultCommand(
      new ArcadeDrive(
        driveSubsystem,
        () -> -driverController.getLeftY(),  // Forward/backward (inverted)
        () -> driverController.getRightX()   // Rotation
      )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
