// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.PosPivot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;


public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  // Controllers
  private final CommandPS5Controller driverController = new CommandPS5Controller(OIConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    // MIN POSITION: Circle
    driverController.circle().onTrue(
      new PosPivot(pivotSubsystem, PivotConstants.MAX_POSITION)
    );

    // INTAKE POSITION: Cross
    driverController.cross().onTrue(
      new PosPivot(pivotSubsystem, PivotConstants.INTAKE_POSITION)
    );

    // MAX POSITION: Square
    driverController.square().onTrue(
      new PosPivot(pivotSubsystem, PivotConstants.MIN_POSITION)
    );
  }
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

    // Pivot Configs: R2 for pivot up and L2 for pivot down
      pivotSubsystem.setDefaultCommand(
      new ManualPivot(pivotSubsystem, () -> driverController.getR2Axis() - driverController.getL2Axis()
      )
    );
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
