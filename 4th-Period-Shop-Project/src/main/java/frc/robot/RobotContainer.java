// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ServoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.ManualServo;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.PositionServo;
import frc.robot.commands.PositionPivot;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.StopperServo;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class RobotContainer {
  // Subsystems
  private final TankDriveSubsystem tankDrive= new TankDriveSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final PneumaticsSubsystem pneumaticSubsystem = new PneumaticsSubsystem();
  private final StopperServo intakeSubsystem = new StopperServo();


  // Controllers
  private final CommandPS5Controller driverController = new CommandPS5Controller(OIConstants.DRIVER_CONTROLLER_PORT);


  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();


  }

  private void configureBindings() {
    // Square: 0° (stowed)
    driverController.square().onTrue(
      new PositionPivot(pivotSubsystem, PivotConstants.MIN_ANGLE)
    );

    // Cross: 45° (mid position)
    driverController.cross().onTrue(
      new PositionPivot(pivotSubsystem, PivotConstants.MID_POSITION)
    );

    // Circle: 90° (fully raised)
    driverController.circle().onTrue(
      new PositionPivot(pivotSubsystem, PivotConstants.MAX_ANGLE)
    );
    //PNEUMATICS: Triangle
    driverController.triangle().onTrue(
      new InstantCommand(() -> pneumaticSubsystem.togglePneumatic(), pneumaticSubsystem)
    );

    //SERVO: Create button - move to open position (120 degrees)
    driverController.create().onTrue(
      new PositionServo(intakeSubsystem, ServoConstants.OPEN_POSITION) );

    //SERVO: R1 open (counterclockwise to 120°), L1 close (back to home 0°)
    driverController.R1().whileTrue(
      new ManualServo(intakeSubsystem, ServoConstants.INTAKE_SPEED)
    );

    driverController.L1().whileTrue(
      new ManualServo(intakeSubsystem, -ServoConstants.INTAKE_SPEED)
    );
}
  private void configureDefaultCommands() {
    tankDrive.setDefaultCommand(new RunCommand(() -> {
      tankDrive.arcadeDrive(driverController.getLeftY(), driverController.getRightX());
    }, tankDrive));
  
    // Pivot Configs: R2 for pivot up and L2 for pivot down
      pivotSubsystem.setDefaultCommand(
      new ManualPivot(pivotSubsystem, () -> driverController.getL2Axis() - driverController.getR2Axis()
      )
    );

}
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}