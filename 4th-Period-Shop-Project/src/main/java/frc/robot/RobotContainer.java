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
  public double joyConLeft;
  public double joyConRight;


  public RobotContainer() {

    joyConLeft = 0;
    joyConRight = 0;

    configureBindings();
    configureDefaultCommands();


  }

  private void configureBindings() {
    // MAX POSITION: Circle
    driverController.circle().onTrue(
      new PositionPivot(pivotSubsystem, PivotConstants.MAX_POSITION)
    );

    // INTAKE POSITION: Cross
    driverController.cross().onTrue(
      new PositionPivot(pivotSubsystem, PivotConstants.INTAKE_POSITION)
    );

    // MIN POSITION: Square
    driverController.square().onTrue(
      new PositionPivot(pivotSubsystem, PivotConstants.MIN_POSITION)
    );
    //PNEUMATICS: Triangle
    driverController.triangle().onTrue(
      new InstantCommand(() -> pneumaticSubsystem.togglePneumatic(), pneumaticSubsystem)
    );

    //INTAKE: Create buttton (Ran out of buttons to use .. I know it's small but please change if a better button is available)
    driverController.create().onTrue(
      new PositionServo(intakeSubsystem, ServoConstants.SERVO_POSITION) );
    
    //Intake: R1 close L1 open
    driverController.R1().whileTrue(
      new ManualServo(intakeSubsystem, ServoConstants.INTAKE_SPEED)
    );

    driverController.L1().whileTrue(
      new ManualServo(intakeSubsystem, -ServoConstants.INTAKE_SPEED)
    );
}
  private void configureDefaultCommands() {
    tankDrive.setDefaultCommand(new RunCommand(() -> {
      this.joyConLeft = driverController.getLeftY();
      this.joyConRight = driverController.getRightX(); 
      tankDrive.altDrive(joyConLeft, joyConRight);
    }, tankDrive));
  
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