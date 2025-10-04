// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.TankDriveSubsystem;

public class RobotContainer {

  // Subsystems defined
  private final TankDriveSubsystem tankSubsystem;

  // Controls defined
  private final CommandPS5Controller Controller;
  public double joyConLeft;
  public double joyConRight;

  public RobotContainer() {

    // Subsystem created
    tankSubsystem = new TankDriveSubsystem();

    // Config bindings
    Controller = new CommandPS5Controller(0);
    joyConLeft = 0;
    joyConRight = 0;

    configureBindings();
  }


  private void configureBindings() {
    tankSubsystem.setDefaultCommand(new RunCommand(() -> {
      this.joyConLeft = Controller.getLeftY(); // Left Y-axis for PS5 controller
      this.joyConRight = Controller.getRightX(); // Right Y-axis for PS5 controller
      tankSubsystem.altDrive(joyConLeft, joyConRight);
    }, tankSubsystem));
  }

  // /**
  // * Use this to pass the autonomous command to the main {@link Robot} class.
  // *
  // * @return the command to run in autonomous
  // */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}