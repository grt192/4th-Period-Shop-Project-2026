package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ServoConstants;
import frc.robot.subsystems.StopperServo;

public class PositionServo extends Command {
    private final StopperServo intakeSubsystem;
    private final double targetPosition;
    
    public PositionServo(StopperServo intakeSubsystem, double targetPosition) {
        this.intakeSubsystem = intakeSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        intakeSubsystem.setPosition(targetPosition);
    }
    
    @Override
    public boolean isFinished() {
        return (Math.abs(intakeSubsystem.getPosition() - targetPosition) < ServoConstants.INTAKE_TOLERANCE);
    }
}