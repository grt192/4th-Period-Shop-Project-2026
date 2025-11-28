package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class PositionIntake extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double targetPosition;
    
    public PositionIntake(IntakeSubsystem intakeSubsystem, double targetPosition) {
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
        return (Math.abs(intakeSubsystem.getPosition() - targetPosition) < IntakeConstants.INTAKE_TOLERANCE);
    }
}