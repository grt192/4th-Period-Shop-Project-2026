package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ServoConstants;
import frc.robot.subsystems.StopperServo;

public class PositionServo extends Command {
    private final StopperServo intakeSubsystem;
    private final double targetPosition;
    
    /**
     * Creates a new PositionServo command.
     * 
     * @param intakeSubsystem The stopper servo subsystem
     * @param targetPosition Desired position for the servo to move to
     */
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
        // Calculate position error and check if within acceptable tolerance
        return (Math.abs(intakeSubsystem.currentPosition() - targetPosition) < ServoConstants.INTAKE_TOLERANCE);
    }
}