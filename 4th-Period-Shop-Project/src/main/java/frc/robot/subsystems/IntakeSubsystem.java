package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase{
    private Servo intakeServo = new Servo(IntakeConstants.SERVO_ID);
    public IntakeSubsystem() {
        closeIntake();
    }

    public void openIntake() {
        intakeServo.setPosition(IntakeConstants.OPEN_POSITION);
    }
 
    public void closeIntake() {
        intakeServo.setPosition(IntakeConstants.CLOSED_POSITION);
    }
    
    public boolean isOpen() {
        return Math.abs(intakeServo.getPosition() - IntakeConstants.OPEN_POSITION) < IntakeConstants.INTAKE_TOLERANCE;
    }
    public void setPosition(double position) {
        double clampedPosition = Math.max(IntakeConstants.MIN_POSITION, Math.min(IntakeConstants.MAX_POSITION, position));
        intakeServo.setPosition(clampedPosition);
    }
    public double getPosition() {
        return intakeServo.getPosition();
    }

    public void move(double speed) {
        double newPosition = intakeServo.getPosition() + speed;
        setPosition(newPosition);
    
    }
}
