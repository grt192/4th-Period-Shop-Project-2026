package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ServoConstants;


public class StopperServo extends SubsystemBase{
    private Servo intakeServo = new Servo(ServoConstants.SERVO_ID);
    public StopperServo() {
        closeIntake();
    }

    public void openIntake() {
        intakeServo.setPosition(ServoConstants.OPEN_POSITION);
    }
 
    public void closeIntake() {
        intakeServo.setPosition(ServoConstants.CLOSED_POSITION);
    }
    
    public boolean isOpen() {
        return Math.abs(currentPosition() - ServoConstants.OPEN_POSITION) < ServoConstants.INTAKE_TOLERANCE;
    }
    public void setPosition(double position) {
        double clampedPosition = Math.max(ServoConstants.MIN_POSITION, Math.min(ServoConstants.MAX_POSITION, position));
        intakeServo.setPosition(clampedPosition);
    }
    public double currentPosition() {
        return intakeServo.getPosition();
    }

    public void move(double speed) {
        double newPosition = currentPosition() + speed;
        if (speed > 0 && currentPosition() >= ServoConstants.MAX_POSITION) {
            return;
        }
        if (speed < 0 && currentPosition() <= ServoConstants.MIN_POSITION) {
            return;
        }
        setPosition(newPosition);
}
}