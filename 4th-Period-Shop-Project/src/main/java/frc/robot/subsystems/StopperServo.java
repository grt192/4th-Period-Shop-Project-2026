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
    // Set servo to the preset closed position
    public void closeIntake() {
        intakeServo.setPosition(ServoConstants.CLOSED_POSITION);
    }
    
    public boolean isOpen() {
        return Math.abs(currentPosition() - ServoConstants.OPEN_POSITION) < ServoConstants.INTAKE_TOLERANCE;
    }

     /**
     * Sets servo to a specific position
     * Clamped values: position limited to a valid range between the min & max position
     * @param position Desired servo position (typically 0.0 to 1.0)
     */
    public void setPosition(double position) {
        double clampedPosition = Math.max(ServoConstants.MIN_POSITION, Math.min(ServoConstants.MAX_POSITION, position));
        intakeServo.setPosition(clampedPosition);
    }

    // Get's the servo position and returns it (0.0-1.0)
    public double currentPosition() {
        return intakeServo.getPosition();
    }


     /**
     * @param speed The amount to change position by, either + or - where 
     * 
     * Positive values move toward MAX_POSITION & negative toward MIN_POSITION.
     * 
     * Values are clamped at max and min positions
     */
    public void move(double speed) {
        // Calculate new target position
        double newPosition = currentPosition() + speed;
        if (speed > 0 && currentPosition() >= ServoConstants.MAX_POSITION) {
            return;
        }
        if (speed < 0 && currentPosition() <= ServoConstants.MIN_POSITION) {
            return;
        }
    
        // Apply new position
        setPosition(newPosition);
}
}