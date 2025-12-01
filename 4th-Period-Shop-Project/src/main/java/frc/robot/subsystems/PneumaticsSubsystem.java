package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
    private final Solenoid solenoid = new Solenoid(Constants.PneumaticsConstants.pneumaticCANId, PneumaticsModuleType.CTREPCM,
            Constants.PneumaticsConstants.pneumaticID);
            
    public PneumaticsSubsystem() {
        // Starts with pneumatic retracted
        solenoid.set(false); 
    }

       /**
     * Toggles solenoid between extended and retracted 
     * If extended, it'll retract & if retracted, it'll extend.
     */
    public void togglePneumatic() {
        solenoid.toggle();
    }
    public void extend() {
        solenoid.set(true);
    }

    public void retract() {
        solenoid.set(false);
    }

    public boolean isExtended() {
        return solenoid.get();
    }
}