package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
    private final Solenoid solenoid = new Solenoid(
            Constants.PneumaticsConstants.pneumaticCANId,
            PneumaticsModuleType.CTREPCM,
            Constants.PneumaticsConstants.pneumaticForwardChannel
            );
            
    public PneumaticsSubsystem() {
        // Starts with pneumatic retracted
        solenoid.set(false);
    }

       /**
     * Toggles solenoid between extended and retracted 
     * If extended, it'll retract & if retracted, it'll extend.
     */
    boolean flipFlop = false;
    public void togglePneumatic() {
        System.out.println("togglePneumatic");
        if (flipFlop){
            solenoid.set(flipFlop);
            flipFlop = false;
        }
        else{
            solenoid.set(flipFlop);
            flipFlop = true;
        }
        // doubleSolenoid.toggle();
    }
}