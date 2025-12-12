package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
    private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(
            Constants.PneumaticsConstants.pneumaticCANId,
            PneumaticsModuleType.CTREPCM,
            Constants.PneumaticsConstants.pneumaticForwardChannel,
            Constants.PneumaticsConstants.pneumaticReverseChannel);
    private final Compressor compressor = new Compressor(Constants.PneumaticsConstants.pneumaticCANId, PneumaticsModuleType.CTREPCM);

    public PneumaticsSubsystem() {
        // Starts with pneumatic retracted
        doubleSolenoid.set(DoubleSolenoid.Value.kReverse);

        // Enable the compressor in "digital" mode (automatically turns on/off based on pressure switch)
        compressor.enableDigital();
    }

       /**
     * Toggles solenoid between extended and retracted
     * If extended, it'll retract & if retracted, it'll extend.
     */
    public void togglePneumatic() {
        doubleSolenoid.toggle();
    }
    public void extend() {
        doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isExtended() {
        return doubleSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    // Compressor control methods
    public void disableCompressor() {
        compressor.disable();
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public boolean isCompressorRunning() {
        return compressor.isEnabled();
    }

    public double getCompressorCurrent() {
        return compressor.getCurrent();
    }
}