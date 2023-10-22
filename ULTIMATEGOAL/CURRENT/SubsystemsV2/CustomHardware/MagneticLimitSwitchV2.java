package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.CustomHardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.jetbrains.annotations.NotNull;

/**
 * REV Magnetic Limit Switch implementation class.
 * @author Domenic Iorfida, FTC 17014
 */
public class MagneticLimitSwitchV2 {

    private DigitalChannel limitSwitch;

    /**
     * Constructor.
     * @param limitSwitch The digital device port for the limit switch.
     */
    public MagneticLimitSwitchV2(@NotNull DigitalChannel limitSwitch){
        this.limitSwitch = limitSwitch;
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Method that interprets the reading from the limit switch.
     * @return If the switch detects the magnet.
     */
    public boolean isMagnetPresent(){
        return !limitSwitch.getState();
    }
}
