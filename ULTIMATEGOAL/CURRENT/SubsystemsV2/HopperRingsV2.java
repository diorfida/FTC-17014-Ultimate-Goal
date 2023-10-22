package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;

public class HopperRingsV2 {

    public enum NumRingsV2{
        ZERO_RINGS,
        ONE_RING,
        TWO_RINGS,
        THREE_RINGS
    }

    private static final double threeRingThreshold = 2;
    private static final double twoRingThreshold = threeRingThreshold + .75;
    private static final double oneRingThreshold = twoRingThreshold + .75;

    private Rev2mDistanceSensor ringHeight;

    public HopperRingsV2(@NotNull HardwareMap hardwareMap){
        ringHeight = hardwareMap.get(Rev2mDistanceSensor.class, "ringHeight");
    }

    public NumRingsV2 determineRingHopperHeight(){
        if (ringHeight.getDistance(DistanceUnit.INCH) < threeRingThreshold)
            return NumRingsV2.THREE_RINGS;
        else if (ringHeight.getDistance(DistanceUnit.INCH) < twoRingThreshold)
            return NumRingsV2.TWO_RINGS;
        else if (ringHeight.getDistance(DistanceUnit.INCH) < oneRingThreshold)
            return NumRingsV2.ONE_RING;
        else
            return NumRingsV2.ZERO_RINGS;
    }
}
