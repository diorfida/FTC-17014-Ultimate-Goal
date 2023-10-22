package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.jetbrains.annotations.NotNull;

/**
 * The timer used for changing the color of the lights at certain points during the match.
 * @author Domenic Iorfida, FTC 17014
 */
@Config
public class LEDLights {

    private LinearOpMode opMode;

    private enum GameTime{
        DRIVER_CONTROLLED,
        ENDGAME,
        LAST_5_SECONDS,
        GAME_OVER
    }

    public enum HopperRings{
        ZERO_RINGS,
        ONE_RING,
        TWO_RINGS,
        THREE_RINGS
    }

    private static double threeRingThreshold = 1.67;
    private static double twoRingThreshold = threeRingThreshold + .75;
    private static double oneRingThreshold = twoRingThreshold + .75;

    private RevBlinkinLedDriver lights;
    private Rev2mDistanceSensor ringHeight;

    /**
     * Constructor
     */
    public LEDLights(@NotNull LinearOpMode opMode){
        this.opMode = opMode;

        lights = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        ringHeight = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "ringHeight");
    }

    /**
     * Updates the light patterns during the switch from autonomous to teleop.
     * @param autoDrive Tells if the autonomous drivetrain capabilities are enabled.
     */
    public void initPhaseLights(boolean autoDrive){
        if (autoDrive){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
        }else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        }
    }

    /**
     * Sets the individual pattern for the lights.
     * @param pattern The intended light pattern.
     */
    public void setLightPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        lights.setPattern(pattern);
    }

    /**
     * Sets the lights to the proper alliance color.
     * @param alliance Enum for the alliance.
     */
    public void allianceLights(@NotNull Alliance alliance){
        switch (alliance){
            case RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                break;
        }
    }

    public void updateEndgameTimer(ElapsedTime timer, Alliance alliance, boolean tilted){
        switch (determineGameTime(timer)){
            case DRIVER_CONTROLLED:
                if (tilted){
                    switch (alliance){
                        case BLUE:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                            break;
                        case RED:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
                            break;
                    }
                }else {
                    switch (determineRingHopperHeight()) {
                        case ZERO_RINGS:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                            break;
                        case ONE_RING:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                            break;
                        case TWO_RINGS:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                            break;
                        case THREE_RINGS:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            break;
                    }
                }
                break;
            case ENDGAME:
                if (tilted){
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
                }else {
                    switch (determineRingHopperHeight()) {
                        case ZERO_RINGS:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                            break;
                        case ONE_RING:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
                            break;
                        case TWO_RINGS:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                            break;
                        case THREE_RINGS:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                            break;
                    }
                }
                break;
            case LAST_5_SECONDS:
                switch (alliance){
                    case BLUE:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                        break;
                    case RED:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                        break;
                }
                break;
            case GAME_OVER:
                switch (alliance){
                    case BLUE:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                        break;
                    case RED:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                        break;
                }
                break;
        }
    }

    public HopperRings determineRingHopperHeight(){
        if (ringHeight.getDistance(DistanceUnit.INCH) < threeRingThreshold){
            return HopperRings.THREE_RINGS;
        }else if (ringHeight.getDistance(DistanceUnit.INCH) < twoRingThreshold){
            return HopperRings.TWO_RINGS;
        }else if (ringHeight.getDistance(DistanceUnit.INCH) < oneRingThreshold){
            return HopperRings.ONE_RING;
        }else {
            return HopperRings.ZERO_RINGS;
        }
    }

    private GameTime determineGameTime(@NotNull ElapsedTime timer){
        if (timer.seconds() > 90 && timer.seconds() <= 115){
            return GameTime.ENDGAME;
        }else if (timer.seconds() > 115 && timer.seconds() <= 120){
            return GameTime.LAST_5_SECONDS;
        }else if (timer.seconds() > 120 && timer.seconds() <= 122){
            return GameTime.GAME_OVER;
        }else {
            return GameTime.DRIVER_CONTROLLED;
        }
    }
}
