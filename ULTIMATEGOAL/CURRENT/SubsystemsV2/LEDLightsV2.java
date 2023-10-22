package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Enums.AllianceV2;
import org.jetbrains.annotations.NotNull;

public class LEDLightsV2 extends HopperRingsV2 {

    private enum GameTimeV2{
        DRIVER_CONTROLLED,
        FIVE_UNTIL_ENDGAME,
        ENDGAME,
        LAST_5_SECONDS,
        GAME_OVER
    }

    private RevBlinkinLedDriver lights;

    public LEDLightsV2(@NotNull HardwareMap hardwareMap){
        super(hardwareMap);
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }

    public void initPhaseLights(){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
    }

    public void setLightPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        lights.setPattern(pattern);
    }

    public void allianceLights(@NotNull AllianceV2 alliance){
        switch (alliance){
            case RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                break;
        }
    }

    private GameTimeV2 determineGameTime(@NotNull ElapsedTime timer){
        if (timer.seconds() > 85 && timer.seconds() <= 90)
            return GameTimeV2.FIVE_UNTIL_ENDGAME;
        else if (timer.seconds() > 90 && timer.seconds() <= 115)
            return GameTimeV2.ENDGAME;
        else if (timer.seconds() > 115 && timer.seconds() <= 120)
            return GameTimeV2.LAST_5_SECONDS;
        else if (timer.seconds() > 120 && timer.seconds() <= 122)
            return GameTimeV2.GAME_OVER;
        else
            return GameTimeV2.DRIVER_CONTROLLED;
    }

    public void updateEndgameTimer(ElapsedTime timer, AllianceV2 alliance, boolean tilted){
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
                            switch (alliance){
                                case BLUE:
                                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                                    break;
                                case RED:
                                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                                    break;
                            }
                            break;
                        case ONE_RING:
                            switch (alliance){
                                case BLUE:
                                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                                    break;
                                case RED:
                                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                                    break;
                            }
                            break;
                        case TWO_RINGS:
                            switch (alliance){
                                case BLUE:
                                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                                    break;
                                case RED:
                                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                                    break;
                            }
                            break;
                        case THREE_RINGS:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            break;
                    }
                }
                break;
            case FIVE_UNTIL_ENDGAME:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                break;
            case ENDGAME:
                if (tilted){
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
                }else {
                    switch (determineRingHopperHeight()) {
                        case ZERO_RINGS:
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
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
}
