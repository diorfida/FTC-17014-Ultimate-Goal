package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2;

import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Data class that houses information about the field.
 * @author Domenic Iorfida, FTC 17014
 */
public class FieldSpecificationsV2 {
    // Note: none of these field dimensions include an adjustment for where our launcher is located.
    // That's what the LAUNCHER_CIRCLE_RADIUS is for.

    public static final double LAUNCHER_CIRCLE_RADIUS = 157.25/25.4; //157.25 154

    public static final Vector2d RED_TOWER_GOAL = new Vector2d(72, -36);

    public static final Vector2d RED_RIGHT_POWER_SHOT_TELE = new Vector2d(72.5, -16.5); //-18.75
    public static final Vector2d RED_LEFT_POWER_SHOT_TELE = new Vector2d(72.5, -7.5); //-1.75

    public static final Vector2d RED_RIGHT_POWER_SHOT_AUTO = new Vector2d(72.5, -16.75);
    public static final Vector2d RED_LEFT_POWER_SHOT_AUTO = new Vector2d(72.5, -10);
}
