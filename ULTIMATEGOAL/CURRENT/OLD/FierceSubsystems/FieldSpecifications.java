package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Data class that houses information about the field.
 * @author Domenic Iorfida, FTC 17014
 */
public class FieldSpecifications {
    // Note: none of these field dimensions include an adjustment for where our launcher is located.
    // That's what the LAUNCHER_CIRCLE_RADIUS is for.

    public static final double LAUNCHER_CIRCLE_RADIUS = 157.25/25.4; //157.25 154

    public static final Vector2d RED_TOWER_GOAL = new Vector2d(72, -36);
    public static final Vector2d RED_RIGHT_POWER_SHOT = new Vector2d(72.5, -18.5); //-18.75
    public static final Vector2d RED_MIDDLE_POWER_SHOT = new Vector2d(72.5, -11); //-11.25
    public static final Vector2d RED_LEFT_POWER_SHOT = new Vector2d(72.5, -3.5); //-1.75

    public static final Vector2d RED_RIGHT_POWER_SHOT_TELE = new Vector2d(72.5, -5);
    public static final Vector2d RED_MIDDLE_POWER_SHOT_TELE = new Vector2d(72.5, 0);
    public static final Vector2d RED_LEFT_POWER_SHOT_TELE = new Vector2d(72.5, 8);

    public static final Vector2d BLUE_TOWER_GOAL = new Vector2d(72.5, 40);
    public static final Vector2d BLUE_RIGHT_POWER_SHOT = new Vector2d(72.5, 3.75);
    public static final Vector2d BLUE_MIDDLE_POWER_SHOT = new Vector2d(72.5, 11.25);
    public static final Vector2d BLUE_LEFT_POWER_SHOT = new Vector2d(72.5, 18.75);
}
