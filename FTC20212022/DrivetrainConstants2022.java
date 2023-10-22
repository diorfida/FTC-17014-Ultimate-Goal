package org.firstinspires.ftc.teamcode.FTC20212022;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DrivetrainConstants2022 {

    private static final double TICKS_PER_REV = 28;
    private static final double MAX_RPM = 6000;

    // TODO: FILL THESE IN
    public static double WHEEL_RADIUS = 1;
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 1;

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    public static double MAX_VEL = 45;
    public static double MAX_ACCEL = 37.5;
    public static double MAX_ANG_VEL = Math.toRadians(180.0);
    public static double MAX_ANG_ACCEL = Math.toRadians(180.0);

    ///////////////////////////////////////////////////////////////////////////

    public static double encoderTicksToInches(double ticks)
    {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm)
    {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond)
    {
        return 32767 / ticksPerSecond;
    }
}
