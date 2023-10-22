package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Drivetrain;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class DriveConstantsV2 {

    public static final double TICKS_PER_REV = 28;
    public static final double MAX_RPM = 6000;


    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0, 10, 13.48); //getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV)


    public static double WHEEL_RADIUS = (75.0/2)/25.4; // in
    public static double GEAR_RATIO = (1/19.2)*1.4; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 12.58; // in


    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;


    public static double MAX_VEL = 45;
    public static double MAX_ACCEL = 37.5;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
