package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * The declaration of variables necessary for autonomous movements.
 * @author Domenic Iorfida, FTC 17014
 */
@Config
public class DrivetrainConstraints {

    /**
     * Ticks per revolution of the drivetrain motor encoders output shaft.
     */
    public static final double TICKS_PER_REV = 28;
    /**
     * The maximum revolutions per minute for the drivetrain motors.
     */
    public static final double MAX_RPM = 6000;

    /**
     * Variable for setting the encoder mode for the drivetrain motors.
     */
    public static final boolean RUN_USING_ENCODER = true;
    /**
     * Drivetrain velocity PID coefficients.
     */
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20,0,10, 13.48);//getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV)

    /**
     * The wheel radius for the drivetrain wheels.
     */
    public static double WHEEL_RADIUS = (75.0/2)/25.4; // this is the REV mecanum wheel
    /**
     * The gear ratio for the drivetrain motors.
     * Note: this gear ratio is the wheel speed divided by the motor speed.
     */
    public static double GEAR_RATIO = (1/19.2)*1.4; // output (wheel) speed / input (motor) speed
    /**
     * The distance between the center points of the drivetrain wheels.
     * Note: this number starts out as a rough estimate and is tuned later.
     */
    public static double TRACK_WIDTH = 12.58;

    //Since we are using PID with motor encoders, these values are fine as they are
    /**
     * Feedforward parameter.
     * Note: variable that remains untouched if using PID with motor encoders.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    /**
     * Feedforward parameter.
     * Note: variable that remains untouched if using PID with motor encoders.
     */
    public static double kA = 0;
    /**
     * Feedforward parameter.
     * Note: variable that remains untouched if using PID with motor encoders.
     */
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    /**
     * The base constraints used to generate robot trajectories in autonomous.
     */
    public static double MAX_VEL = 45; //35
    public static double MAX_ACCEL = 37.5; //30
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    /**
     * Method to convert motor encoder tick counts to inches.
     * @param ticks Ticks per revolution of the encoder output shaft for the motor.
     * @return How many ticks are in one wheel revolution.
     */
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    /**
     * Method to convert RPM to velocity for power control.
     * @param rpm Maximum RPM for each motor.
     * @return Velocity
     */
    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    /**
     * Formula to obtain the velocity feedforward parameter.
     * @return Velocity
     */
    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    private static double getMaxVelocity(){
        return (MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * (2 * Math.PI);
    }

    private static double getMaxAngularVelocity(){
        return getMaxVelocity() / TRACK_WIDTH;
    }
}
