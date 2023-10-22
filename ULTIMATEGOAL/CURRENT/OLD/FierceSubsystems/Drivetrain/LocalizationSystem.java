package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.CustomHardware.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/**
 * Localization wheel class.
 * @author Domenic Iorfida, FTC 17014
 */
@Config
public class LocalizationSystem extends ThreeTrackingWheelLocalizer {
    /**
     * Ticks per revolution of the localization encoder.
     */
    public static double TICKS_PER_REV = 8192;
    /**
     * The wheel radius for the localization wheels.
     */
    public static double WHEEL_RADIUS = 30/25.4; // in
    /**
     * The gear ratio for the encoder wheels, if any.
     */
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    /**
     * The distance between the left and right localization wheels.
     */
    public static double LATERAL_DISTANCE = 13.5; // in; distance between the left and right wheels
    /**
     * The distance of the forward localization wheel from the robot center.
     */
    public static double FORWARD_OFFSET = -6; // in; offset of the lateral wheel // -7.75

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    /**
     * Constructor: Sets the poses of the tracking wheels and initializes the encoders.
     * @param hardwareMap The hardware map to be imported from an FTC opMode.
     */
    public LocalizationSystem(@NotNull HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET,  0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "actuator"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    /**
     * Method to convert motor encoder tick counts to inches.
     * @param ticks Ticks per revolution of the encoder output shaft for the motor.
     * @return How many ticks are in one wheel revolution.
     */
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    /**
     * Places the localization encoders in a list.
     * @return The list of the localization encoders.
     */
    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        // competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        // compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
