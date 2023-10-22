package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Drivetrain;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.CustomHardware.EncoderV2;

import java.util.Arrays;
import java.util.List;

public class LocalizationV2 extends ThreeTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 30/25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6; // in; offset of the lateral wheel

    private EncoderV2 leftEncoder, rightEncoder, frontEncoder;

    public LocalizationV2(HardwareMap hardwareMap){
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET,  0, Math.toRadians(90)) // front
        ));

        leftEncoder = new EncoderV2(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        rightEncoder = new EncoderV2(hardwareMap.get(DcMotorEx.class, "actuator"));
        frontEncoder = new EncoderV2(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        rightEncoder.setDirection(EncoderV2.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

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
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
