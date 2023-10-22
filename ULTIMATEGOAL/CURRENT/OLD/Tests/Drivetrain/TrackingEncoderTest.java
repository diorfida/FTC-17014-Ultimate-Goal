package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Tests.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.LocalizationSystem;

import java.util.List;

@Disabled
@TeleOp(group = "test")
public class TrackingEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LocalizationSystem localizer = new LocalizationSystem(hardwareMap);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            List<Double> wheelPositions = localizer.getWheelPositions();

            telemetry.addData("Left Position:", wheelPositions.get(0));
            telemetry.addData("Right Position:", wheelPositions.get(1));
            telemetry.addData("Front Position:", wheelPositions.get(2));
            telemetry.update();
        }
    }
}
