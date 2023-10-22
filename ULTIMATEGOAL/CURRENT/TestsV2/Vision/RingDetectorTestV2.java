package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.TestsV2.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Enums.TargetWobbleZoneV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.RingDetectorV2;

//@Disabled
@TeleOp(group = "test")
public class RingDetectorTestV2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RingDetectorV2 detector = new RingDetectorV2(this);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            TargetWobbleZoneV2 zone = detector.scan();
            telemetry.update();
        }
        detector.terminate();
    }
}
