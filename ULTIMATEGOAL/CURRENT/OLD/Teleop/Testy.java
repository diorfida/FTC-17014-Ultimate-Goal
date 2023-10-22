package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;

@Disabled
@TeleOp
public class Testy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrianHardware fierceBot = new DrivetrianHardware(this);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            fierceBot.update();
            telemetry.addData("Heading:", Math.toDegrees(fierceBot.getPoseEstimate().getHeading()));
            telemetry.update();
        }
    }
}
