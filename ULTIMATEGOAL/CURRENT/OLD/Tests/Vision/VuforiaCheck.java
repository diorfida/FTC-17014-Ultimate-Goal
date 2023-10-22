package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Tests.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision.NavigationCorrection;

@Disabled
@TeleOp(group = "tests")
public class VuforiaCheck extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrianHardware fierceBot = new DrivetrianHardware(this);
        NavigationCorrection correction = new NavigationCorrection(this, fierceBot, Alliance.RED);

        waitForStart();

        correction.start();

        while (opModeIsActive()){
            correction.run();
            telemetry.update();
        }

    }
}
