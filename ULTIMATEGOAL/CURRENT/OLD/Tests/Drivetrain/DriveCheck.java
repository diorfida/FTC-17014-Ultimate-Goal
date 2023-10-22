package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Tests.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(group = "test")
public class DriveCheck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft, frontRight, backLeft, backRight;

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
