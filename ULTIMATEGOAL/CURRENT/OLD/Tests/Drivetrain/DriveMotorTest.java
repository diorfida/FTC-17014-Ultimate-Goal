package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Tests.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;

/**
 * Test that runs the drive motors to determine the proper direction and motor functionality,
 * @author Domenic Iorfida, FTC 17014
 */
@Disabled
@TeleOp(group = "test")
public class  DriveMotorTest extends LinearOpMode {

    DrivetrianHardware fierceBot;

    @Override
    public void runOpMode() throws InterruptedException {
        fierceBot = new DrivetrianHardware(this);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Powering the Front Left wheel");
        telemetry.update();
        fierceBot.setMotorPowers(1,0,0,0);
        sleep(5000);
        stopMotors();
        telemetry.addLine("Stopping...");
        telemetry.update();
        sleep(1000);

        telemetry.addLine("Powering the Front Right wheel");
        telemetry.update();
        fierceBot.setMotorPowers(0,0,0,1);
        sleep(5000);
        stopMotors();
        telemetry.addLine("Stopping...");
        telemetry.update();
        sleep(1000);

        telemetry.addLine("Powering the Back Left wheel");
        telemetry.update();
        fierceBot.setMotorPowers(0,1,0,0);
        sleep(5000);
        stopMotors();
        telemetry.addLine("Stopping...");
        telemetry.update();
        sleep(1000);

        telemetry.addLine("Powering the Back Right wheel");
        telemetry.update();
        fierceBot.setMotorPowers(0,0,1,0);
        sleep(5000);
        stopMotors();
        telemetry.addLine("Stopping...");
        telemetry.update();
        sleep(1000);

    }
    private void stopMotors(){
        fierceBot.setMotorPowers(0,0,0,0);
    }
}
