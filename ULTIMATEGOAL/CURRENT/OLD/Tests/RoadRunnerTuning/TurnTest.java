package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Tests.RoadRunnerTuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;

/*
 * This is a simple routine to test turning capabilities.
 */
@Disabled
//@Config
@Autonomous(group = "tuning")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrianHardware drive = new DrivetrianHardware(this);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
