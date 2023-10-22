package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Autonomous.BlueAlliance;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.AutoCorollary;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.TargetWobbleZone;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.LEDLights;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.TextFileMaster;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision.RingDetector;

import java.io.File;

/**
 * Moves one wobble goal into the scoring zone for the red alliance.
 * @author Domenic Iorfida, FTC 17014
 */
@Disabled
@Autonomous(name = "Single Wobble Goal Outer Blue", group = "blue")
public class SingleWobbleGoalOuterBlue extends LinearOpMode {

    private Alliance alliance = Alliance.BLUE;

    private TargetWobbleZone wobbleZone;

    private Pose2d startPose = new Pose2d(-63, 50, 0);

    private File lastX = AppUtil.getInstance().getSettingsFile("XFile.txt");
    private File lastY = AppUtil.getInstance().getSettingsFile("YFile.txt");
    private File lastHeading = AppUtil.getInstance().getSettingsFile("HeadingFile.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrianHardware fierceBot = new DrivetrianHardware(this);
        TextFileMaster textFiles = new TextFileMaster(fierceBot);
        LEDLights lights = new LEDLights(this);
        RingDetector detector = new RingDetector(this);
        AutoCorollary helper = new AutoCorollary(fierceBot, this, alliance);

        textFiles.checkTextFiles(lastX, lastY, lastHeading);
        fierceBot.setPoseEstimate(startPose);

        /////////////////////////////////////

        Trajectory deliver1stGoalA = fierceBot.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(26.0, 60.0))
                .build();

        Trajectory parkItA = fierceBot.trajectoryBuilder(deliver1stGoalA.end())
                .forward(1.0)
                .splineToLinearHeading(new Pose2d(10.0, 29.0, 0.0), Math.toRadians(180.0))
                .build();

        //////////////////////////////

        Trajectory deliver1stGoalB = fierceBot.trajectoryBuilder(startPose)
                .forward(30.0)
                .splineTo(new Vector2d(45.0, 37.0), 0.0)
                .build();

        Trajectory parkItB = fierceBot.trajectoryBuilder(deliver1stGoalB.end())
                .strafeLeft(10.0)
                .splineToLinearHeading(new Pose2d(10.0, 61.0, 0.0), Math.toRadians(180.0))
                .build();

        //////////////////////////////////////

        Trajectory deliver1stGoalC = fierceBot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(35.0, 55.0), 0.0)
                .splineTo(new Vector2d(55.0, 45.5), Math.toRadians(270.0))
                .build();

        Trajectory parkItC = fierceBot.trajectoryBuilder(deliver1stGoalC.end())
                .splineTo(new Vector2d(10.0, 59.0), Math.toRadians(180.0))
                .build();

        lights.setLightPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        while (!isStarted()){
            telemetry.addLine("REMINDER: The robot should be starting on the outer starting line.");
            telemetry.addLine();
            telemetry.addLine("OpMode initialized! Scanning rings...");
            wobbleZone = detector.scan();
            telemetry.update();
        }

        detector.terminate();

        waitForStart();

        lights.allianceLights(alliance);

        switch (wobbleZone){
            case ZONE_A:
                fierceBot.followTrajectory(deliver1stGoalA);
                helper.releaseWobbleGoal();

                fierceBot.followTrajectory(parkItA);
                break;
            case ZONE_B:
                fierceBot.followTrajectory(deliver1stGoalB);
                helper.releaseWobbleGoal();

                fierceBot.followTrajectory(parkItB);
                break;
            case ZONE_C:
                fierceBot.followTrajectory(deliver1stGoalC);
                helper.releaseWobbleGoal();

                fierceBot.followTrajectory(parkItC);
                break;
        }

        // TERMINATION PHASE
        textFiles.updateTextFiles(lastX, lastY, lastHeading);
    }
}
