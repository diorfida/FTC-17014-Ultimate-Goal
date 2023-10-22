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
 * Moves both wobble goals into the scoring zone for the red alliance.
 * @author Domenic Iorfida, FTC 17014
 */
@Disabled
@Deprecated
@Autonomous(name = "Double Wobble Goal Outer Blue", group = "blue")
public class DoubleWobbleGoalOuterBlue extends LinearOpMode {

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

        Trajectory get2ndGoalA = fierceBot.trajectoryBuilder(deliver1stGoalA.end())
                .strafeRight(7.5)
                .addDisplacementMarker(helper::swingClawOut)
                .splineToLinearHeading(new Pose2d(-36.5, 20.5, 0.0), Math.toRadians(180.0))
                .build();

        Trajectory deliver2ndGoalA = fierceBot.trajectoryBuilder(get2ndGoalA.end())
                .lineToLinearHeading(new Pose2d(11.3, 49.5, Math.toRadians(270.0)))
                .build();

        Trajectory parkItA = fierceBot.trajectoryBuilder(deliver2ndGoalA.end())
                .lineTo(new Vector2d(10.0, 39.0))
                .build();

        //////////////////////////////

        Trajectory deliver1stGoalB = fierceBot.trajectoryBuilder(startPose)
                .forward(30.0)
                .splineTo(new Vector2d(45.0, 37.0), 0.0)
                .build();

        Trajectory get2ndGoalB = fierceBot.trajectoryBuilder(deliver1stGoalB.end())
                .strafeRight(5.0)
                .addDisplacementMarker(helper::swingClawOut)
                .splineToConstantHeading(new Vector2d(-20.0, 25.0), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(-36.25, 20.5), Math.toRadians(180.0))
                .build();

        Trajectory deliver2ndGoalB = fierceBot.trajectoryBuilder(get2ndGoalB.end())
                .lineToLinearHeading(new Pose2d(23.0, 30.5, Math.toRadians(180.0)))
                .build();

        Trajectory parkItB = fierceBot.trajectoryBuilder(deliver2ndGoalB.end())
                .forward(10.0)
                .build();

        //////////////////////////////////////

        Trajectory deliver1stGoalC = fierceBot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(35.0, 55.0), 0.0)
                .splineTo(new Vector2d(55.0, 45.5), Math.toRadians(270.0))
                .build();

        Trajectory get2ndGoalC1 = fierceBot.trajectoryBuilder(deliver1stGoalC.end())
                .splineToLinearHeading(new Pose2d(35.0, 30.0, 0.0), Math.toRadians(180.0))
                .build();

        Trajectory get2ndGoalC2 = fierceBot.trajectoryBuilder(get2ndGoalC1.end())
                .back(1.0)
                .addDisplacementMarker(helper::swingClawOut)
                .splineToConstantHeading(new Vector2d(-20.0, 25.0), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(-36.25, 20.5), Math.toRadians(180.0))
                .build();

        Trajectory deliver2ndGoalC = fierceBot.trajectoryBuilder(get2ndGoalC2.end())
                .lineToLinearHeading(new Pose2d(61.0, 47.0, Math.toRadians(270.0)))
                .build();

        Trajectory parkItC = fierceBot.trajectoryBuilder(deliver2ndGoalC.end())
                .lineTo(new Vector2d(10.0, 39.0))
                .build();

        lights.setLightPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        while (!isStarted()){
            telemetry.addLine("REMINDER: The robot should be starting on the outer starting line facing forwards.");
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
                fierceBot.followTrajectory(get2ndGoalA);
                helper.grabWobbleGoal();

                fierceBot.followTrajectory(deliver2ndGoalA);
                helper.swingClawOut();
                sleep(250);
                helper.releaseWobbleGoal();
                fierceBot.followTrajectory(parkItA);
                break;
            case ZONE_B:
                fierceBot.followTrajectory(deliver1stGoalB);
                helper.releaseWobbleGoal();
                fierceBot.followTrajectory(get2ndGoalB);
                helper.grabWobbleGoal();

                fierceBot.followTrajectory(deliver2ndGoalB);
                helper.swingClawOut();
                sleep(350);
                helper.releaseWobbleGoal();

                fierceBot.followTrajectory(parkItB);
                break;
            case ZONE_C:
                fierceBot.followTrajectory(deliver1stGoalC);
                helper.releaseWobbleGoal();
                fierceBot.followTrajectory(get2ndGoalC1);
                fierceBot.followTrajectory(get2ndGoalC2);
                helper.grabWobbleGoal();

                fierceBot.followTrajectory(deliver2ndGoalC);
                helper.swingClawOut();
                sleep(350);
                helper.releaseWobbleGoal();
                fierceBot.followTrajectory(parkItC);
                break;
        }

        // TERMINATION PHASE
        textFiles.updateTextFiles(lastX, lastY, lastHeading);
    }
}
