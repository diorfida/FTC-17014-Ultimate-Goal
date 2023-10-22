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
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Launcher;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.TextFileMaster;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision.RingDetector;

import java.io.File;

@Disabled
@Autonomous(name = "Shoot preloaded rings + Single wobble (outer blue)", group = "blue")
public class ShootPreloadsSingleWobbleOuterBlue extends LinearOpMode {

    private final Alliance alliance = Alliance.BLUE;

    private TargetWobbleZone wobbleZone;

    private final Pose2d startPose = new Pose2d(-63, 50, 0);

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

        Trajectory readyToShoot = fierceBot.trajectoryBuilder(startPose)
                .forward(60.0)
                .build();

        ////////////////////////////////////

        Trajectory deliver1stGoalA = fierceBot.trajectoryBuilder(readyToShoot.end())
                .lineTo(new Vector2d(26.0, 60.0))
                .build();

        Trajectory parkItA = fierceBot.trajectoryBuilder(deliver1stGoalA.end())
                .forward(1.0)
                .splineToLinearHeading(new Pose2d(10.0, 29.0, 0.0), Math.toRadians(180.0))
                .build();

        //////////////////////////////

        Trajectory deliver1stGoalB = fierceBot.trajectoryBuilder(readyToShoot.end())
                .splineTo(new Vector2d(45.0, 37.0), 0.0)
                .build();

        Trajectory parkItB = fierceBot.trajectoryBuilder(deliver1stGoalB.end())
                .strafeLeft(10.0)
                .splineToLinearHeading(new Pose2d(10.0, 61.0, 0.0), Math.toRadians(180.0))
                .build();

        //////////////////////////////////////

        Trajectory deliver1stGoalC = fierceBot.trajectoryBuilder(readyToShoot.end())
                .lineToLinearHeading(new Pose2d(55.0, 45.5, Math.toRadians(270.0)))
                .build();

        Trajectory parkItC = fierceBot.trajectoryBuilder(deliver1stGoalC.end())
                .splineTo(new Vector2d(10.0, 59.0), Math.toRadians(180.0))
                .build();

        lights.setLightPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        while (!isStarted() && !isStopRequested()){
            telemetry.addLine("REMINDER: The robot should be starting on the outer starting line facing forwards.");
            telemetry.addLine();
            telemetry.addLine("OpMode initialized! Scanning rings...");
            wobbleZone = detector.scan();
            telemetry.update();
        }

        detector.terminate();

        waitForStart();

        lights.allianceLights(alliance);

        fierceBot.followTrajectory(readyToShoot);
        helper.tiltUp(true);
        helper.startFlywheel(Launcher.LaunchTarget.TOWER_GOAL);

        //helper.tempTurnFunction(BLUE_TOWER_GOAL);

        sleep(100);
        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.tiltResetUpDown();

        sleep(200);
        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.tiltResetUpDown();

        sleep(250);
        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.stopFlywheel();
        helper.tiltDown();

        helper.faceForward();

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
