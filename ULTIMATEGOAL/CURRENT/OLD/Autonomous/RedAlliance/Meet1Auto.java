package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Autonomous.RedAlliance;

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
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.LEDLights;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Launcher;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.TextFileMaster;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision.RingDetector;

import java.io.File;

@Deprecated
@Disabled
@Autonomous(name = "Meet #1 Experiment Auto", group = "red")
public class Meet1Auto extends LinearOpMode {

    private final Alliance alliance = Alliance.RED;

    private TargetWobbleZone wobbleZone;

    private final Pose2d startPose = new Pose2d(-63, -50, 0);

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
                .lineTo(new Vector2d(26.0, -63.0))
                .build();

        Trajectory get2ndGoalA = fierceBot.trajectoryBuilder(deliver1stGoalA.end())
                .strafeLeft(7.5)
                .addDisplacementMarker(helper::swingClawOut)
                .splineToLinearHeading(new Pose2d(-36.5, -31.5, 0.0), Math.toRadians(180.0))
                .build();

        Trajectory shootPowerShot1A = fierceBot.trajectoryBuilder(get2ndGoalA.end())
                .lineTo(new Vector2d(0, -36 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS))
                .addTemporalMarker(2, () -> {
                    helper.startFlywheel(Launcher.LaunchTarget.POWER_SHOT);
                })
                .build();

//        Trajectory shootPowerShot2A = fierceBot.trajectoryBuilder(shootPowerShot1A.end())
//                .strafeTo(new Vector2d(-2.0, -11.25 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS))
//                .build();
//
//        Trajectory shootPowerShot3A = fierceBot.trajectoryBuilder(shootPowerShot2A.end())
//                .strafeTo(new Vector2d(-2.0, -18.75 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS))
//                .build();

        Trajectory deliver2ndGoalA = fierceBot.trajectoryBuilder(shootPowerShot1A.end())
                .lineToLinearHeading(new Pose2d(11.3, -49.5, Math.toRadians(90.0)))
                .build();

        Trajectory parkItA = fierceBot.trajectoryBuilder(deliver2ndGoalA.end())
                .lineTo(new Vector2d(10.0, -39.0))
                .build();

        //////////////////////////////

        Trajectory deliver1stGoalB = fierceBot.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(45.0, -47.0))
                .build();

        Trajectory get2ndGoalB = fierceBot.trajectoryBuilder(deliver1stGoalB.end())
                .strafeLeft(5.0)
                .addDisplacementMarker(helper::swingClawOut)
                .splineToConstantHeading(new Vector2d(-20.0, -25.0), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(-36.25, -32.5), Math.toRadians(180.0))
                .build();

        Trajectory shootPowerShot1B = fierceBot.trajectoryBuilder(get2ndGoalB.end())
                .lineTo(new Vector2d(0, -36 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS))
                .addTemporalMarker(2, () -> {
                    helper.startFlywheel(Launcher.LaunchTarget.POWER_SHOT);
                })
                .build();

//        Trajectory shootPowerShot2B = fierceBot.trajectoryBuilder(shootPowerShot1B.end())
//                .lineTo(new Vector2d(-2.0, -11.25 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS))
//                .build();
//
//        Trajectory shootPowerShot3B = fierceBot.trajectoryBuilder(shootPowerShot2B.end())
//                .lineTo(new Vector2d(-2.0, -18.75 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS))
//                .build();

        Trajectory deliver2ndGoalB = fierceBot.trajectoryBuilder(shootPowerShot1B.end())
                .lineToLinearHeading(new Pose2d(23.0, -30.5, Math.toRadians(180.0)))
                .build();

        Trajectory getStackB = fierceBot.trajectoryBuilder(deliver2ndGoalB.end())
                .lineTo(new Vector2d(-20.0, -37.0))
                .addTemporalMarker(.8, helper::startIntake)
                .build();

        Trajectory shootStack = fierceBot.trajectoryBuilder(getStackB.end())
                .lineToLinearHeading(new Pose2d(0.0, -36 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS, 0.0))
                .addTemporalMarker(1.75, () -> {
                    helper.startFlywheel(Launcher.LaunchTarget.TOWER_GOAL);
                })
                .build();

        Trajectory parkItB = fierceBot.trajectoryBuilder(shootStack.end())
                .forward(10.0)
                .build();

        //////////////////////////////////////

        Trajectory deliver1stGoalC = fierceBot.trajectoryBuilder(startPose)
                //.splineTo(new Vector2d(-20.0, -47.5), 0.0)
                .splineTo(new Vector2d(0.0, -50.0), 0.0)
                .splineTo(new Vector2d(55.0, -45.5), Math.toRadians(90.0))
                .build();

        Trajectory shootPowerShot1C = fierceBot.trajectoryBuilder(deliver1stGoalC.end())
                .lineToLinearHeading(new Pose2d(0, -36 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS, 0.0))
                .addTemporalMarker(2.75, () -> {
                    helper.startFlywheel(Launcher.LaunchTarget.POWER_SHOT);
                })
                .build();

//        Trajectory shootPowerShot2C = fierceBot.trajectoryBuilder(shootPowerShot1C.end())
//                .lineTo(new Vector2d(-2.0, -11.25 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS))
//                .build();
//
//        Trajectory shootPowerShot3C = fierceBot.trajectoryBuilder(shootPowerShot2C.end())
//                .lineTo(new Vector2d(-2.0, -3.75 + FieldSpecifications.LAUNCHER_CIRCLE_RADIUS))
//                .build();

        Trajectory get2ndGoalC = fierceBot.trajectoryBuilder(shootPowerShot1C.end())
                .lineTo(new Vector2d(-37.6, -32.5))
                .build();

        Trajectory deliver2ndGoalC = fierceBot.trajectoryBuilder(get2ndGoalC.end())
                .lineToLinearHeading(new Pose2d(61.0, -47.0, Math.toRadians(90.0)))
                .build();

        Trajectory parkItC = fierceBot.trajectoryBuilder(deliver2ndGoalC.end())
                .lineTo(new Vector2d(10.0, -39.0))
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

        switch (wobbleZone){
            case ZONE_A:
                fierceBot.followTrajectory(deliver1stGoalA);
                helper.releaseWobbleGoal();
                fierceBot.followTrajectory(get2ndGoalA);
                helper.grabWobbleGoal();

                fierceBot.followTrajectory(shootPowerShot1A);
                helper.tiltUp(true);
                sleep(100);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.tiltResetUpDown();

                sleep(100);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.tiltResetUpDown();

                sleep(250);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.stopFlywheel();
                helper.tiltDown();

                helper.faceForward();

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

                fierceBot.followTrajectory(shootPowerShot1B);
                helper.tiltUp(true);
                sleep(100);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.tiltResetUpDown();

                sleep(100);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.tiltResetUpDown();

                sleep(250);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.stopFlywheel();
                helper.tiltDown();

                fierceBot.followTrajectory(deliver2ndGoalB);
                helper.swingClawOut();
                sleep(500);
                helper.releaseWobbleGoal();

                fierceBot.followTrajectory(getStackB);

                fierceBot.followTrajectory(shootStack);
                helper.stopIntake();
                helper.tiltUp(false);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.stopFlywheel();
                helper.tiltDown();
//
                fierceBot.followTrajectory(parkItB);
                break;
            case ZONE_C:
                fierceBot.followTrajectory(deliver1stGoalC);
                helper.releaseWobbleGoal();
                helper.tiltUp(true);

                fierceBot.followTrajectory(shootPowerShot1C);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.tiltResetUpDown();

                sleep(300);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.tiltResetUpDown();

                sleep(300);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.stopFlywheel();
                helper.tiltDown();
                helper.swingClawOut();

                fierceBot.followTrajectory(get2ndGoalC);
                helper.grabWobbleGoal();

                fierceBot.followTrajectory(deliver2ndGoalC);
                helper.releaseWobbleGoal();
                fierceBot.followTrajectory(parkItC);
                break;
        }

        // TERMINATION PHASE
        textFiles.updateTextFiles(lastX, lastY, lastHeading);
    }
}
