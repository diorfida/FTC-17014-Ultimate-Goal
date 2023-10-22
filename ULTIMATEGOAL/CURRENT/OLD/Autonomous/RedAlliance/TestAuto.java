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
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.LEDLights;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Launcher;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.TextFileMaster;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision.RingDetector;

import java.io.File;

import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.LAUNCHER_CIRCLE_RADIUS;
import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.RED_LEFT_POWER_SHOT;
import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.RED_MIDDLE_POWER_SHOT;
import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.RED_RIGHT_POWER_SHOT;

@Disabled
@Autonomous(name = "Test Auto", group = "red")
public class TestAuto extends LinearOpMode {

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

        Trajectory readyToShoot = fierceBot.trajectoryBuilder(startPose)
                .forward(60.0)
                .splineToConstantHeading(new Vector2d(10.0, -36.0), Math.toRadians(90.0))
                //.addDisplacementMarker(() -> helper.startFlywheel(Launcher.LaunchTarget.LEFT_POWER_SHOT))
                .splineToConstantHeading(new Vector2d(-2, -40 + LAUNCHER_CIRCLE_RADIUS), Math.toRadians(180.0))
                .build();

        ////////////////////////////////////

        Trajectory deliver1stGoalA = fierceBot.trajectoryBuilder(readyToShoot.end())
                .lineToLinearHeading(new Pose2d(26.0, -60.0, Math.toRadians(45.0)))
                .build();

        Trajectory get2ndGoalA = fierceBot.trajectoryBuilder(deliver1stGoalA.end())
                .addTemporalMarker(1.5, helper::swingClawOut)
                .splineToLinearHeading(new Pose2d(-35.25, -30.9, 0.0), Math.toRadians(180.0))
                .build();

        Trajectory deliver2ndGoalA = fierceBot.trajectoryBuilder(get2ndGoalA.end())
                .lineToLinearHeading(new Pose2d(11.3, -49.5, Math.toRadians(90.0)))
                .build();

        Trajectory parkItA = fierceBot.trajectoryBuilder(deliver2ndGoalA.end())
                .lineTo(new Vector2d(10.0, -39.0))
                .build();

        //////////////////////////////

        Trajectory deliver1stGoalB = fierceBot.trajectoryBuilder(readyToShoot.end())
                .lineTo(new Vector2d(45.0, -47.0))
                .build();

        Trajectory get2ndGoalB = fierceBot.trajectoryBuilder(deliver1stGoalB.end())
                .strafeLeft(5.0)
                .addDisplacementMarker(helper::swingClawOut)
                .splineToConstantHeading(new Vector2d(-20.0, -25.0), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(-36.0, -31.75), Math.toRadians(180.0))
                .build();

        Trajectory getStackB = fierceBot.trajectoryBuilder(get2ndGoalB.end())
                .splineToConstantHeading(new Vector2d(-36.0, -36.0), Math.toRadians(270.0))
                .splineToConstantHeading(new Vector2d(-16.0, -40.5), 0.0)
                .addTemporalMarker(2, () -> helper.startFlywheel(Launcher.LaunchTarget.TOWER_GOAL))
                .splineToConstantHeading(new Vector2d(-1.0, -43 + LAUNCHER_CIRCLE_RADIUS), 0.0)
                .addTemporalMarker(.5, helper::startIntake)
                .build();

        Trajectory deliver2ndGoalB = fierceBot.trajectoryBuilder(getStackB.end())
                .lineToLinearHeading(new Pose2d(23.0, -30.5, Math.toRadians(180.0)))
                .build();

        Trajectory parkItB = fierceBot.trajectoryBuilder(deliver2ndGoalB.end())
                .forward(15.0)
                .build();

        //////////////////////////////////////

        Trajectory deliver1stGoalC = fierceBot.trajectoryBuilder(readyToShoot.end())
                .lineToLinearHeading(new Pose2d(55.0, -55.5, Math.toRadians(180-45.0)))
                .build();

        Trajectory get2ndGoalC1 = fierceBot.trajectoryBuilder(deliver1stGoalC.end())
                .splineToLinearHeading(new Pose2d(35.0, -30.0, 0.0), Math.toRadians(180.0))
                .build();

        Trajectory get2ndGoalC2 = fierceBot.trajectoryBuilder(get2ndGoalC1.end())
                .back(1.0)
                .addDisplacementMarker(helper::swingClawOut)
                .splineToConstantHeading(new Vector2d(-20.0, -25.0), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(-35.0, -29.5), Math.toRadians(180.0))
                .build();

        Trajectory getStackC = fierceBot.trajectoryBuilder(get2ndGoalC2.end())
                .splineToConstantHeading(new Vector2d(-36.0, -36.0), Math.toRadians(270.0))
                .splineToConstantHeading(new Vector2d(-16.0, -40.5), 0.0)
                .addTemporalMarker(2, () -> helper.startFlywheel(Launcher.LaunchTarget.TOWER_GOAL))
                .splineToConstantHeading(new Vector2d(-1.0, -43 + LAUNCHER_CIRCLE_RADIUS), 0.0)
                .addTemporalMarker(.5, helper::startIntake)
                .build();

        Trajectory deliver2ndGoalC = fierceBot.trajectoryBuilder(getStackC.end())
                .lineToLinearHeading(new Pose2d(57.0, -51.0, Math.toRadians(90.0)))
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

        fierceBot.followTrajectory(readyToShoot);
        helper.tiltUp(false);
        helper.turnTo(RED_RIGHT_POWER_SHOT);

        sleep(350);
        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        //helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        //helper.tiltResetUpDown();
        //helper.tiltUp(false);

        helper.turnTo(RED_MIDDLE_POWER_SHOT);

        //sleep(200);
        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.turnTo(RED_LEFT_POWER_SHOT);

        //sleep(250);
        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.stopFlywheel();
        helper.tiltDown();
        helper.faceForward();

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

                fierceBot.followTrajectory(getStackB);
                helper.stopIntake();
                switch (lights.determineRingHopperHeight()){
                    case ZERO_RINGS:
                        helper.stopFlywheel();
                        break;
                    case ONE_RING:
                        helper.tiltUp(true);
                        sleep(350);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.stopFlywheel();
                        helper.tiltDown();
                        break;
                    case TWO_RINGS:
                        helper.tiltUp(true);
                        sleep(350);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.stopFlywheel();
                        helper.tiltDown();
                        break;
                    case THREE_RINGS:
                        helper.tiltUp(true);
                        sleep(350);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.stopFlywheel();
                        helper.tiltDown();
                        break;
                }
                fierceBot.followTrajectory(deliver2ndGoalB);
                helper.swingClawOut();
                sleep(100);
                helper.releaseWobbleGoal();

                fierceBot.followTrajectory(parkItB);
                break;
            case ZONE_C:
                fierceBot.followTrajectory(deliver1stGoalC);
                helper.releaseWobbleGoal();
                fierceBot.followTrajectory(get2ndGoalC1);
                fierceBot.followTrajectory(get2ndGoalC2);
                helper.grabWobbleGoal();

                fierceBot.followTrajectory(getStackC);
                helper.stopIntake();
                switch (lights.determineRingHopperHeight()){
                    case ZERO_RINGS:
                        helper.stopFlywheel();
                        break;
                    case ONE_RING:
                        helper.tiltUp(true);
                        sleep(350);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.stopFlywheel();
                        helper.tiltDown();
                        break;
                    case TWO_RINGS:
                        helper.tiltUp(true);
                        sleep(350);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.stopFlywheel();
                        helper.tiltDown();
                        break;
                    case THREE_RINGS:
                        helper.tiltUp(true);
                        sleep(350);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                        helper.stopFlywheel();
                        helper.tiltDown();
                        break;
                }

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
