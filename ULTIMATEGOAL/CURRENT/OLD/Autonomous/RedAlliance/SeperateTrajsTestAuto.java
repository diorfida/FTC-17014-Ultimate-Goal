package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Autonomous.RedAlliance;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.AutoCorollary;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrainConstraints;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.TargetWobbleZone;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.LEDLights;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Launcher;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.TextFileMaster;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision.RingDetector;

import java.io.File;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.LAUNCHER_CIRCLE_RADIUS;
import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.RED_MIDDLE_POWER_SHOT;
import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.RED_RIGHT_POWER_SHOT;

@Disabled
@Autonomous(group = "red")
public class SeperateTrajsTestAuto extends LinearOpMode {

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
                .forward(43.0)
                .splineTo(new Vector2d(-2, -19), 0.0)
                .build();

        //////////////////////////////////////

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
        helper.startFlywheel(Launcher.LaunchTarget.TOWER_GOAL);

        telemetry.addData("Pose:", fierceBot.getPoseEstimate().toString());
        telemetry.update();
        helper.tiltUp(false);
        //helper.turnTo(RED_RIGHT_POWER_SHOT);

        sleep(450);
        //helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);

        helper.turnTo(RED_RIGHT_POWER_SHOT);

        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.turnTo(RED_MIDDLE_POWER_SHOT);

        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
        helper.stopFlywheel();
        helper.tiltDown();
        fierceBot.updatePoseEstimate();
//        helper.faceForward();

        switch (wobbleZone){
            case ZONE_A:
                Trajectory deliver1stGoalA = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(26.0, -60.0, Math.toRadians(45.0)))
                        .build();
                fierceBot.followTrajectory(deliver1stGoalA);

                helper.releaseWobbleGoal();

                Trajectory get2ndGoalA = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .addTemporalMarker(1.5, helper::swingClawOut)
                        .splineToLinearHeading(new Pose2d(-36.05, -29.25, 0.0), Math.toRadians(180.0),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DrivetrainConstraints.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(35, DrivetrainConstraints.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(30))
                        .build();
                fierceBot.followTrajectory(get2ndGoalA);

                helper.grabWobbleGoal();

                Trajectory deliver2ndGoalA = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(11.3, -49.5, Math.toRadians(90.0)))
                        .build();
                fierceBot.followTrajectory(deliver2ndGoalA);

//                helper.swingClawOut();
//                sleep(250);
                helper.releaseWobbleGoal();

                Trajectory parkItA = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .lineTo(new Vector2d(10.0, -39.0))
                        .build();
                fierceBot.followTrajectory(parkItA);

                break;
            case ZONE_B:
                Trajectory deliver1stGoalB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(45.0, -47.0, 0.0))
                        .build();
                fierceBot.followTrajectory(deliver1stGoalB);

                helper.releaseWobbleGoal();

                Trajectory get2ndGoalB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .strafeLeft(10.0)
                        .addDisplacementMarker(helper::swingClawOut)
                        .splineToConstantHeading(new Vector2d(-20.0, -25.0), Math.toRadians(180.0),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DrivetrainConstraints.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(35, DrivetrainConstraints.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(30))
                        .splineToConstantHeading(new Vector2d(-36, -29.5), Math.toRadians(180.0))
                        .build();
                fierceBot.followTrajectory(get2ndGoalB);

                helper.grabWobbleGoal();

                Trajectory getStackB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        //.splineToConstantHeading(new Vector2d(-36.0, -36.0), Math.toRadians(270.0))
                        .splineTo(new Vector2d(-16.0, -46.5), 0.0)
                        .addTemporalMarker(2, () -> helper.startFlywheel(Launcher.LaunchTarget.TOWER_GOAL))
                        .splineTo(new Vector2d(0, -45 + LAUNCHER_CIRCLE_RADIUS), 0.0)
                        .addTemporalMarker(.5, helper::startIntake)
                        .build();
                fierceBot.followTrajectory(getStackB);

                helper.swingClawUp();

                helper.stopIntake();
                helper.tiltUp(true);
                sleep(1500);
                helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
                helper.stopFlywheel();
                helper.tiltDown();

                Trajectory deliver2ndGoalB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(23.0, -30.5, Math.toRadians(180.0)))
                        .build();
                fierceBot.followTrajectory(deliver2ndGoalB);

                helper.swingClawOut();
                sleep(100);
                helper.releaseWobbleGoal();

                Trajectory parkItB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .forward(15.0)
                        .build();
                fierceBot.followTrajectory(parkItB);

                break;
            case ZONE_C:
                Trajectory deliver1stGoalC = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(55.0, -55.5, Math.toRadians(180-45.0)))
                        .build();
                fierceBot.followTrajectory(deliver1stGoalC);

                helper.releaseWobbleGoal();

                Trajectory get2ndGoalC1 = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(35.0, -30.0, 0.0), Math.toRadians(180.0))
                        .build();
                fierceBot.followTrajectory(get2ndGoalC1);

                Trajectory get2ndGoalC2 = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .back(1.0)
                        .addDisplacementMarker(helper::swingClawOut)
                        .splineToConstantHeading(new Vector2d(-20.0, -22.0), Math.toRadians(180.0))
                        .splineToConstantHeading(new Vector2d(-36, -29), Math.toRadians(180.0),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DrivetrainConstraints.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(35, DrivetrainConstraints.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(30))
                        .build();
                fierceBot.followTrajectory(get2ndGoalC2);

                helper.grabWobbleGoal();

//                Trajectory getStackC = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
//                        .splineToConstantHeading(new Vector2d(-36.0, -36.0), Math.toRadians(270.0))
//                        .splineToConstantHeading(new Vector2d(-16.0, -40.5), 0.0)
//                        .addTemporalMarker(2, () -> helper.startFlywheel(Launcher.LaunchTarget.TOWER_GOAL))
//                        .splineToConstantHeading(new Vector2d(-1.0, -43 + LAUNCHER_CIRCLE_RADIUS), 0.0)
//                        .addTemporalMarker(.5, helper::startIntake)
//                        .build();
//                fierceBot.followTrajectory(getStackC);
//
//                helper.stopIntake();
//                switch (lights.determineRingHopperHeight()){
//                    case ZERO_RINGS:
//                        helper.stopFlywheel();
//                        break;
//                    case ONE_RING:
//                        helper.tiltUp(true);
//                        sleep(400);
//                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
//                        helper.stopFlywheel();
//                        helper.tiltDown();
//                        break;
//                    case TWO_RINGS:
//                        helper.tiltUp(true);
//                        sleep(400);
//                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
//                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
//                        helper.stopFlywheel();
//                        helper.tiltDown();
//                        break;
//                    case THREE_RINGS:
//                        helper.tiltUp(true);
//                        sleep(400);
//                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
//                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
//                        helper.shoot(Launcher.LaunchTarget.TOWER_GOAL);
//                        helper.stopFlywheel();
//                        helper.tiltDown();
//                        break;
//                }

                Trajectory deliver2ndGoalC = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(61.0, -51.0, Math.toRadians(90.0)))
                        .build();
                fierceBot.followTrajectory(deliver2ndGoalC);

                helper.releaseWobbleGoal();

                Trajectory parkItC = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                        .lineTo(new Vector2d(10.0, -39.0))
                        .build();
                fierceBot.followTrajectory(parkItC);
                break;
        }
        // TERMINATION PHASE
        textFiles.updateTextFiles(lastX, lastY, lastHeading);
    }
}
