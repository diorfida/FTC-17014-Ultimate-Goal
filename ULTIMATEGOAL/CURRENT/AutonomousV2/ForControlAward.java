package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.AutonomousV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Drivetrain.DrivetrainHardwareV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Enums.AllianceV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.LEDLightsV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrainConstraints;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.LAUNCHER_CIRCLE_RADIUS;

@Disabled
@Autonomous
public class ForControlAward extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(-63, -50, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainHardwareV2 fierceBot = new DrivetrainHardwareV2(this);
        LEDLightsV2 lights = new LEDLightsV2(hardwareMap);

        fierceBot.setPoseEstimate(startPose);

        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();

        lights.allianceLights(AllianceV2.RED);

        Trajectory readyToShoot = fierceBot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-35, -27.0), Math.toRadians(70.0)) //33.25
                .splineTo(new Vector2d(0.0, -19.0), 0.0)
                .build();
        fierceBot.followTrajectory(readyToShoot);

//        Trajectory readyToShoot = fierceBot.trajectoryBuilder(startPose)
//                .forward(43.0)
//                .splineTo(new Vector2d(-2, -19), 0.0)
//                .build();
//        fierceBot.followTrajectory(readyToShoot);

        Trajectory deliver1stGoalA = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(26.0, -60.0, Math.toRadians(45.0)))
                .build();
        fierceBot.followTrajectory(deliver1stGoalA);

        Trajectory get2ndGoalA = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
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

        Trajectory deliver2ndGoalA = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(11.3, -49.5, Math.toRadians(90.0)))
                .build();
        fierceBot.followTrajectory(deliver2ndGoalA);

        Trajectory parkItA = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .lineTo(new Vector2d(10.0, -39.0))
                .build();
        fierceBot.followTrajectory(parkItA);

        Trajectory deliver1stGoalB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(45.0, -47.0, 0.0))
                .build();
        fierceBot.followTrajectory(deliver1stGoalB);

        Trajectory get2ndGoalB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .strafeLeft(10.0)
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

        Trajectory getStackB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                //.splineToConstantHeading(new Vector2d(-36.0, -36.0), Math.toRadians(270.0))
                .splineTo(new Vector2d(-16.0, -46.5), 0.0)
                .splineTo(new Vector2d(0, -45 + LAUNCHER_CIRCLE_RADIUS), 0.0)
                .build();
        fierceBot.followTrajectory(getStackB);

        Trajectory deliver2ndGoalB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(23.0, -30.5, Math.toRadians(180.0)))
                .build();
        fierceBot.followTrajectory(deliver2ndGoalB);

        Trajectory parkItB = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .forward(15.0)
                .build();
        fierceBot.followTrajectory(parkItB);

        Trajectory deliver1stGoalC = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(55.0, -55.5, Math.toRadians(180-45.0)))
                .build();
        fierceBot.followTrajectory(deliver1stGoalC);

        Trajectory get2ndGoalC1 = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(35.0, -30.0, 0.0), Math.toRadians(180.0))
                .build();
        fierceBot.followTrajectory(get2ndGoalC1);

        Trajectory get2ndGoalC2 = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .back(1.0)
                .splineToConstantHeading(new Vector2d(-20.0, -22.0), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(-35, -29), Math.toRadians(180.0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DrivetrainConstraints.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(35, DrivetrainConstraints.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(30))
                .build();
        fierceBot.followTrajectory(get2ndGoalC2);

        Trajectory getStackC = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-36.0, -36.0), Math.toRadians(270.0))
                .splineToConstantHeading(new Vector2d(-16.0, -40.5), 0.0)
                .splineToConstantHeading(new Vector2d(-1.0, -43 + LAUNCHER_CIRCLE_RADIUS), 0.0)
                .build();
        fierceBot.followTrajectory(getStackC);

        Trajectory deliver2ndGoalC = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(61.0, -51.0, Math.toRadians(90.0)))
                .build();
        fierceBot.followTrajectory(deliver2ndGoalC);

        Trajectory parkItC = fierceBot.trajectoryBuilder(fierceBot.getPoseEstimate())
                .lineTo(new Vector2d(10.0, -39.0))
                .build();
        fierceBot.followTrajectory(parkItC);
    }
}
