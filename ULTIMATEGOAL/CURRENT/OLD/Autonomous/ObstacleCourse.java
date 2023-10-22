package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;

import java.util.concurrent.ThreadLocalRandom;

/**
 * The obstacle course our team designed for pre-season drive train testing.
 * @author Domenic Iorfida, FTC 17014
 */
@Disabled
@Autonomous(name = "Obstacle Course")
public class ObstacleCourse extends LinearOpMode {

    private static final Pose2d startPose = new Pose2d(-63.0, 63.0, Math.toRadians(270.0));

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrianHardware fierceBot = new DrivetrianHardware(this);

        Trajectory initMove = fierceBot.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-60, 60))
                .splineToSplineHeading(new Pose2d(-40, 20, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-60, -20, Math.toRadians(270)), Math.toRadians(270))
                .forward(15)
                .build();

        Trajectory insideMov = fierceBot.trajectoryBuilder(initMove.end())
                .splineTo(new Vector2d(-10, -36), 0)
                .forward(16.0)
                .splineTo(new Vector2d(20, 0), Math.toRadians(90))
                .forward(40.0)
                .splineTo(new Vector2d(63, 60), 0)
                .build();

        Trajectory outsideMov = fierceBot.trajectoryBuilder(initMove.end())
                .splineTo(new Vector2d(-10.0, -60.0), 0.0)
                .forward(40.0)
                .splineTo(new Vector2d(60.0, -40.0), Math.toRadians(90.0))
                .forward(20.0)
                .splineTo(new Vector2d(40.0, -5.0), Math.toRadians(180.0))
                .splineTo(new Vector2d(20.0, 5.0), Math.toRadians(90.0))
                .forward(35.0)
                .splineTo(new Vector2d(63.0, 60.0), 0.0)
                .build();

        fierceBot.setPoseEstimate(startPose);

        waitForStart();

        fierceBot.followTrajectory(initMove);

        int randomNum = ThreadLocalRandom.current().nextInt(1, 2 + 1);

        if (randomNum == 1){
            fierceBot.followTrajectory(insideMov);
        }else if (randomNum == 2){
            fierceBot.followTrajectory(outsideMov);
        }
    }
}
