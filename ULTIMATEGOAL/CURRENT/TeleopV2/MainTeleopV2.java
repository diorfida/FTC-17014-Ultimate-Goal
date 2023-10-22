package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.TeleopV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Drivetrain.TeleopDrivetrainV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Enums.AllianceV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.FieldSpecificationsV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.IntakeV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.LEDLightsV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.LauncherV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.WobbleSystemV2;

public class MainTeleopV2 {

    private LinearOpMode opMode;

    private TeleopDrivetrainV2 drivetrain;
    private boolean drivetrainDoubleClick = false;
    private boolean drivetrainToggle = false;

    private LauncherV2 launcher;
    private LauncherV2.LaunchTargetV2 launchTarget = LauncherV2.LaunchTargetV2.NONE;
    private boolean tiltToggle = false;
    private boolean tiltDoubleClick = false;

    private IntakeV2 intake;

    private WobbleSystemV2 wobbleSystem;
    private boolean wobbleClawToggle = false;
    private boolean wobbleClawDoubleClick = false;
    private boolean wobbleSwingToggle = false;
    private boolean wobbleSwingDoubleClick = false;

    private LEDLightsV2 lights;

    private AllianceV2 alliance;

    public MainTeleopV2(LinearOpMode opMode, AllianceV2 alliance){
        this.opMode = opMode;
        this.alliance = alliance;

        drivetrain = new TeleopDrivetrainV2(opMode);
        launcher = new LauncherV2(opMode);
        intake = new IntakeV2(opMode);
        wobbleSystem = new WobbleSystemV2(opMode);
        lights = new LEDLightsV2(opMode.hardwareMap);

        int min = 301;
        int max = 700;
        int range = (max - min) + 1;
        int rand = (int)(Math.random() * range) + min;
        opMode.telemetry.addLine("Go ahead and score " + rand + " points this match!");
        opMode.telemetry.addData("Alliance:", alliance);
        opMode.telemetry.addLine("Waiting for start");
        opMode.telemetry.update();

        launcher.tiltDown(false);
        wobbleSystem.run(false, false);
        lights.initPhaseLights();
    }

    public void execute(ElapsedTime gameTime){
        // DRIVETRAIN
        drivetrain.drive(drivetrainToggle);

        if (opMode.gamepad1.start && opMode.gamepad1.y && !drivetrainToggle && !drivetrainDoubleClick)
            drivetrainToggle = true;
        else if (opMode.gamepad1.start && opMode.gamepad1.y && drivetrainToggle && !drivetrainDoubleClick)
            drivetrainToggle = false;

        drivetrainDoubleClick = opMode.gamepad1.start && opMode.gamepad1.y;

        // LAUNCHER
        launcher.run(launchTarget, tiltToggle, true);

        if (opMode.gamepad2.left_bumper && !tiltToggle && !tiltDoubleClick){
            tiltToggle = true;
            launchTarget = LauncherV2.LaunchTargetV2.TOWER_GOAL;
        }else if (opMode.gamepad2.left_bumper && tiltToggle && !tiltDoubleClick){
            tiltToggle = false;
            launchTarget = LauncherV2.LaunchTargetV2.NONE;
        }else if (opMode.gamepad2.back)
            launchTarget = LauncherV2.LaunchTargetV2.POWER_SHOT;

        tiltDoubleClick = opMode.gamepad2.left_bumper;

        // INTAKE
        intake.run();

        // WOBBLE SYSTEM
        wobbleSystem.run(wobbleClawToggle, wobbleSwingToggle);

        if (opMode.gamepad2.right_bumper && !wobbleClawToggle && !wobbleClawDoubleClick)
            wobbleClawToggle = true;
        else if (opMode.gamepad2.right_bumper && wobbleClawToggle && !wobbleClawDoubleClick)
            wobbleClawToggle = false;

        wobbleClawDoubleClick = opMode.gamepad2.right_bumper;

        if (opMode.gamepad2.y && !wobbleSwingToggle && !wobbleSwingDoubleClick)
            wobbleSwingToggle = true;
        else if (opMode.gamepad2.y && wobbleSwingToggle && !wobbleSwingDoubleClick)
            wobbleSwingToggle = false;

        wobbleSwingDoubleClick = opMode.gamepad2.y;

        // LED LIGHTS
        lights.updateEndgameTimer(gameTime, alliance, tiltToggle);

        opMode.telemetry.update();
    }

    public void redPowerShotsSequence(){
        drivetrain.setPoseEstimate(new Pose2d(0, 15, Math.toRadians(270.0)));

        Trajectory powerShots = drivetrain.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-2, -19), 0.0)
                .build();

        drivetrain.followTrajectory(powerShots);
        drivetrain.update();

        launcher.tiltUp(false, LauncherV2.LaunchTargetV2.POWER_SHOT);
        drivetrain.faceForward();
        launcher.run(LauncherV2.LaunchTargetV2.TOWER_GOAL, true, false);
        opMode.sleep(500);
        shootAutoOnly();

        // TODO: LOOK HERE FOR POTENTIAL FIXES
        //drivetrain.setPoseEstimate(new Pose2d(-2, -36 + LAUNCHER_CIRCLE_RADIUS, 0));
        drivetrain.turnToLaunch(FieldSpecificationsV2.RED_RIGHT_POWER_SHOT_TELE);
        shootAutoOnly();

        drivetrain.turnToLaunch(FieldSpecificationsV2.RED_LEFT_POWER_SHOT_TELE);
        shootAutoOnly();

        launcher.run(LauncherV2.LaunchTargetV2.NONE, false, false);
        launcher.tiltDown(false);
    }

    private void shootAutoOnly(){
        launcher.run(LauncherV2.LaunchTargetV2.POWER_SHOT, true, false);

        opMode.sleep(200);
        launcher.pushRingAuto();
        opMode.sleep(350);
    }
}
