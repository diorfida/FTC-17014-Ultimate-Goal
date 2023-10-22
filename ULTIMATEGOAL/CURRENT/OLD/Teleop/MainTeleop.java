package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.TeleopDrivetrain;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.RobotMode;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Intake;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.LEDLights;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Launcher;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.WobbleSystem;

import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.LAUNCHER_CIRCLE_RADIUS;

public class MainTeleop {

    private LinearOpMode opMode;
    private TeleopDrivetrain drivetrain;
    private Launcher launcher;
    private Intake intake;
    private WobbleSystem wobbleSystem;
    private LEDLights lights;
    private boolean automatedCapabilities;

    private Alliance alliance;
    private RobotMode mode;

    private boolean driveTrainDoubleClick = false;
    private boolean driveTrainToggle = false;

    private Launcher.LaunchTarget launchTarget = Launcher.LaunchTarget.NONE;
    private boolean tiltToggle = false;
    private boolean tiltDoubleClick = false;

    private boolean wobbleClawToggle = false;
    private boolean wobbleClawDoubleClick = false;
    private boolean wobbleSwingToggle = false;
    private boolean wobbleSwingDoubleClick = false;

    private boolean allianceDoubleClick = false;

    public MainTeleop(LinearOpMode opMode, Alliance alliance, RobotMode mode, boolean singleDriver){
        this.opMode = opMode;
        this.alliance = alliance;
        this.mode = mode;

        drivetrain = new TeleopDrivetrain(opMode, mode, singleDriver);
        automatedCapabilities = drivetrain.checkAutoPosition();
        launcher = new Launcher(opMode, automatedCapabilities);
        intake = new Intake(opMode);
        lights = new LEDLights(opMode);
        wobbleSystem = new WobbleSystem(opMode);

        opMode.telemetry.addData("Automation Active:", automatedCapabilities);
        opMode.telemetry.addData("Alliance:", alliance);
        opMode.telemetry.addData("Robot Mode:", mode);
        opMode.telemetry.addLine("Waiting for start");
        opMode.telemetry.update();

        launcher.tiltDown(true);
        wobbleSystem.openClaw();
        wobbleSystem.swingUp();

        lights.initPhaseLights(automatedCapabilities);
    }

    public boolean execute(ElapsedTime gameTime){
        boolean returner;
        // All of the Drivetrain Things
        Pose2d currentPose = drivetrain.drive(automatedCapabilities, driveTrainToggle, alliance);

        if (opMode.gamepad1.start && opMode.gamepad1.y && !driveTrainToggle && !driveTrainDoubleClick){
            driveTrainToggle = true;
        }else if (opMode.gamepad1.start && opMode.gamepad1.y && driveTrainToggle && !driveTrainDoubleClick){
            driveTrainToggle = false;
        }
        driveTrainDoubleClick = opMode.gamepad1.start && opMode.gamepad1.y;

        if (TeleopDrivetrain.followerState == TeleopDrivetrain.AutoFollowerMode.AUTOMATED_FOLLOWER){
            returner = true;
        }else {
            returner = false;
        }

        // Launcher things
        launcher.run(launchTarget, currentPose, tiltToggle, alliance, true);

        if (opMode.gamepad2.left_bumper && !tiltToggle && !tiltDoubleClick){
            tiltToggle = true;
            launchTarget = Launcher.LaunchTarget.TOWER_GOAL;
        }else if (opMode.gamepad2.left_bumper && tiltToggle && !tiltDoubleClick){
            tiltToggle = false;
            launchTarget = Launcher.LaunchTarget.NONE;
        }else if (opMode.gamepad2.back){
            launchTarget = Launcher.LaunchTarget.POWER_SHOT;
        }
        tiltDoubleClick = opMode.gamepad2.left_bumper;

        // Intake and Transfer system
        intake.run();

        // Wobble Goal System
        wobbleSystem.run(wobbleClawToggle, wobbleSwingToggle);

        if (opMode.gamepad2.right_bumper && !wobbleClawToggle && !wobbleClawDoubleClick){
            wobbleClawToggle = true;
        }else if (opMode.gamepad2.right_bumper && wobbleClawToggle && !wobbleClawDoubleClick){
            wobbleClawToggle = false;
        }
        wobbleClawDoubleClick = opMode.gamepad2.right_bumper;

        if (opMode.gamepad2.y && !wobbleSwingToggle && !wobbleSwingDoubleClick){
            wobbleSwingToggle = true;
        }else if (opMode.gamepad2.y && wobbleSwingToggle && !wobbleSwingDoubleClick){
            wobbleSwingToggle = false;
        }
        wobbleSwingDoubleClick = opMode.gamepad2.y;

        //tapeMeasure.run(true);

        // LED Lights
        lights.updateEndgameTimer(gameTime, alliance, tiltToggle);

        // Alliance stuff for the outreach robot
        if (mode == RobotMode.OUTREACH){
            if (opMode.gamepad1.back && !allianceDoubleClick){
                if (alliance == Alliance.RED){
                    alliance = Alliance.BLUE;
                }else if (alliance == Alliance.BLUE){
                    alliance = Alliance.RED;
                }
            }
            opMode.telemetry.addData("Alliance:", alliance);
            opMode.telemetry.addLine("Press the back button on gamepad 1 to change the alliance.");
            allianceDoubleClick = opMode.gamepad1.back;
        }

        // This is done so that if other programs add telemetry, they will all appear on the screen.
        opMode.telemetry.update();
        return returner;
    }

    public void powerShotsSequence(){
        drivetrain.goToPowerShots();
        launcher.tiltUp(false, false);
        drivetrain.turnPowerAutoPart(1);

        launcher.run(Launcher.LaunchTarget.TOWER_GOAL, new Pose2d(), true, alliance, false);

        opMode.sleep(350);
        shootAutoOnly(Launcher.LaunchTarget.TOWER_GOAL);

        drivetrain.setThePoseEstimate(new Pose2d(-2, -36 + LAUNCHER_CIRCLE_RADIUS, 0));
        drivetrain.turnPowerAutoPart(2);

        shootAutoOnly(Launcher.LaunchTarget.TOWER_GOAL);
        drivetrain.turnPowerAutoPart(3);

        shootAutoOnly(Launcher.LaunchTarget.TOWER_GOAL);
        launcher.run(Launcher.LaunchTarget.NONE, new Pose2d(), false, Alliance.RED, false);
        launcher.tiltDown(false);
    }

    private void shootAutoOnly(Launcher.LaunchTarget target){
        launcher.run(target, new Pose2d(), true, alliance, false);

        opMode.sleep(200);
        launcher.pushRingAuto();
        opMode.sleep(350);
    }

    public void terminate(){
        drivetrain.terminate();
    }
}
