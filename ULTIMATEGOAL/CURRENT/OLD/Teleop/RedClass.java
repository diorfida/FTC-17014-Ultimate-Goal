package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.RobotMode;

@Disabled
@TeleOp(name = "RED Driver Controlled", group = "main")
public class RedClass extends LinearOpMode {
    private Alliance alliance = Alliance.RED;
    private RobotMode mode = RobotMode.COMPETITIVE;

    private TeleMode teleMode = TeleMode.NORMAL_OPERATION;

    @Override
    public void runOpMode() throws InterruptedException {
        MainTeleop teleop = new MainTeleop(this, alliance, mode, false);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        boolean inTraj = false;

        while (opModeIsActive()){
            if (!inTraj) {
                switch (teleMode){
                    case NORMAL_OPERATION:
                        inTraj = teleop.execute(timer);
                        if (gamepad1.left_trigger > .5){
                            teleMode = TeleMode.GET_THE_POWER_SHOTS;
                        }
                        break;
                    case GET_THE_POWER_SHOTS:
                        teleop.powerShotsSequence();
                        teleMode = TeleMode.NORMAL_OPERATION;
                        break;
                }
            }
        }
        // TERMINATION PHASE
        teleop.terminate();
    }
}
