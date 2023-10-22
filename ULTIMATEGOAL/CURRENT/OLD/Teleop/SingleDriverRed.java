package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.RobotMode;

@Disabled
@Deprecated
@TeleOp(name = "SINGLE RED Driver Controlled", group = "main")
public class SingleDriverRed extends LinearOpMode {
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
                        if (gamepad1.left_bumper){
                            teleMode = TeleMode.GET_THE_POWER_SHOTS;
                        }
                        break;
                    case GET_THE_POWER_SHOTS:
                        teleop.powerShotsSequence();
                        break;
                }
            }
        }
        // TERMINATION PHASE
        teleop.terminate();
    }
}
