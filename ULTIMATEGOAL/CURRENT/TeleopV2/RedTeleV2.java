package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.TeleopV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Enums.AllianceV2;

@TeleOp(name = "RED Teleop", group = "main")
public class RedTeleV2 extends LinearOpMode {

    private final AllianceV2 alliance = AllianceV2.RED;
    private TeleModeV2 teleMode = TeleModeV2.NORMAL_OPERATION;

    @Override
    public void runOpMode() throws InterruptedException {
        MainTeleopV2 teleop = new MainTeleopV2(this, alliance);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()){
            switch (teleMode){
                case NORMAL_OPERATION:
                    teleop.execute(timer);
                    if (gamepad1.left_trigger > .5)
                        teleMode = TeleModeV2.GET_THE_POWER_SHOTS;
                    break;
                case GET_THE_POWER_SHOTS:
                    teleop.redPowerShotsSequence();
                    teleMode = TeleModeV2.NORMAL_OPERATION;
                    break;
            }
        }
    }
}
