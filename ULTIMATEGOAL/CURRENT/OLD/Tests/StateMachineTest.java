package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(group = "test")
public class StateMachineTest extends LinearOpMode {

    private enum test{
        FALSE_STATE,
        TRUE_STATE
    }

    test state;

    boolean doubleClick = false;
    boolean toggleDriveMode = false;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {

            if (toggleDriveMode){
                state = test.TRUE_STATE;
            }else {
                state = test.FALSE_STATE;
            }

            switch (state){
                case TRUE_STATE:
                    telemetry.addLine("In the true state block!");
                    telemetry.addData("State:", state);
                    break;
                case FALSE_STATE:
                    telemetry.addLine("In the false blocky thing");
                    telemetry.addData("State:", state);
                    break;
            }

            if (gamepad1.start && gamepad1.y && !toggleDriveMode && !doubleClick){
                toggleDriveMode = true;
            }else if (gamepad1.start && gamepad1.y && toggleDriveMode && !doubleClick){
                toggleDriveMode = false;
            }

            doubleClick = gamepad1.start && gamepad1.y;

            telemetry.update();
        }
    }
}
