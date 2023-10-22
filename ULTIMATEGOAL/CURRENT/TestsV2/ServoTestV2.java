package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.TestsV2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Disabled
@TeleOp(group = "test")
public class ServoTestV2 extends LinearOpMode {

    ServoImplEx tilt;

    @Override
    public void runOpMode() throws InterruptedException {
        tilt = hardwareMap.get(ServoImplEx.class, "ringPush");
        PwmControl.PwmRange tiltRange = new PwmControl.PwmRange(600, 950);
        tilt.setPwmRange(tiltRange);

        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                tilt.setPosition(1);
            }else if (gamepad1.b){
                tilt.setPosition(0);
            }
        }
    }
}
