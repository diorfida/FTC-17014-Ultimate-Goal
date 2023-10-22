package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;

@Deprecated
public class TapeMeasure {

    private LinearOpMode opMode;
    private CRServoImplEx tapeMeasure;

    public TapeMeasure(LinearOpMode opMode){
        this.opMode = opMode;
        tapeMeasure = opMode.hardwareMap.get(CRServoImplEx.class, "tapeMeasure");
        tapeMeasure.setDirection(DcMotorSimple.Direction.REVERSE);
        PwmControl.PwmRange tapeRange = new PwmControl.PwmRange(1000, 2000);
        tapeMeasure.setPwmRange(tapeRange);
    }

    public void run(boolean teleop){
        if (opMode.gamepad2.right_stick_x > .1 || opMode.gamepad2.right_stick_x < .1){
            tapeMeasure.setPower(opMode.gamepad2.right_stick_y);
        }else {
            tapeMeasure.setPower(0);
        }
    }
}
