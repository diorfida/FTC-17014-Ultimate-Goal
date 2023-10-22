package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.jetbrains.annotations.NotNull;

public class IntakeV2 {

    private LinearOpMode opMode;

    private DcMotorEx intakeMotorFront;
    private DcMotorEx intakeMotorCurve;

    private static final double threshold = .1;

    public IntakeV2(@NotNull LinearOpMode opMode){
        this.opMode = opMode;

        intakeMotorFront = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotorCurve = opMode.hardwareMap.get(DcMotorEx.class, "frontEncoder");
    }

    public void run(){
        if (opMode.gamepad2.right_trigger > threshold)
            setMotorPowers(true, false);
        else if (opMode.gamepad2.left_trigger > threshold)
            setMotorPowers(true, true);
        else
            setMotorPowers(false, false);
    }

    protected void setMotorPowers(boolean on, boolean reverse){
        if (on) {
            if (reverse){
                intakeMotorFront.setPower(.9);
                intakeMotorCurve.setPower(.8);
            }else {
                intakeMotorFront.setPower(-.9);
                intakeMotorCurve.setPower(-.8); //.88
            }
        }else {
            intakeMotorFront.setPower(0);
            intakeMotorCurve.setPower(0);
        }
    }
}
