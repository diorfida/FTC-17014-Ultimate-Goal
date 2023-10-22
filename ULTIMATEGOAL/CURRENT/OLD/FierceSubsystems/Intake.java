package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.jetbrains.annotations.NotNull;

public class Intake {

    private LinearOpMode opMode;

    private DcMotorEx intakeMotorFront;
    private DcMotorEx intakeMotorCurve;

    private static final double threshold = .1;

    /**
     * Constructor.
     * @param opMode The instance of LinearOpMode from the current OpMode.
     */
    public Intake(@NotNull LinearOpMode opMode){
        this.opMode = opMode;

        intakeMotorFront = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotorCurve = opMode.hardwareMap.get(DcMotorEx.class, "frontEncoder");
    }

    /**
     * Runs the intake subsystem.
     */
    public void run(){
        if (opMode.gamepad2.right_trigger > threshold){
            setMotorPowers(true, false);
        }else if (opMode.gamepad2.left_trigger > threshold){
            setMotorPowers(true, true);
        }else {
            setMotorPowers(false, false);
        }
    }

    public void autoIntake(boolean on){
        if (on){
            setMotorPowers(true, false);
        }else {
            setMotorPowers(false, false);
        }
    }

    private void setMotorPowers(boolean on, boolean reverse){
        if (on) {
            if (reverse){
                intakeMotorFront.setPower(.9);
                intakeMotorCurve.setPower(.8);
            }else {
                intakeMotorFront.setPower(-.9);
                intakeMotorCurve.setPower(-.8); //.88
            }
        }
        else {
            intakeMotorFront.setPower(0);
            intakeMotorCurve.setPower(0);
        }
    }
}
