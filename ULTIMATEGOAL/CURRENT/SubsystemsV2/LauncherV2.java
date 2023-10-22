package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LauncherV2 {

    private LinearOpMode opMode;

    public enum LaunchTargetV2{
        TOWER_GOAL,
        POWER_SHOT,
        NONE
    }

    private DcMotorEx flywheel;
    private static final double MOTOR_TICKS_PER_REV = 28;
    private static final double MOTOR_MAX_RPM = 6000;
    private static final double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    private static boolean RUN_USING_ENCODER = true;
    private static boolean DEFAULT_GAINS = false;
    private static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0, 0, 13.75);
    private VoltageSensor batteryVoltageSensor;

    private static final double highGoalTilt = .98;
    private static final double powerShotTilt = .9325;
    private static final double pushForwardTime = 300;
    private static final double pushBackTime = pushForwardTime * 2;

    private ServoImplEx ringPush;
    private ElapsedTime rapidTime;
    private boolean firstTimePush;

    private ServoImplEx tilt;
    private ServoImplEx stopper;
    private boolean firstTimeUp;
    private boolean firstTimeDown;
    private ElapsedTime stopperTime;

    public LauncherV2(LinearOpMode opMode){
        this.opMode = opMode;

        flywheel = opMode.hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(motorConfigurationType);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(flywheel, MOTOR_VELO_PID);

        ringPush = opMode.hardwareMap.get(ServoImplEx.class, "ringPush");
        PwmControl.PwmRange pushRange = new PwmControl.PwmRange(1400, 1950); //1945
        ringPush.setDirection(Servo.Direction.REVERSE);
        ringPush.setPwmRange(pushRange);

        tilt = opMode.hardwareMap.get(ServoImplEx.class, "tilt");
        PwmControl.PwmRange tiltRange = new PwmControl.PwmRange(1200, 1390 + 235);
        tilt.setPwmRange(tiltRange);
        tilt.setPosition(0);

        stopper = opMode.hardwareMap.get(ServoImplEx.class, "stopper");
        PwmControl.PwmRange stopperRange = new PwmControl.PwmRange(970, 1185);
        stopper.setDirection(Servo.Direction.REVERSE);
        stopper.setPwmRange(stopperRange);
        stopperTime = new ElapsedTime();

        rapidTime = new ElapsedTime();
    }

    public void run(LaunchTargetV2 target, boolean tiltToggled, boolean teleop){
        if (target == LaunchTargetV2.NONE)
            flywheel.setPower(0);
        else
            flywheel.setPower(.875);

        if (teleop){
            // pushing rings
            if (opMode.gamepad2.x)
                rapidFire();
            else if (opMode.gamepad2.a)
                pushRingThrough();
            else
                resetRingPush();

            // hopper tilt
            if (tiltToggled) {
                tiltUp(true, target);
                firstTimeDown = true;
            }else {
                tiltDown(true);
                firstTimeUp = true;
            }
        }
    }

    public void tiltUp(boolean teleop, LaunchTargetV2 target){
        if (teleop){
            if (firstTimeUp){
                stopperTime.reset();
                stopper.setPosition(1);
                firstTimeUp = false;
            }else {
                if (stopperTime.milliseconds() >= 150){
                    if (opMode.gamepad2.dpad_down)
                        tilt.setPosition(0);
                    else {
                        if (target == LaunchTargetV2.POWER_SHOT)
                            tilt.setPosition(powerShotTilt);
                        else
                            tilt.setPosition(highGoalTilt);
                    }
                }
            }
        }else {
            stopper.setPosition(1);
            opMode.sleep(150);
            if (target == LaunchTargetV2.POWER_SHOT)
                tilt.setPosition(powerShotTilt);
            else
                tilt.setPosition(highGoalTilt);
        }
    }

    public void tiltDown(boolean teleop){
        if (teleop){
            if (firstTimeDown){
                stopperTime.reset();
                tilt.setPosition(0);
                firstTimeDown = false;
            }else {
                if (stopperTime.milliseconds() >= 250)
                    stopper.setPosition(0);
            }
        }else {
            tilt.setPosition(0);
            opMode.sleep(250);
            stopper.setPosition(0);
        }
    }

    private void rapidFire(){
        if (firstTimePush){
            rapidTime.reset();
            firstTimePush = false;
        }else {
            if (rapidTime.milliseconds() >= pushForwardTime && rapidTime.milliseconds() < pushBackTime)
                resetRingPush();
            else if (rapidTime.milliseconds() < pushForwardTime)
                pushRingThrough();
            else
                firstTimePush = true;
        }
    }

    public void pushRingAuto(){
        pushRingThrough();
        opMode.sleep((long) pushForwardTime);
        resetRingPush();
    }

    private void pushRingThrough(){
        ringPush.setPosition(1);
    }
    private void resetRingPush(){
        ringPush.setPosition(0);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER) {
            return;
        }

        if (!DEFAULT_GAINS) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        }
    }
}
