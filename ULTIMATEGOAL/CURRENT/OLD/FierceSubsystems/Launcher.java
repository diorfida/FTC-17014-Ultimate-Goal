package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.jetbrains.annotations.NotNull;

/**
 * Class that controls our ring launching system.
 * @author Domenic Iorfida, FTC 17014
 */
@Config
public class Launcher {

    private LinearOpMode opMode;
    private boolean automatedCapabilities;

    /**
     * Where the launch target is.
     */
    public enum LaunchTarget{
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

    //private static final double IDEAL_RPM = 0.865 * MOTOR_MAX_RPM;

    private static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0, 0, 13.75);
    //private static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30, 0, 2, 15);

//    private double lastKp = 0.0;
//    private double lastKi = 0.0;
//    private double lastKd = 0.0;
//    private double lastKf = getMotorVelocityF();

    //private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    private static final double highGoalTilt = .98; //.98
    private static final double powerShotTilt = .9375;

    private static final double pushForwardTime = 300;
    private static final double pushBackTime = pushForwardTime * 2;

    private ServoImplEx ringPush;
    private ServoImplEx tilt;
    private ServoImplEx stopper;

    ElapsedTime stopperTime;
    private boolean firstTimeUp;
    private boolean firstTimeDown;

    ElapsedTime rapidTime;
    private boolean firstTimePush;
    private boolean pushFinish;

    /**
     * Constructor.
     * @param opMode The instance of the FTC opmode.
     */
    public Launcher(@NotNull LinearOpMode opMode, boolean automatedCapabilities){
        this.opMode = opMode;
        this.automatedCapabilities = automatedCapabilities;

        flywheel = opMode.hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(motorConfigurationType);

        if (RUN_USING_ENCODER)
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(flywheel, MOTOR_VELO_PID);

        ringPush = opMode.hardwareMap.get(ServoImplEx.class, "ringPush");
        PwmControl.PwmRange pushRange = new PwmControl.PwmRange(1400, 1940); //1945
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

    /**
     * Runs the flywheel system.
     * @param target Where the ring is intended to be launched to.
     * @param currentPose The current pose of the robot on the field.
     * @param alliance Specified alliance.
     */
    public void run(LaunchTarget target, Pose2d currentPose, boolean tiltToggled, Alliance alliance, boolean teleop) {
//        if (automatedCapabilities){
//            switch (target){
//                case TOWER_GOAL:
//                    switch (alliance){
//                        case BLUE:
//                            flywheel.setVelocity(flywheelFunction(currentPose, FieldSpecifications.BLUE_TOWER_GOAL), AngleUnit.RADIANS);
//                            break;
//                        case RED:
//                            flywheel.setVelocity(flywheelFunction(currentPose, FieldSpecifications.RED_TOWER_GOAL), AngleUnit.RADIANS);
//                            break;
//                    }
//                    break;
//                case LEFT_POWER_SHOT:
//                    switch (alliance){
//                        case BLUE:
//                            flywheel.setVelocity(flywheelFunction(currentPose, FieldSpecifications.BLUE_LEFT_POWER_SHOT), AngleUnit.RADIANS);
//                            break;
//                        case RED:
//                            flywheel.setVelocity(flywheelFunction(currentPose, FieldSpecifications.RED_LEFT_POWER_SHOT), AngleUnit.RADIANS);
//                            break;
//                    }
//                    break;
//                case MIDDLE_POWER_SHOT:
//                    switch (alliance){
//                        case BLUE:
//                            flywheel.setVelocity(flywheelFunction(currentPose, FieldSpecifications.BLUE_MIDDLE_POWER_SHOT), AngleUnit.RADIANS);
//                            break;
//                        case RED:
//                            flywheel.setVelocity(flywheelFunction(currentPose, FieldSpecifications.RED_MIDDLE_POWER_SHOT), AngleUnit.RADIANS);
//                            break;
//                    }
//                    break;
//                case RIGHT_POWER_SHOT:
//                    switch (alliance){
//                        case BLUE:
//                            flywheel.setVelocity(flywheelFunction(currentPose, FieldSpecifications.BLUE_RIGHT_POWER_SHOT), AngleUnit.RADIANS);
//                            break;
//                        case RED:
//                            flywheel.setVelocity(flywheelFunction(currentPose, FieldSpecifications.RED_RIGHT_POWER_SHOT), AngleUnit.RADIANS);
//                            break;
//                    }
//                    break;
//                case NONE:
//                    flywheel.setVelocity(0);
//                    break;
//            }
//            opMode.telemetry.addData("Launcher Mode:", target);
//        }else {
//            if (target == LaunchTarget.NONE){
//                flywheel.setVelocity(0);
//                opMode.telemetry.addData("Launcher Mode:", "OFF");
//            }else {
//                flywheel.setVelocity(MAX_VELOCITY, AngleUnit.RADIANS);
//                opMode.telemetry.addData("Launcher Mode:", "ON");
//            }
//        }
        if (target == LaunchTarget.NONE) {
            //setVelocity(flywheel, 0);
            flywheel.setPower(0);
            opMode.telemetry.addData("Launcher Mode:", "OFF");
        }else {
            //setVelocity(flywheel, IDEAL_RPM);
            flywheel.setPower(.875);
            opMode.telemetry.addData("Launcher Mode:", "ON");
        }

        if (teleop) {
            if (opMode.gamepad2.x) {
                rapidFire();
                pushFinish = false;
            }else if (opMode.gamepad2.a){
                pushRingThrough();
//                if (!pushFinish){
//                    rapidFire();
//                }
            }else {
                resetRingPush();
                pushFinish = false;
            }

            if (tiltToggled) {
                tiltUp(true, target != LaunchTarget.POWER_SHOT);
                firstTimeDown = true;
            } else {
                tiltDown(true);
                firstTimeUp = true;
            }
        }
    }

    public void tiltUp(boolean teleop, boolean highGoal){
        if (teleop) {
            if (firstTimeUp) {
                stopperTime.reset();
                stopper.setPosition(1);
                firstTimeUp = false;
            } else {
                if (stopperTime.milliseconds() >= 150) {
                    if (opMode.gamepad2.dpad_down) {
                        tilt.setPosition(0);
                    } else {
                        if (highGoal) {
                            tilt.setPosition(highGoalTilt);
                        }else {
                            tilt.setPosition(powerShotTilt);
                        }
                    }
                }
            }
        }else {
            stopper.setPosition(1);
            opMode.sleep(150);
            if (highGoal) {
                tilt.setPosition(highGoalTilt);
            }else {
                tilt.setPosition(powerShotTilt);
            }
        }
    }

    public void tiltDown(boolean teleop){
        if (teleop) {
            if (firstTimeDown) {
                stopperTime.reset();
                tilt.setPosition(0);
                firstTimeDown = false;
            } else {
                if (stopperTime.milliseconds() >= 250) {
                    stopper.setPosition(0);
                }
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
            if (rapidTime.milliseconds() >= pushForwardTime && rapidTime.milliseconds() < pushBackTime){
                resetRingPush();
            }else if (rapidTime.milliseconds() < pushForwardTime){
                pushRingThrough();
            }else {
                firstTimePush = true;
                pushFinish = true;
            }
        }
    }

    private void pushRingThrough(){
        ringPush.setPosition(1);
    }

    private void resetRingPush(){
        ringPush.setPosition(0);
    }

    public void pushRingAuto(){
        pushRingThrough();
        opMode.sleep((long) pushForwardTime);
        resetRingPush();
    }

    public void tiltResetForAuto(){
        tilt.setPosition(0);
        opMode.sleep(250);
        tilt.setPosition(powerShotTilt);
    }

    private void setVelocity(DcMotorEx motor, double power) {
        if(RUN_USING_ENCODER) {
            motor.setVelocity(rpmToTicksPerSecond(power));
        }
        else {
            motor.setPower(power / MOTOR_MAX_RPM);
        }
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

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV);
    }

//    private void incrementalAngleAdjustment(){
//        double adjustmentStick = -opMode.gamepad1.right_stick_y;
//        double highest = 1;
//        double lowest = .9;
//        double threshold = .1;
//        int state;
//
//        if (adjustmentStick > threshold){
//            state = 1;
//        }else if (adjustmentStick < threshold){
//            state = -1;
//        }else {
//            state = 0;
//        }
//
//        if (state == 1){ // adjust up
//            tilt.setPosition(Range.clip(tilt.getPosition() + adjustmentStick / 20, lowest, highest));
//        }else if (state == -1){
//            tilt.setPosition(Range.clip(tilt.getPosition() - adjustmentStick / 20, lowest, highest));
//        }
//    }
}
