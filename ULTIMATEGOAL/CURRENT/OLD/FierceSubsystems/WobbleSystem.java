package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.CustomHardware.MagneticLimitSwitch;
import org.jetbrains.annotations.NotNull;

/**
 * Class for control of our wobble goal system.
 */
public class WobbleSystem {

    private LinearOpMode opMode;

    private DcMotor actuator;
    private ServoImplEx wobbleClaw;
    private ServoImplEx wobbleSwing;

    private MagneticLimitSwitch topLimitSwitch;
    private MagneticLimitSwitch bottomLimitSwitch;

    private static final double threshold = .1;

    private boolean clawToggle;
    private boolean swingy;

    /**
     * Constructor.
     * @param opMode The instance of the FTC opmode.
     */
    public WobbleSystem(@NotNull LinearOpMode opMode){
        this.opMode = opMode;

        actuator = opMode.hardwareMap.dcMotor.get("actuator");

        wobbleClaw = opMode.hardwareMap.get(ServoImplEx.class, "wobbleClaw");
        PwmControl.PwmRange clawRange = new PwmControl.PwmRange(750,  1200);
        wobbleClaw.setPwmRange(clawRange);

        wobbleSwing = opMode.hardwareMap.get(ServoImplEx.class, "wobbleSwing");
        PwmControl.PwmRange swingRange = new PwmControl.PwmRange(920, 1535);
        wobbleSwing.setPwmRange(swingRange);

        topLimitSwitch = new MagneticLimitSwitch(opMode.hardwareMap.get(DigitalChannel.class, "topLimitSwitch"));
        bottomLimitSwitch = new MagneticLimitSwitch(opMode.hardwareMap.get(DigitalChannel.class, "bottomLimitSwitch"));
    }

    /**
     * Main method to run the wobble goal system.
     * @param clawToggle If the claw is to be closed.
     * @param swingToggle Determines the position of the swing.
     */
    public void run(boolean clawToggle, boolean swingToggle){
        boolean up;

        if (swingToggle){
            swingOut();
            swingy = true;
            up = false;
        }else {
            swingUp();
            up = true;
        }

        if (opMode.gamepad2.right_stick_y < -threshold && !topLimitSwitch.isMagnetPresent()){
            actuator.setPower(-opMode.gamepad2.right_stick_y);
        }else if (opMode.gamepad2.right_stick_y > threshold && !bottomLimitSwitch.isMagnetPresent()){
            actuator.setPower(- opMode.gamepad2.right_stick_y);
        }else {
            actuator.setPower(0);
        }

        if (clawToggle){
            closeClaw();
            clawToggle = true;
        }else {
            if (clawToggle){
                openClaw();
                clawToggle = false;
            }else {
                openClaw();
            }
        }
    }

    /**
     * Opens the wobble goal claw.
     */
    public void openClaw(){
        wobbleClaw.setPosition(0);
    }

    /**
     * Closes the wobble goal claw.
     */
    public void closeClaw(){
        wobbleClaw.setPosition(1);
    }

    /**
     * Swings the wobble claw up and in.
     */
    public void swingUp(){
        wobbleSwing.setPosition(0);
    }

    /**
     * Swings the wobble claw to the back of the robot.
     */
    public void swingOut(){
        wobbleSwing.setPosition(1);
    }

    public void actuatorDownAuto(){
        while (!bottomLimitSwitch.isMagnetPresent()){
            actuator.setPower(-1);
        }
    }
}
