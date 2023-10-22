package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.CustomHardware.MagneticLimitSwitchV2;
import org.jetbrains.annotations.NotNull;

public class WobbleSystemV2 {

    private LinearOpMode opMode;

    private DcMotor actuator;
    private ServoImplEx wobbleClaw;
    private ServoImplEx wobbleSwing;

    private MagneticLimitSwitchV2 topLimitSwitch;
    private MagneticLimitSwitchV2 bottomLimitSwitch;

    private static final double threshold = .1;

    public WobbleSystemV2(@NotNull LinearOpMode opMode){
        this.opMode = opMode;

        actuator = opMode.hardwareMap.dcMotor.get("actuator");

        wobbleClaw = opMode.hardwareMap.get(ServoImplEx.class, "wobbleClaw");
        PwmControl.PwmRange clawRange = new PwmControl.PwmRange(750,  1200);
        wobbleClaw.setPwmRange(clawRange);

        wobbleSwing = opMode.hardwareMap.get(ServoImplEx.class, "wobbleSwing");
        PwmControl.PwmRange swingRange = new PwmControl.PwmRange(920, 1535);
        wobbleSwing.setPwmRange(swingRange);

        topLimitSwitch = new MagneticLimitSwitchV2(opMode.hardwareMap.get(DigitalChannel.class, "topLimitSwitch"));
        bottomLimitSwitch = new MagneticLimitSwitchV2(opMode.hardwareMap.get(DigitalChannel.class, "bottomLimitSwitch"));
    }

    public void run(boolean clawToggle, boolean swingToggle){
        if (swingToggle)
            swingOut();
        else
            swingUp();

        if (opMode.gamepad2.left_stick_y < -threshold && !topLimitSwitch.isMagnetPresent())
            actuator.setPower(-opMode.gamepad2.left_stick_y);
        else if (opMode.gamepad2.left_stick_y > threshold && !bottomLimitSwitch.isMagnetPresent())
            actuator.setPower(-opMode.gamepad2.left_stick_y);
        else
            actuator.setPower(0);

        if (clawToggle)
            closeClaw();
        else
            openClaw();
    }

    protected void openClaw(){
        wobbleClaw.setPosition(0);
    }

    protected void closeClaw(){
        wobbleClaw.setPosition(1);
    }

    protected void swingUp(){
        wobbleSwing.setPosition(0);
    }

    protected void swingOut(){
        wobbleSwing.setPosition(1);
    }

    protected void actuatorDownAuto(){
        while (!bottomLimitSwitch.isMagnetPresent()){
            actuator.setPower(-1);
        }
    }
}
