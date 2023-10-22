package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.TargetWobbleZone;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/**
 * Ring stack detector class.
 */
public class RingDetector {

    private LinearOpMode opMode;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String SINGLE_RING = "Single";
    private static final String FOUR_RINGS = "Quad";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /**
     * Constructor.
     * @param opMode Instance of the FTC opmode.
     */
    public RingDetector(@NotNull LinearOpMode opMode){
        this.opMode = opMode;

        vuforiaInit(opMode.hardwareMap);

        tfodInit(opMode.hardwareMap);

        if (tfod != null){
            tfod.activate();
        }
    }

    /**
     * Scans for the number of rings in the stack.
     * @return The target wobble goal zone.
     */
    public TargetWobbleZone scan(){
        TargetWobbleZone targetZone = null;

        if (tfod != null){
            List<Recognition> recognitions = tfod.getRecognitions();
            if (recognitions.isEmpty()){
                targetZone = TargetWobbleZone.ZONE_A;
            }else if (recognitions.get(0).getLabel().equals(SINGLE_RING)){
                targetZone = TargetWobbleZone.ZONE_B;
            }else if (recognitions.get(0).getLabel().equals(FOUR_RINGS)){
                targetZone = TargetWobbleZone.ZONE_C;
            }
        }
        opMode.telemetry.addData("Target Zone:", targetZone);
        return targetZone;
    }

    /**
     * Shuts down the TensorFlow program.
     */
    public void terminate(){
        tfod.shutdown();
    }

    private void vuforiaInit(@NotNull HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ARUNzQv/////AAABmadvet0IKkLTuYwkUp9uXY0CZOuxb9EMIUvdMHVx/gM8gAe139KR+EzlsxsRSXIls0q5ofis1JBAB656nqphK0Wz+1j" +
                "4BkAVPgYDUJgSQJK9w/ipnuF1BZPH41REZl80xKnvzfbmPdj9paBZPwpGHYPHpuZoXzV5h8UcsCk8nHpxOp1grWRjSuCLqVDjzO/V277m5G1+aOmMsTSfeRAgyMRdDnMf6m" +
                "bHsGTbBxF70/ppilKr/eZNXqlS1UjubV2pQsflHwyYYr+I9mKAOpYZZZoyY7PthLZ/pkWvcwv5RtLxc9AT1BL9dWsdU/LoQYiyTf2Uc7SI/gdGRfI1TkWW89ppIfeREZMS2V" +
                "vDUzVGGq/T";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void tfodInit(@NotNull HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, FOUR_RINGS, SINGLE_RING);

        tfod.setZoom(2, 16.0/9);
    }
}
