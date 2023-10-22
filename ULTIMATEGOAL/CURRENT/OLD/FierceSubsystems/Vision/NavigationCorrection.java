package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.jetbrains.annotations.NotNull;

public class NavigationCorrection {

    private LinearOpMode opMode;
    private DrivetrianHardware fierceBot;
    private WebcamName webcam;
    private VuforiaLocalizer vuforia;
    private Alliance alliance;

    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;
    private OpenGLMatrix cameraLocation;

    public NavigationCorrection(@NotNull LinearOpMode opMode, DrivetrianHardware fierceBot, Alliance alliance){
        this.opMode = opMode;
        this.fierceBot = fierceBot;
        this.alliance = alliance;

        webcam = opMode.hardwareMap.get(WebcamName.class, "webcam");

// TODO: Remove the cameraMonitorViewId to save power if we have no need to see the camera on the RC.
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ARUNzQv/////AAABmadvet0IKkLTuYwkUp9uXY0CZOuxb9EMIUvdMHVx/gM8gAe139KR+EzlsxsRSXIls0q5ofis1JBAB656nqphK0Wz+1j" +
                "4BkAVPgYDUJgSQJK9w/ipnuF1BZPH41REZl80xKnvzfbmPdj9paBZPwpGHYPHpuZoXzV5h8UcsCk8nHpxOp1grWRjSuCLqVDjzO/V277m5G1+aOmMsTSfeRAgyMRdDnMf6m" +
                "bHsGTbBxF70/ppilKr/eZNXqlS1UjubV2pQsflHwyYYr+I9mKAOpYZZZoyY7PthLZ/pkWvcwv5RtLxc9AT1BL9dWsdU/LoQYiyTf2Uc7SI/gdGRfI1TkWW89ppIfeREZMS2V" +
                "vDUzVGGq/T";

        parameters.cameraName = webcam;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        visionTargets = vuforia.loadTrackablesFromAsset("UltimateGoal");

        float mmPerInch = 25.4f;
        float mmFTCFieldWidth = (12*12 - 2) * mmPerInch;

        OpenGLMatrix visionTargetLocation;

        if (alliance == Alliance.RED) {
            target = visionTargets.get(1);
            target.setName("Red Tower Goal");

            visionTargetLocation = createMatrix(72 * mmPerInch, -36 * mmPerInch, 0, 90, 0, 270);

            cameraLocation = createMatrix(7 * mmPerInch, 0, 7 * mmPerInch, 0, 0, 0);
        }else {
            target = visionTargets.get(0);
            target.setName("Blue Tower Goal");

            visionTargetLocation = createMatrix(mmFTCFieldWidth / 2, 24.5f * mmPerInch, 0, 90, 0, 270);
        }

        target.setLocationFtcFieldFromTarget(visionTargetLocation);
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setCameraLocationOnRobot(webcam, cameraLocation);
    }

    public void start(){
        visionTargets.activate();
    }

    public void run(){
        OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation(); //getUpdated

        if (latestLocation != null){
            opMode.telemetry.addData("location", formatMatrix(latestLocation));
            double[] information = getStuff(formatMatrix(latestLocation));
            for (double things : information){
                opMode.telemetry.addData("yo", things);
            }
            Pose2d robotThink = fierceBot.getPoseEstimate();
            double upperBoundX = robotThink.getX() + .5;
            double lowerBoundX = robotThink.getX() - .5;
            double upperBoundY = robotThink.getY() + .5;
            double lowerBoundY = robotThink.getY() - .5;
            if (information[3] > upperBoundX || information[3] < lowerBoundX){
                fierceBot.setPoseEstimate(new Pose2d(information[3], fierceBot.getPoseEstimate().getY(), fierceBot.getPoseEstimate().getHeading()));
            }
            if (information[4] > upperBoundY || information[4] < lowerBoundY){
                fierceBot.setPoseEstimate(new Pose2d(fierceBot.getPoseEstimate().getX(), information[4], fierceBot.getPoseEstimate().getHeading()));
            }
            opMode.telemetry.addData("LOCATION UPDATED", true);
        }else {
            opMode.telemetry.addData("Current Location", "not different");
        }

        if (latestLocation != null){
            String visionLocation = formatMatrix(latestLocation);

        }
        fierceBot.update();
    }

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    private String formatMatrix(@NotNull OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }

    private double[] getStuff(String transformation){
        String theThingsString = transformation.substring(15);
        String[] info = new String[6];
        double[] returner = new double[6];

        for (int i = 1; i <= 6; i ++){
            if (i == 6 || i == 3){
                int temp = theThingsString.indexOf("}");
                info[i - 1] = theThingsString.substring(0, temp);
                if (i == 3){
                    theThingsString = theThingsString.substring(temp + 1);
                    theThingsString = theThingsString.substring(2);
                }
            }else {
                int temp = theThingsString.indexOf(" ");
                info[i - 1] = theThingsString.substring(0, temp);
                theThingsString = theThingsString.substring(temp + 1);
            }
        }

        for (int i = 0; i < 6; i ++){
            returner[i] = Double.parseDouble(info[i]);
        }
        return toInches(returner);
        //return returner;
    }

    private double[] toInches(double[] convert){
        double [] newStuff = new double[convert.length];
        newStuff[0] = convert[0];
        newStuff[1] = convert[1];
        newStuff[2] = convert[2];
        for (int i = 3; i < convert.length; i++){
            newStuff[i] = convert[i] / 25.4;
        }
        return newStuff;
    }
}
