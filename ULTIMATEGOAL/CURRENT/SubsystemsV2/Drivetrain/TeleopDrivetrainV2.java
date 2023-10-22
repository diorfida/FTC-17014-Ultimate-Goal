package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.FieldSpecificationsV2;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

public class TeleopDrivetrainV2 extends DrivetrainHardwareV2 {

    private LinearOpMode opMode;

    private enum ControlModeV2 {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    private static final double powerScale = Math.tan(1);

    public TeleopDrivetrainV2(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;
    }

    public void drive(boolean fieldCentric){

        ControlModeV2 driveStyle;

        if (fieldCentric) {
            driveStyle = ControlModeV2.FIELD_CENTRIC;
        }else {
            driveStyle = ControlModeV2.ROBOT_CENTRIC;
        }

        opMode.telemetry.addData("Drive Style:", driveStyle);

        update();

        switch (driveStyle) {
            case ROBOT_CENTRIC:
                robotCentricDrive();
                break;
            case FIELD_CENTRIC:
                fieldCentricDrive();
                break;
        }
        update();
    }

    private void robotCentricDrive(){
        double sensitivityTrigger = Math.atan(opMode.gamepad1.right_trigger * powerScale);

        double xSpeed = -opMode.gamepad1.left_stick_y * sensitivityTrigger;
        double ySpeed;
        if (opMode.gamepad1.left_bumper){
            ySpeed = -1;
        }else if (opMode.gamepad1.right_bumper){
            ySpeed = 1;
        }else {
            ySpeed = 0;
        }
        double rotationSpeed = -opMode.gamepad1.right_stick_x * sensitivityTrigger;

        Pose2d powerPose = new Pose2d(xSpeed, ySpeed, rotationSpeed);
        setWeightedDrivePower(powerPose);
    }

    private void fieldCentricDrive(){
        double sensitivityTrigger = Math.atan(opMode.gamepad1.right_trigger);

        double xSpeed = -opMode.gamepad1.left_stick_y * sensitivityTrigger;
        double ySpeed = -opMode.gamepad1.left_stick_x * sensitivityTrigger;
        double rotationSpeed = -opMode.gamepad1.right_stick_x * sensitivityTrigger;

        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(
                xSpeed,
                ySpeed
        ).rotated(-poseEstimate.getHeading());

        setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        rotationSpeed
                )
        );
    }

    public void turnToLaunch(Vector2d targetTurn){
        setMotorPowers(0,0,0,0);
        Vector2d turnLocation = getTurnPoint(targetTurn);

        turnProcedure(turnLocation, getPoseEstimate());
    }

    public void faceForward(){
        setMotorPowers(0,0,0,0);
        Vector2d targetTurn = new Vector2d(72, getPoseEstimate().getY());

        turnProcedure(targetTurn, getPoseEstimate());
    }

    private void turnProcedure(@NotNull Vector2d turnLocation, @NotNull Pose2d poseEstimate){
        double goalAngle;

        double xLength = poseEstimate.getX();
        double yLength = poseEstimate.getY();

        double xPoint = turnLocation.getX();
        double yPoint = turnLocation.getY();

        if (xLength < 0){
            xLength = Math.abs(xLength) + xPoint;
        }else {
            xLength = xPoint - xLength;
        }

        if (yPoint < 0){
            if (yLength < 0) {
                if (yLength < yPoint) {
                    yLength = Math.abs(yLength) - -yPoint;
                }else {
                    yLength = Math.abs(Math.abs(yLength) - -yPoint);
                }
            }else {
                yLength += -yPoint;
            }
        }else {
            if (yLength > 0) {
                if (yLength > yPoint) {
                    yLength = Math.abs(yLength) - yPoint;
                }else {
                    yLength = Math.abs(Math.abs(yLength) - yPoint);
                }
            }else {
                yLength =  Math.abs(yLength) + yPoint;
            }
        }

        if (poseEstimate.getY() < yPoint){
            goalAngle = Math.atan(yLength / xLength);
        }else {
            goalAngle = Math.toRadians(360) - Math.atan(yLength / xLength);
        }

        //FOR THE AUTO SHOTS ONLY!!!!
        turn(Angle.normDelta(goalAngle - poseEstimate.getHeading()));
        update();
    }

    @NotNull
    @Contract("_ -> new")
    private Vector2d getTurnPoint(@NotNull Vector2d targetTurn){
        Pose2d robotPose = getPoseEstimate();

        Vector2d translatedTarget = new Vector2d(targetTurn.getX() - robotPose.getX(), targetTurn.getY() - robotPose.getY());

        double externalX = translatedTarget.getX();
        double externalY = translatedTarget.getY();
        double circleRadius = FieldSpecificationsV2.LAUNCHER_CIRCLE_RADIUS;

        double denom = externalY * externalY;
        double rSquared = circleRadius * circleRadius;

        double rightSide = rSquared * denom;

        double quadraticA = Math.pow(externalX, 2) + denom;
        double quadraticB = (-externalX * rSquared) * 2;
        double quadraticC = Math.pow(rSquared, 2) - rightSide;

        double squareRootGuts = Math.pow(quadraticB, 2) - 4 * quadraticA * quadraticC;
        double tanX1 = (-quadraticB + Math.sqrt(squareRootGuts)) / (2 * quadraticA);
        double tanX2 = (-quadraticB - Math.sqrt(squareRootGuts)) / (2 * quadraticA);

        double tanY1 = (rSquared - externalX * tanX1) / externalY;
        double tanY2 = (rSquared - externalX * tanX2) / externalY;

        if (tanY1 < tanY2){
            Vector2d pointOfTangency = new Vector2d(tanX1 + robotPose.getX(), tanY1 + robotPose.getY());
            double differenceX = robotPose.getX() - pointOfTangency.getX();
            double differenceY = robotPose.getY() - pointOfTangency.getY();
            return new Vector2d(targetTurn.getX() + differenceX, targetTurn.getY() + differenceY);
        }else {
            Vector2d pointOfTangency = new Vector2d(tanX2 + robotPose.getX(), tanY2 + robotPose.getY());
            double differenceX = robotPose.getX() - pointOfTangency.getX();
            double differenceY = robotPose.getY() - pointOfTangency.getY();
            return new Vector2d(targetTurn.getX() + differenceX, targetTurn.getY() + differenceY);
        }
    }
}
