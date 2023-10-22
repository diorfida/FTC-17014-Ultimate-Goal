package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Drivetrain.DrivetrainHardwareV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Enums.AllianceV2;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

public class AutoCorollaryV2 {

    private LinearOpMode opMode;
    private AllianceV2 alliance;

    private DrivetrainHardwareV2 fierceBot;
    private IntakeV2 intake;
    private LauncherV2 launcher;
    private WobbleSystemV2 wobbleSystem;

    public AutoCorollaryV2(DrivetrainHardwareV2 fierceBot, LinearOpMode opMode, AllianceV2 alliance){
        this.fierceBot = fierceBot;
        this.opMode = opMode;
        this.alliance = alliance;
        intake = new IntakeV2(opMode);
        launcher = new LauncherV2(opMode);
        wobbleSystem = new WobbleSystemV2(opMode);

        tiltDown();
        wobbleSystem.actuatorDownAuto();
        wobbleSystem.swingUp();
        wobbleSystem.closeClaw();
    }

    public void shoot(LauncherV2.LaunchTargetV2 target){
        launcher.run(target, true, false);
        opMode.sleep(200);
        launcher.pushRingAuto();
        opMode.sleep(350);
    }

    public void tiltUp(LauncherV2.LaunchTargetV2 target){
        launcher.tiltUp(false, target);
    }

    public void tiltDown(){
        launcher.tiltDown(false);
    }

    public void startFlywheel(){
        launcher.run(LauncherV2.LaunchTargetV2.TOWER_GOAL, false, false);
    }

    public void stopFlywheel(){
        launcher.run(LauncherV2.LaunchTargetV2.NONE, false, false);
    }

    public void swingClawOut(){
        wobbleSystem.swingOut();
    }

    public void swingClawUp(){
        wobbleSystem.swingUp();
    }

    public void grabWobbleGoal(){
        wobbleSystem.swingOut();
        opMode.sleep(500);
        wobbleSystem.closeClaw();
        opMode.sleep(200);
    }

    public void releaseWobbleGoal(){
        wobbleSystem.swingOut();
        opMode.sleep(300);
        wobbleSystem.openClaw();
        opMode.sleep(100);
        wobbleSystem.swingUp();
    }

    public void startIntake(){
        intake.setMotorPowers(true, false);
    }

    public void stopIntake(){
        intake.setMotorPowers(false, false);
    }

    public void turnTo(Vector2d targetTurn){
        fierceBot.setMotorPowers(0,0,0,0);
        Vector2d turnLocation = getTurnPoint(targetTurn);

        turnProcedure(turnLocation, fierceBot.getPoseEstimate());
    }

    public void faceForward(){
        fierceBot.setMotorPowers(0,0,0,0);
        Vector2d targetTurn = new Vector2d(72, fierceBot.getPoseEstimate().getY());

        turnProcedure(targetTurn, fierceBot.getPoseEstimate());
    }

    private void turnProcedure(Vector2d turnLocation, Pose2d poseEstimate){
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
        fierceBot.turn(Angle.normDelta(goalAngle - poseEstimate.getHeading()));
        fierceBot.update();
    }

    @NotNull
    @Contract("_ -> new")
    private Vector2d getTurnPoint(@NotNull Vector2d targetTurn){
        Pose2d robotPose = fierceBot.getPoseEstimate();

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
