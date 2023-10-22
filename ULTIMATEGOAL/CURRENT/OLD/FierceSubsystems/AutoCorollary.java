package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

public class AutoCorollary {

    private LinearOpMode opMode;
    private Alliance alliance;

    private DrivetrianHardware fierceBot;
    private Launcher launcher;
    private WobbleSystem wobbleSystem;
    private Intake intake;

    public AutoCorollary(DrivetrianHardware fierceBot, LinearOpMode opMode, Alliance alliance){
        this.fierceBot = fierceBot;
        this.opMode = opMode;
        this.alliance = alliance;
        launcher = new Launcher(opMode, true);
        wobbleSystem = new WobbleSystem(opMode);
        intake = new Intake(opMode);

        // init phase
        tiltDown();
        wobbleSystem.actuatorDownAuto();
        wobbleSystem.swingUp();
        wobbleSystem.closeClaw();
    }

    /**
     * Method to position the robot and shoot autonomously.
     * @param target The intended launch target.
     */
    public void shoot(Launcher.LaunchTarget target){
        launcher.run(target, fierceBot.getPoseEstimate(), true, alliance, false);
//        switch (alliance){
//            case BLUE:
//                switch (target){
//                    case RIGHT_POWER_SHOT:
//                        turnToPoint(FieldSpecifications.BLUE_RIGHT_POWER_SHOT);
//                        break;
//                    case MIDDLE_POWER_SHOT:
//                        turnToPoint(FieldSpecifications.BLUE_MIDDLE_POWER_SHOT);
//                        break;
//                    case LEFT_POWER_SHOT:
//                        turnToPoint(FieldSpecifications.BLUE_LEFT_POWER_SHOT);
//                        break;
//                    case TOWER_GOAL:
//                        turnToPoint(FieldSpecifications.BLUE_TOWER_GOAL);
//                    case NONE:
//                        break;
//                }
//                break;
//            case RED:
//                switch (target){
//                    case RIGHT_POWER_SHOT:
//                        turnToPoint(FieldSpecifications.RED_RIGHT_POWER_SHOT);
//                        break;
//                    case MIDDLE_POWER_SHOT:
//                        turnToPoint(FieldSpecifications.RED_MIDDLE_POWER_SHOT);
//                        break;
//                    case LEFT_POWER_SHOT:
//                        turnToPoint(FieldSpecifications.RED_LEFT_POWER_SHOT);
//                        break;
//                    case TOWER_GOAL:
//                        turnToPoint(FieldSpecifications.RED_TOWER_GOAL);
//                        break;
//                    case NONE:
//                        break;
//                }
//                break;
//        }
        launcher.run(target, fierceBot.getPoseEstimate(), true, alliance, false);
        opMode.sleep(200);
        launcher.pushRingAuto();
        opMode.sleep(350);
    }

    public void tiltResetUpDown(){
        launcher.tiltResetForAuto();
    }

    public void tiltUp(boolean highGoal){
        launcher.tiltUp(false, highGoal);
    }

    public void tiltDown(){
        launcher.tiltDown(false);
    }

    /**
     * Makes the robot face forwards (heading 0 degrees);
     */
    public void faceForward(){
        turnToForFaceForward(new Vector2d(72, fierceBot.getPoseEstimate().getY()));
    }

    public void startFlywheel(Launcher.LaunchTarget target){
        launcher.run(target, fierceBot.getPoseEstimate(), true, alliance, false);
    }

    public void stopFlywheel(){
        launcher.run(Launcher.LaunchTarget.NONE, fierceBot.getPoseEstimate(), false, alliance, false);
    }

    public void swingClawOut(){
        wobbleSystem.swingOut();
    }

    public void swingClawUp(){wobbleSystem.swingUp();}

    public void grabWobbleGoal(){
        wobbleSystem.actuatorDownAuto();
        wobbleSystem.swingOut();
        opMode.sleep(500);
        wobbleSystem.closeClaw();
        opMode.sleep(200);
        //wobbleSystem.swingUp();
    }

    public void releaseWobbleGoal(){
        wobbleSystem.actuatorDownAuto();
        wobbleSystem.swingOut();
        opMode.sleep(300);
        wobbleSystem.openClaw();
        opMode.sleep(100);
        wobbleSystem.swingUp();
    }

    public void startIntake(){
        intake.autoIntake(true);
    }

    public void stopIntake(){
        intake.autoIntake(false);
    }

    public void turnTo(Vector2d targetTurn){
        fierceBot.setMotorPowers(0,0,0,0);

        Pose2d poseEstimate = fierceBot.getPoseEstimate();
        double goalAngle;

        Vector2d turnLocation = getTurnPoint(targetTurn);

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
        double circleRadius = FieldSpecifications.LAUNCHER_CIRCLE_RADIUS;

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

        opMode.telemetry.addData("Y1", tanY1);
        opMode.telemetry.addData("Y2", tanY2);
        opMode.telemetry.update();

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

    private void turnToForFaceForward(@NotNull Vector2d targetTurn){
        fierceBot.setMotorPowers(0,0,0,0);

        Pose2d poseEstimate = fierceBot.getPoseEstimate();
        double goalAngle;

        double xPoint = targetTurn.getX();
        double yPoint = targetTurn.getY();

        double xLength = poseEstimate.getX();
        double yLength = poseEstimate.getY();

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

        fierceBot.turn(Angle.normDelta(goalAngle - poseEstimate.getHeading()));
    }
}
