package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.Alliance;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Enums.RobotMode;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.TextFileMaster;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Vision.NavigationCorrection;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.io.File;

import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.RED_LEFT_POWER_SHOT;
import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.RED_MIDDLE_POWER_SHOT;
import static org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.FieldSpecifications.RED_RIGHT_POWER_SHOT;

/**
 * Algorithms for tele-op drive train control.
 * @author Domenic Iorfida, FTC 17014
 */
@Config
public class TeleopDrivetrain {

    private DrivetrianHardware fierceBot;
    private TextFileMaster textFiles;
    private LinearOpMode opMode;

    private NavigationCorrection navigationCorrection;

    private RobotMode robotMode;

    private enum ControlMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public enum AutoFollowerMode {
        DRIVER_CONTROLLED,
        AUTOMATED_FOLLOWER
    }

    private static final double powerScaleFull = Math.tan(1);
    private static final double powerScaleHalf = Math.tan(.5);

    public static AutoFollowerMode followerState = AutoFollowerMode.DRIVER_CONTROLLED;

    private File startingXFile = AppUtil.getInstance().getSettingsFile("XFile.txt");
    private File startingYFile = AppUtil.getInstance().getSettingsFile("YFile.txt");
    private File startingHeadingFile = AppUtil.getInstance().getSettingsFile("HeadingFile.txt");

    private boolean singleDriver;
    private boolean started = false;

    private static double angle;

    /**
     * Constructor.
     * @param opMode The instance of the FTC opmode that called this class.
     * @param robotMode The mode of the robot drivetrain.
     */
    public TeleopDrivetrain(LinearOpMode opMode, RobotMode robotMode, boolean singleDriver){
        fierceBot = new DrivetrianHardware(opMode);
        fierceBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        textFiles = new TextFileMaster(fierceBot);
        this.opMode = opMode;
        this.robotMode = robotMode;
        this.singleDriver = singleDriver;
        navigationCorrection = new NavigationCorrection(opMode, fierceBot, Alliance.RED);
        navigationCorrection.start();
    }

    /**
     * Checks if a position was last reported from autonomous mode.
     * @return If a position exists.
     */
    public boolean checkAutoPosition(){
        return textFiles.DeriveTextFiles(startingXFile, startingYFile, startingHeadingFile);
    }

    /**
     * Enters the drivetrain into termination phase.
     */
    public void terminate(){
        startingXFile.delete();
        startingYFile.delete();
        startingHeadingFile.delete();
    }

    /**
     * Main method for controlling the drivetrain.
     * @param autoDriveEnabled If the automated drive train is enabled.
     * @param fieldCentric Toggle variable from the opMode.
     * @param alliance What alliance we are on.
     */
    public Pose2d drive(boolean autoDriveEnabled, boolean fieldCentric, Alliance alliance) {

        ControlMode driveStyle;

        if (fieldCentric) {
            driveStyle = ControlMode.FIELD_CENTRIC;
        } else {
            driveStyle = ControlMode.ROBOT_CENTRIC;
        }

        opMode.telemetry.addData("Drive Style:", driveStyle);

        fierceBot.update();
        Pose2d currentPose = fierceBot.getPoseEstimate();
        opMode.telemetry.addData("follower state", followerState);

        switch (followerState) {
            case DRIVER_CONTROLLED:
                robotCommandCenter(autoDriveEnabled, driveStyle, currentPose, alliance);
                break;
            case AUTOMATED_FOLLOWER:
                if (opMode.gamepad1.dpad_up){
                    fierceBot.cancelFollowing();
                    followerState = AutoFollowerMode.DRIVER_CONTROLLED;
                }

                if (!fierceBot.isBusy()){
                    followerState = AutoFollowerMode.DRIVER_CONTROLLED;
                }
                break;
        }
        //navigationCorrection.run();
        fierceBot.update();
        return fierceBot.getPoseEstimate();
    }

    private void robotCommandCenter(boolean autoDriveEnabled, ControlMode driveStyle, Pose2d currentPose, Alliance alliance){
        if (singleDriver){
            if (opMode.gamepad2.b && autoDriveEnabled) { // turn to the tower goal
                switch (alliance) {
                    case BLUE:
                        turnTo(FieldSpecifications.BLUE_TOWER_GOAL, false);
                        break;
                    case RED:
                        turnTo(FieldSpecifications.RED_TOWER_GOAL, false);
                        break;
                }
            } else if (opMode.gamepad2.dpad_up && autoDriveEnabled) { // turn to the middle power shot
                switch (alliance) {
                    case BLUE:
                        turnTo(FieldSpecifications.BLUE_MIDDLE_POWER_SHOT, false);
                        break;
                    case RED:
                        turnTo(RED_MIDDLE_POWER_SHOT, false);
                        break;
                }
            } else if (opMode.gamepad2.dpad_left && autoDriveEnabled) { // turn to the left power shot
                switch (alliance) {
                    case BLUE:
                        turnTo(FieldSpecifications.BLUE_LEFT_POWER_SHOT, false);
                        break;
                    case RED:
                        turnTo(RED_LEFT_POWER_SHOT, false);
                        break;
                }
            } else if (opMode.gamepad2.dpad_right && autoDriveEnabled) { // turn to the right power shot
                switch (alliance) {
                    case BLUE:
                        turnTo(FieldSpecifications.BLUE_RIGHT_POWER_SHOT, false);
                        break;
                    case RED:
                        turnTo(RED_RIGHT_POWER_SHOT, false);
                        break;
                }
            } else {
                switch (driveStyle) {
                    case ROBOT_CENTRIC:
                        RobotCentricDrive();
                        break;
                    case FIELD_CENTRIC:
                        FieldCentricDrive();
                        break;
                }
            }
        }else {
            if (opMode.gamepad1.a && autoDriveEnabled) { // turn to the tower goal
                switch (alliance) {
                    case BLUE:
                        turnTo(FieldSpecifications.BLUE_TOWER_GOAL, false);
                        break;
                    case RED:
                        turnTo(FieldSpecifications.RED_TOWER_GOAL, false);
                        break;
                }
            } else if (opMode.gamepad1.y && autoDriveEnabled) { // turn to the middle power shot
                switch (alliance) {
                    case BLUE:
                        turnTo(FieldSpecifications.BLUE_MIDDLE_POWER_SHOT, false);
                        break;
                    case RED:
                        turnTo(RED_MIDDLE_POWER_SHOT, false);
                        break;
                }
            } else if (opMode.gamepad1.x && autoDriveEnabled) { // turn to the left power shot
                switch (alliance) {
                    case BLUE:
                        turnTo(FieldSpecifications.BLUE_LEFT_POWER_SHOT, false);
                        break;
                    case RED:
                        turnTo(RED_LEFT_POWER_SHOT, false);
                        break;
                }
            } else if (opMode.gamepad1.b && autoDriveEnabled) { // turn to the right power shot
                switch (alliance) {
                    case BLUE:
                        turnTo(FieldSpecifications.BLUE_RIGHT_POWER_SHOT, false);
                        break;
                    case RED:
                        turnTo(RED_RIGHT_POWER_SHOT, false);
                        break;
                }
            } else {
                switch (driveStyle) {
                    case ROBOT_CENTRIC:
                        RobotCentricDrive();
                        break;
                    case FIELD_CENTRIC:
                        FieldCentricDrive();
                        break;
                }
            }
        }
    }

    private void RobotCentricDrive(){
        switch (robotMode){
            case COMPETITIVE:
                if (singleDriver){
                    double xSpeed = -opMode.gamepad2.left_stick_y;
                    double ySpeed = -opMode.gamepad2.left_stick_x;
                    double rotationSpeed = -opMode.gamepad2.right_stick_x;

                Pose2d powerPose = new Pose2d(xSpeed, ySpeed, rotationSpeed);
                fierceBot.setWeightedDrivePower(powerPose);
                }else {
                    double sensitivityTriggerFull = Math.atan(opMode.gamepad1.right_trigger * powerScaleFull);

                    double xSpeed = -opMode.gamepad1.left_stick_y * sensitivityTriggerFull;
                    double ySpeed;
                    if (opMode.gamepad1.left_bumper){
                        ySpeed = -1;
                    }else if (opMode.gamepad1.right_bumper){
                        ySpeed = 1;
                    }else {
                        ySpeed = 0;
                    }
                    double rotationSpeed = -opMode.gamepad1.right_stick_x * sensitivityTriggerFull;

                Pose2d powerPose = new Pose2d(xSpeed, ySpeed, rotationSpeed);
                fierceBot.setWeightedDrivePower(powerPose);

                }
                break;
            case OUTREACH:
                double sensitivityTriggerHalf = Math.atan(opMode.gamepad1.right_trigger / 2 * powerScaleHalf);

                double outreachX = -opMode.gamepad1.left_stick_y * sensitivityTriggerHalf;
                double outreachY = -opMode.gamepad1.left_stick_x * sensitivityTriggerHalf;
                double outreachRotation = -opMode.gamepad1.right_stick_x * sensitivityTriggerHalf;

                Pose2d powerOutreachP = new Pose2d(outreachX, outreachY, outreachRotation);
                fierceBot.setWeightedDrivePower(powerOutreachP);

                break;
        }
    }

    private void FieldCentricDrive(){
        switch (robotMode){
            case COMPETITIVE:
                double sensitivityTriggerFull = Math.atan(opMode.gamepad1.right_trigger * powerScaleFull);

                double xSpeed = -opMode.gamepad1.left_stick_y * sensitivityTriggerFull;
                double ySpeed = -opMode.gamepad1.left_stick_x * sensitivityTriggerFull;
                double rotationSpeed = -opMode.gamepad1.right_stick_x * sensitivityTriggerFull;

                Pose2d poseEstimate = fierceBot.getPoseEstimate();

                Vector2d input = new Vector2d(
                        xSpeed,
                        ySpeed
                ).rotated(-poseEstimate.getHeading());

                fierceBot.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                rotationSpeed
                        )
                );

                break;
            case OUTREACH:
                double sensitivityTriggerHalf = Math.atan(opMode.gamepad1.right_trigger / 2 * powerScaleHalf);

                double outreachX = -opMode.gamepad1.left_stick_y * sensitivityTriggerHalf;
                double outreachY = -opMode.gamepad1.left_stick_x * sensitivityTriggerHalf;
                double outreachRotation = -opMode.gamepad1.right_stick_x * sensitivityTriggerHalf;

                Pose2d thePoseEstimate = fierceBot.getPoseEstimate();

                Vector2d theInput = new Vector2d(
                        outreachX,
                        outreachY
                ).rotated(-thePoseEstimate.getHeading());

                fierceBot.setWeightedDrivePower(
                        new Pose2d(
                                theInput.getX(),
                                theInput.getY(),
                                outreachRotation
                        )
                );
                break;
        }
    }

    private void turnTo(Vector2d targetTurn, boolean autoPower){
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

        if (autoPower){
            fierceBot.turn(Angle.normDelta(goalAngle - poseEstimate.getHeading()));
            fierceBot.update();
        }else {
            fierceBot.turnAsync(Angle.normDelta(goalAngle - poseEstimate.getHeading()));
            fierceBot.update();

            followerState = AutoFollowerMode.AUTOMATED_FOLLOWER;
        }
    }

    @NotNull
    @Contract("_ -> new")
    private Vector2d getTurnPoint(@NotNull Vector2d targetTurn){
        Pose2d robotPose = fierceBot.getPoseEstimate();
        Vector2d circleCenter = new Vector2d(0,0);

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

    public void goToPowerShots(){
        fierceBot.setPoseEstimate(new Pose2d(0, 15, 0));

        Trajectory powerShots = fierceBot.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-2, -15, 0.0))
                .build();

        fierceBot.followTrajectory(powerShots);


        fierceBot.update();
    }

    public void setThePoseEstimate(Pose2d newPose){
        fierceBot.setPoseEstimate(newPose);
    }

    public void turnPowerAutoPart(int part){
        if (part == 1){
            turnToForFaceForward(new Vector2d(72, fierceBot.getPoseEstimate().getY()));
        }else if (part == 2){
            turnTo(RED_RIGHT_POWER_SHOT, true);
        }else if (part == 3){
            turnTo(RED_MIDDLE_POWER_SHOT, true);
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
