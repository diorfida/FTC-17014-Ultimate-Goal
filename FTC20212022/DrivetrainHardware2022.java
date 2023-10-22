package org.firstinspires.ftc.teamcode.FTC20212022;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Drivetrain.DriveConstantsV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Util.AxesSignsV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Util.BNO055IMUUtilV2;
import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Util.LynxModuleUtilV2;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class DrivetrainHardware2022 extends MecanumDrive {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0,0,0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0,0,0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static final int POSE_HISTORY_LIMIT = 100;

    public enum DriveMode2022
    {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private DriveMode2022 mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    private DcMotorEx frontLeft, backLeft, backRight, frontRight;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    public DrivetrainHardware2022(LinearOpMode opMode)
    {
        super(DrivetrainConstants2022.kV, DrivetrainConstants2022.kA, DrivetrainConstants2022.kStatic, DrivetrainConstants2022.TRACK_WIDTH, DrivetrainConstants2022.TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = DriveMode2022.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = getVelocityConstraint(DrivetrainConstants2022.MAX_VEL, DrivetrainConstants2022.MAX_ANG_VEL, DrivetrainConstants2022.TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(DrivetrainConstants2022.MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(.25, .25, Math.toRadians(.5)), .5);

        poseHistory = new LinkedList<>();

        LynxModuleUtilV2.ensureMinimumFirmwareVersion(opMode.hardwareMap);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : opMode.hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //BNO055IMUUtilV2.remapAxes(imu, AxesOrder.XYZ, AxesSignsV2.NPN);

        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        for (DcMotorEx motor : motors)
        {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set the localizer
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions()
    {
        return null;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3)
    {

    }

    @Override
    protected double getRawExternalHeading()
    {
        return 0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth)
    {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel)
    {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
