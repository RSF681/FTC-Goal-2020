package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Leon Czolgosz")
public class backupAuton extends LinearOpMode {
    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 8.8;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 9;

    public static final double WHEEL_DIAMETER = 1.42;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    Boolean z = true;

    private UGContourRingPipeline pipeline;
    private PController xCont, yCont, hCont;
    //private VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
    private OpenCvCamera camera;

    private int cameraMonitorViewId;
    private Supplier<UGContourRingPipeline.Height> height;

    private Motor frontLeft, frontRight, backLeft, backRight, shooter;
    private RevIMU imu;
    GamepadEx gPad;
    private MecanumDrive driveTrain;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    public void runOpMode() throws InterruptedException{
        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        shooter = new Motor(hardwareMap, "shooter");

        driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        imu = new RevIMU(hardwareMap);

        gPad = new GamepadEx(gamepad1);

        boolean z = true;

        xCont = new PController(0.4);
        yCont = new PController(0.4);
        hCont = new PController(0.25);

        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
        leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = frontRight.encoder.setDistancePerPulse(-DISTANCE_PER_PULSE);
        centerOdometer = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );


        imu.init();

        frontLeft.setInverted(true);
        backLeft.setInverted(false);
        frontRight.setInverted(false);
        backRight.setInverted(true);

        waitForStart();

        xCont.setSetPoint(-34);
        yCont.setSetPoint(32);
        hCont.setSetPoint(0);

        xCont.setTolerance(5);
        yCont.setTolerance(5);
        hCont.setTolerance(Math.PI/6);

    while(opModeIsActive() && z == true){
        frontLeft.set(0.6);
        frontRight.set(0.6);
        backLeft.set(0.6);
        backRight.set(0.6);
        Thread.sleep(800);
        frontLeft.set(0.0);
        frontRight.set(0.0);
        backLeft.set(0.0);
        backRight.set(0.0);
        z = false;
    }
    }
}
