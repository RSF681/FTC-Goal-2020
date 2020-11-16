package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.actions.InterruptAction;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Experimental Auton")

public class Auton extends OpMode {

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
    //private VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
    //private OpenCvCamera camera;

    private int cameraMonitorViewId;
    private PurePursuitCommand zoom;
    private Pose2d p1, p2, p3, p4, p5, p6, p7;
    private Supplier<UGContourRingPipeline.Height> height;

    private Motor frontLeft, frontRight, backLeft, backRight, shooter;
    private RevIMU imu;
    GamepadEx gPad;
    private MecanumDrive driveTrain;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private OdometrySubsystem odometrySub;

    @Override
    public void init() {

        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        shooter = new Motor(hardwareMap, "shooter");

        driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        imu = new RevIMU(hardwareMap);

        gPad = new GamepadEx(gamepad1);

        p1 = new Pose2d(0, 0, new Rotation2d(0));
        p2 = new Pose2d(-53, 124, new Rotation2d(90));
        p3 = new Pose2d(-64, 137, new Rotation2d(90));
        p4 = new Pose2d(-43, 156, new Rotation2d(90));
        p5 = new Pose2d(-58, 129, new Rotation2d(0));
        p6 = new Pose2d(-57, 129, new Rotation2d(0));
        p7 = new Pose2d(-56, 129, new Rotation2d(0));

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

        odometrySub = new OdometrySubsystem(odometry);

        zoom = new PurePursuitCommand(driveTrain, odometrySub,
                new StartWaypoint(p1),
                new GeneralWaypoint(3, 28, Math.PI, 0.5, 0.3, 0.5),
                //new InterruptWaypoint(p1, 0.5, 0.3, 10, 5, 5, () -> height = () -> pipeline.getHeight()),
                //new InterruptWaypoint(p1, 0.5, 0.3, 10, 5, 5, this::autoAction),
                new EndWaypoint(3, 28, Math.PI, 0.5, 0.3, 0.5, 1, Math.PI/6)
        );


        imu.init();
        zoom.schedule();

        frontLeft.setInverted(true);
        frontRight.setInverted(true);
/*
        cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Jesus"), cameraMonitorViewId);
        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, true));
        camera.openCameraDeviceAsync(() -> camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));


 */
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetry.addData("x", odometrySub.getPose().getX());
        telemetry.addData("y", odometrySub.getPose().getY());
        telemetry.addData("heading", odometrySub.getPose().getRotation().getDegrees());
    }

    public void autoAction() {
/*
        if (height.get() == UGContourRingPipeline.Height.ZERO) {
            zoom.addWaypoint(new InterruptWaypoint(p2, 0.5, 0.3, 10, 5, 5, () -> telemetry.addData("UPDATE WITH WOBBLE GOAL DROP", 1)));
        } else if (height.get() == UGContourRingPipeline.Height.ONE) {
            zoom.addWaypoint(new InterruptWaypoint(p3, 0.5, 0.3, 10, 5, 5, () -> telemetry.addData("UPDATE WITH WOBBLE GOAL DROP", 1)));
        } else {
            zoom.addWaypoint(new InterruptWaypoint(p4, 0.5, 0.3, 10, 5, 5, () -> telemetry.addData("UPDATE WITH WOBBLE GOAL DROP", 1)));
        }

 */
        // interrupts are for powershots
        /*
        zoom.addWaypoint(new InterruptWaypoint(p5, 0.5, 0.3, 10, 5, 5, () -> telemetry.addData("UPDATE WITH POWERSHOT GOAL", 1)));
        zoom.addWaypoint(new InterruptWaypoint(p6, 0.5, 0.3, 10, 5, 5, () -> telemetry.addData("UPDATE WITH POWERSHOT GOAL", 1)));
        zoom.addWaypoint(new InterruptWaypoint(p7, 0.5, 0.3, 10, 5, 5, () -> telemetry.addData("UPDATE WITH POWERSHOT GOAL", 1)));
        */
        //end if for park}
    }
}
