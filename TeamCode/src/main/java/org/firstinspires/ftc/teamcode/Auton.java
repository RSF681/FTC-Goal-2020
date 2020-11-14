package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Charles J. Guiteau")
@Disabled
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
    private OpenCvCamera camera;

    private int cameraMonitorViewId;
    private PurePursuitCommand zoom;

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

        odometrySub = new OdometrySubsystem(odometry);

        zoom = new PurePursuitCommand(driveTrain, odometrySub, new StartWaypoint(0, 0),
                new GeneralWaypoint(200, 0, 0.8, 0.8, 30),
                new EndWaypoint(400, 0, 0, 0.5,
                        0.5, 30, 0.8, 1));

        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
        leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = frontRight.encoder.setDistancePerPulse(-DISTANCE_PER_PULSE);
        centerOdometer = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        imu.init();

        frontLeft.setInverted(true);
        frontRight.setInverted(true);

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

    }

    @Override


    public void loop() {
        while(z){
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.addData("heading", odometry.getPose().getRotation().getDegrees());
            String height = "[HEIGHT]" + " " + pipeline.getHeight();
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
            odometry.updatePose();
            z = false;
        }
    }
}
