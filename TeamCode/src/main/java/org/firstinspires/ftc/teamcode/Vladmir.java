package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
@TeleOp(name = "John Wilkes Booth")
public class Vladmir extends LinearOpMode {

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 8.8;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 8.8;

    public static final double WHEEL_DIAMETER = 1.42;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private UGContourRingPipeline pipeline;
    //private OpenCvCamera camera;

    //private int cameraMonitorViewId;
    //private VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

    private Motor frontLeft, frontRight, backLeft, backRight, shooter, wobbleGoal;
    private SimpleServo kicker;
    private RevIMU imu;
    GamepadEx gPad;
    private MecanumDrive driveTrain;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private OdometrySubsystem odometrySub;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        shooter = new Motor(hardwareMap, "shooter");
        kicker = new SimpleServo(hardwareMap, "kicker");
        wobbleGoal = new Motor(hardwareMap, "wobbleGoal");

        driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        imu = new RevIMU(hardwareMap);

        gPad = new GamepadEx(gamepad1);

        odometrySub = new OdometrySubsystem(odometry);

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

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        imu.init();


        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        wobbleGoal.setInverted(true);
        wobbleGoal.setRunMode(Motor.RunMode.PositionControl);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(0.6,0.03,0);
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
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.driveRobotCentric(gPad.getLeftY(), -gPad.getLeftX(), -gPad.getRightX());

            //double voltage = voltageSensor.getVoltage();
            //double shooterSpeed = (12.5/voltage);

            if(gamepad1.y){
                shooter.set(0.77);
            } else if(gamepad1.a){
                shooter.set(0.65);
            } else {
                shooter.set(0);
            }

            //-34, 32


            if(gamepad1.dpad_up){
                kicker.turnToAngle(40);
                Thread.sleep(500);
                kicker.turnToAngle(-10);
            }

            if(gamepad1.dpad_left){
                //188
                wobbleGoal.setTargetPosition(20);
                wobbleGoal.set(0.3);
            } else if (gamepad1.dpad_right){
                wobbleGoal.setTargetPosition(0);
                wobbleGoal.set(-0.3);
            } else {
                wobbleGoal.stopMotor();
            }


            telemetry.addData("Angle: ", kicker.getAngle());
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.addData("heading", odometry.getPose().getRotation().getDegrees());
            telemetry.update();
            odometry.updatePose();
        }
    }

}