package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Charles J. Guiteau")
public class timedAutonomous extends LinearOpMode {
    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private int cameraMonitorViewId;

    private Motor frontLeft, backLeft, frontRight, backRight;

    private MecanumDrive mecDrive;
    private ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "fR");
        frontRight = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");
        time = new ElapsedTime();

        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);


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

        waitForStart();

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);

        time.reset();
        telemetry.addData("Started Timer", time.seconds());
        telemetry.update();


        if(pipeline.getHeight() == UGContourRingPipeline.Height.ONE){
            while(opModeIsActive() && time.seconds() < 1.6) {
                frontLeft.set(-0.7);
                frontRight.set(-0.7);
                backLeft.set(-0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }
        } else if (pipeline.getHeight() == UGContourRingPipeline.Height.FOUR){
            while(opModeIsActive() && time.seconds() < 0.10) {
                frontLeft.set(0.7);
                frontRight.set(-0.7);
                backLeft.set(0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }

            time.reset();

            while(opModeIsActive() && time.seconds() < 1.9) {
                frontLeft.set(-0.7);
                frontRight.set(-0.7);
                backLeft.set(-0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }
        } else {
            while(opModeIsActive() && time.seconds() < 0.12) {
                frontLeft.set(0.7);
                frontRight.set(-0.7);
                backLeft.set(0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }

            time.reset();

            while(opModeIsActive() && time.seconds() < 0.80) {
                frontLeft.set(-0.7);
                frontRight.set(-0.7);
                backLeft.set(-0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }
        }
    }
}

