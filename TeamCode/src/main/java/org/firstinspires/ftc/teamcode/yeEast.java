package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Kanye East")
public class yeEast extends LinearOpMode {
    private Motor frontLeft, backLeft, frontRight, backRight;

    private MecanumDrive mecDrive;
    private ElapsedTime timeOWO;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "fR");
        frontRight = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");
        timeOWO = new ElapsedTime();

        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        waitForStart();

        timeOWO.reset();
        telemetry.addData("Started Timer", timeOWO.seconds());
        telemetry.update();
        while(opModeIsActive() && timeOWO.seconds() < 0.6) {
            frontLeft.set(-0.7);
            frontRight.set(-0.7);
            backLeft.set(-0.7);
            backRight.set(-0.7);
            telemetry.addData("Current time", timeOWO.seconds());
            telemetry.update();
        }
        telemetry.addData("Current time", timeOWO.seconds());
        telemetry.update();
        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);

        timeOWO.reset();
        telemetry.addData("Started Timer", timeOWO.seconds());
        telemetry.update();
        while(opModeIsActive() && timeOWO.seconds() < 0.6) {
            frontLeft.set(-0.7);
            frontRight.set(-0.7);
            backLeft.set(-0.7);
            backRight.set(-0.7);
            telemetry.addData("Current time", timeOWO.seconds());
            telemetry.update();
        }
    }
}

