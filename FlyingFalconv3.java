package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import java.util.Locale;
@Disabled
@Autonomous(name = "FlyingFalconAuto")
public class FlyingFalconv3 extends LinearOpMode {
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    FalconHardwarePushbot_2 robot = new FalconHardwarePushbot_2();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.leftClaw.setDirection(Servo.Direction.REVERSE);
        robot.leftClaw.setPosition(0.45);
        robot.rightClaw.setPosition(0.45);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        int startVal = robot.leftDrive.getCurrentPosition();


        while (robot.leftDrive.getCurrentPosition() - startVal < 6100) {
            telemetry.addData("MOTORS ", "ON");
            robot.leftDrive.setPower(.5);
            robot.rightDrive.setPower(.5);
            telemetry.addData("left", robot.leftDrive.getCurrentPosition());
            telemetry.addData("startValue", startVal);
            telemetry.update();

        }
        telemetry.addData("done", "step1");
        robot.leftDrive.setPower(0);
        telemetry.update();
        robot.rightDrive.setPower(0);
        int startValR = robot.rightDrive.getCurrentPosition();

        while (robot.rightDrive.getCurrentPosition() - startValR < 2000) {
            robot.leftDrive.setPower(-.5);
            robot.rightDrive.setPower(.5);
            telemetry.addData("right", robot.rightDrive.getCurrentPosition());
            telemetry.addData("startValue", startValR);
            telemetry.update();
        }
        telemetry.addData("done", "step2");
        telemetry.update();
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        int turnVal = robot.rightDrive.getCurrentPosition();
        while (robot.rightDrive.getCurrentPosition() - turnVal < 1900) {
            robot.rightDrive.setPower(.3);
            robot.leftDrive.setPower(.3);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftClaw.setPosition(.2);
        robot.rightClaw.setPosition(.2);


    }

}

