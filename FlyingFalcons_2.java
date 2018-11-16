
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name = "FlyingFalcons_2", group = "Pushbot")
public class FlyingFalcons_2 extends OpMode {

    FalconHardwarePushbot_2 robot = new FalconHardwarePushbot_2();
    double clawOffset = 0.0;   // Servo mid position
    final double CLAW_SPEED = 0.02;  // sets rate to move servo

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");    //
        robot.leftClaw.setDirection(Servo.Direction.REVERSE);
        robot.leftClaw.setPosition(0.2);
        robot.rightClaw.setPosition(0.2);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    double max2 (double a, double b) {
        if (a>b) {
            return a;
        }
        else{
            return b;
        }
    }


    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;


        // Use gamepad left & right Bumpers to open and close the claw
        /*if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;*/

        // Move both servos to new position.  Assume servos are mirror image of each other.
        if (gamepad2.b) { // override for debug
            robot.leftClaw.setPosition(max2 (0.2, gamepad2.left_trigger));
            robot.rightClaw.setPosition(max2(0.2, gamepad2.right_trigger));
        } else {
            if (gamepad2.left_bumper == true) {
                robot.leftClaw.setPosition(0.45);
                robot.rightClaw.setPosition(0.45);
                telemetry.addData("CLAWS TO ", 0.45);    //
            } else {
                robot.leftClaw.setPosition(0.2);
                robot.rightClaw.setPosition(0.2);
                telemetry.addData("CLAWS TO ", 0.2);    //
            }
        }


        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
