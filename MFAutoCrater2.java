package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "MFAutoCrater2")
public class MFAutoCrater2 extends LinearOpMode {
    HardwarePushbot2 robot = new HardwarePushbot2();


    /*
     * instantiate chassis object. chassis object should contains all driver capability.
     * Gyro sensor should be part of chassis
     * should have following methods:
     *    chassis.shift_left(double distance or double time);
     *    chassis.shift_right(double distance or double time);
     *    chassis.turn_left(double degress);
     *    chassis.turn_right(double degress);
     *    chassis.go_straight(double distance or double time); + foward, - backward
     * optional:
     *    chassis.go_leftdiag(double distance or double time); + foward, - backward
     *    chassis.go_rightdiag(double distance or double time); + foward, - backward
     */
    MFChassis chassis = new MFChassis();

    /*
     * instantiate lift and claw object and should have following method
     *     lift_claw.lift_self(double height) // extent, hook, retract
     *     lift_claw.lower_self(double height) // extend, unhook, retract
     *     lift_claw.close()
     *     lift_claw.open()
     *     lift_claw.updown(double height) + up, <=0 down and reset
     *     lift_claw.extend(double length) + extend out, <=0 retract
     */
    MFLiftClaw lift_claw = new MFLiftClaw();

    /*
     * instantiate ColorDistanceSensor. this could be part of chassis object
     *     colorDistant.is_color_yellow()
     *     colorDistant.query_distant()
     */
    MFColorSensor colorDistant = new MFColorSensor();


    private ElapsedTime runtime = new ElapsedTime();




    //@Override
    public void runOpMode() throws InterruptedException {
        // code zero

        robot.init(hardwareMap);
        robot.liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftDrive.setPower(0.1);

        sleep(1000);
        robot.liftDrive.setPower(0);

        robot.claw.setPosition(0.66);
        robot.wrist.setPosition(0.9);



        while(!opModeIsActive()) {
            telemetry.addData("TouchSensorData; ", robot.touchSensor.getValue());
            telemetry.update();
            if (!robot.touchSensor.isPressed()) {
                robot.liftDrive.setPower(0.3);
            }
        }

        // Wait until we're told to go
        waitForStart();

        int raiseValue = robot.liftDrive.getCurrentPosition();

        while (robot.liftDrive.getCurrentPosition() - raiseValue < 2700 ) { // lift
            telemetry.addData("LiftMotor ", "ON");
            robot.liftDrive.setPower(-0.5);
            telemetry.addData("lift", robot.liftDrive.getCurrentPosition());
            telemetry.addData("raiseValue", raiseValue);
            telemetry.update();
        }// Raise the Lift
        robot.liftDrive.setPower(0);
        telemetry.addData("done", "liftOver");
        telemetry.update();

        sleep(300);


        int startVal = robot.leftDrive.getCurrentPosition();
        while (Math.abs(robot.leftDrive.getCurrentPosition() - startVal) < 25) { // move out of hook
            // implement PID control here
            telemetry.addData("MOTORS ", "ON");
            robot.leftDrive.setPower(-0.2);
            robot.rightDrive.setPower(-0.4);
            robot.leftDriveB.setPower(-0.45);
            robot.rightDriveB.setPower(-0.2);
            telemetry.addData("start", startVal);
            telemetry.addData("Motor", robot.leftDrive.getCurrentPosition());
            telemetry.addData("startValue", startVal);
            telemetry.update();
        }

        robot.leftDrive.setPower(0);
        robot.leftDriveB.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightDriveB.setPower(0);

        int leftVal = robot.leftDrive.getCurrentPosition();
        while (Math.abs((robot.leftDrive.getCurrentPosition() + robot.rightDriveB.getCurrentPosition()) / 2 - leftVal) < 2900) { // move to box
            // implement PID control here
            telemetry.addData("MOTORS ", "ON");
            robot.leftDrive.setPower(0.35);
            robot.rightDrive.setPower(-0.35);
            robot.leftDriveB.setPower(-0.35);
            robot.rightDriveB.setPower(0.35);
            telemetry.addData("Motormove", robot.leftDrive.getCurrentPosition() - leftVal);
            telemetry.addData("startValue", leftVal);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.leftDriveB.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightDriveB.setPower(0);

        /*int turnRVal = robot.leftDrive.getCurrentPosition();
        while (Math.abs(robot.leftDrive.getCurrentPosition() - turnRVal) < 325) { // turn to box
            // implement PID control here
            telemetry.addData("MOTORSt ", "ON");
            robot.leftDrive.setPower(0.5);
            robot.rightDrive.setPower(-0.5);
            robot.leftDriveB.setPower(0.5);
            robot.rightDriveB.setPower(-0.5);
            telemetry.addData("Motormove", robot.leftDrive.getCurrentPosition());
            telemetry.addData("startValue", turnRVal);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.leftDriveB.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightDriveB.setPower(0);

        robot.wrist.setPosition(0);
        sleep(2500);
        robot.claw.setPosition(1);

        sleep(500);

        robot.claw.setPosition(0);
        robot.wrist.setPosition(0.9);*/

        /*sleep(1000);

        int fwdVal = robot.leftDrive.getCurrentPosition();
        while (Math.abs(robot.leftDrive.getCurrentPosition() - fwdVal) < 500) { // turn to box
            // implement PID control here
            telemetry.addData("MOTORSt ", "ON");
            robot.leftDrive.setPower(-0.5);     fixme
            robot.rightDrive.setPower(0.5);
            robot.leftDriveB.setPower(-0.5);
            robot.rightDriveB.setPower(0.5);
            telemetry.addData("Motormove", robot.leftDrive.getCurrentPosition());
            telemetry.addData("startValue", turnRVal);
            telemetry.update();
        }*/



        /* while (robot.liftDrive.getCurrentPosition() - raiseValue < 10 ) {
            telemetry.addData("LiftMotor ", "ON");
            robot.liftDrive.setPower(-0.5);
            telemetry.addData("lift", robot.liftDrive.getCurrentPosition());
            telemetry.addData("raiseValue", raiseValue);
            telemetry.update();
        }// Raise the Lift
        telemetry.addData("done", "liftOver");
        robot.liftDrive.setPower(0);
        telemetry.update();

      /*  int startVal = robot.leftDrive.getCurrentPosition();
        while (robot.leftDrive.getCurrentPosition() - startVal < 1000) {
            // implement PID control here
            telemetry.addData("MOTORS ", "ON");
            robot.leftDrive.setPower(-0.5);
            robot.rightDrive.setPower(-0.5); fixme
            robot.leftDriveB.setPower(-0.5);
            robot.rightDriveB.setPower(-0.5);
            telemetry.addData("Motor", robot.leftDrive.getCurrentPosition());
            telemetry.addData("startValue", startVal);
            telemetry.update();*/

        /*int lowerValue = robot.liftDrive.getCurrentPosition();
        while (robot.liftDrive.getCurrentPosition() - lowerValue < 10) {
            //telemetry.addData("LiftMotor2 ", "ON");
            robot.liftDrive.setPower(0.50);
            //telemetry.addData("lift2", robot.liftDrive.getCurrentPosition());
            //telemetry.addData("lowerValue", lowerValue);
            //telemetry.update();

        }*/
        //telemetry.addData("done", "liftOver2");

        //telemetry.update();

        /* //go left
        int startVal = robot.rightDrive.getCurrentPosition();
        while (robot.rightDrive.getCurrentPosition() - startVal <  distance ){
            // implement PID control here
            robot.leftDrive.setPower(-pwr);
            robot.rightDrive.setPower(pwr);
            robot.leftDriveB.setPower(pwr);
            robot.rightDriveB.setPower(-pwr);
            telemetry.addData("right", robot.rightDrive.getCurrentPosition());
            telemetry.addData("startValue", startVal);
            telemetry.update();
        }
        //stop


         */


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //chassis(hardwareMap);  //FIXME:
        lift_claw.init(hardwareMap);  //FIXME:
        //colorDistant(hardwarMap);  //FIXME:

        /*
         * main loop of autonomous
         */
        while (opModeIsActive()) {
            // lift_claw.lift_self()  //FIXME:

            // lift_claw.lower_self()  //FIXME:

            //FIXME:
            // chassis drive to next station
            // etc the whole game should be here.
            //hi this is anamika
        }

    }


}


