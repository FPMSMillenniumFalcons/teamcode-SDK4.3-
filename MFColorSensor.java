package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
@Disabled
@Autonomous(name = "MFColorSensor")
public class MFColorSensor extends LinearOpMode {
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    HardwarePushbot2 robot = new HardwarePushbot2();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor sensorColor1;
    ColorSensor sensorColor2;
    DistanceSensor sensorDistance1;
    DistanceSensor sensorDistance2;

    // create 2D array  table of color conditions
    // | Seq | Alpha low | Alpha high | Red low | Red high | Green low | Green high | Blue low | Blue high |

    public int is_color_yellow(ColorSensor sensorColor) {
        int isyellow = 0;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        //FIXME base on hsvValues determine if it's yellow color

        //telemetry.addData("IN:Distance (cm) 1",
        //    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("IN: Alpha1", sensorColor.alpha());
        telemetry.addData("IN: Red1  ", sensorColor.red());
        telemetry.addData("IN: Green1", sensorColor.green());
        telemetry.addData("IN: Blue1 ", sensorColor.blue());

        telemetry.addData("IN: getDeviceName ", sensorColor.getDeviceName());
        telemetry.addData("IN: Hue1", hsvValues[0]);
        telemetry.update();

        return isyellow; //hsvValues;
    }

    public int is_color_same(double delta_val) {
        int is_same_color = 0;
        float alpha1 = sensorColor1.alpha();
        float alpha2 = sensorColor2.alpha();
        float red1 = sensorColor1.red();
        float red2 = sensorColor2.red();
        float green1 = sensorColor1.green();
        float green2 = sensorColor2.green();
        float blue1 = sensorColor1.blue();
        float blue2 = sensorColor2.blue();
        float alpha_delta = Math.abs(alpha1 - alpha2);
        float red_delta = Math.abs(red1 - red2);
        float green_delta = Math.abs(green1 - green2);
        float blue_delta = Math.abs(blue1 - blue2);

        if ((alpha_delta > delta_val) || (red_delta > delta_val) || (green_delta > delta_val) || (blue_delta > delta_val)) {
            is_same_color = 0;
        } else {
            is_same_color = 1;
        }

        //telemetry.addData("IN:Distance (cm) 1",
        //    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("IN: Alpha1", alpha1);
        telemetry.addData("IN: Red1  ", red1);
        telemetry.addData("IN: Green1", green1);
        telemetry.addData("IN: Blue1 ", blue1);
        telemetry.addData("IN: Alpha2", alpha2);
        telemetry.addData("IN: Red2  ", red2);
        telemetry.addData("IN: Green2", green2);
        telemetry.addData("IN: Blue2 ", blue2);
        telemetry.addData("IN: is_same_color ", is_same_color);
        telemetry.update();

        return is_same_color; //hsvValues;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        robot.init(ahwMap);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        float hsvValues1[], hsvValues2[];

        //sensorColor1 = hardwareMap.get(ColorSensor.class, "sensor_color_distance1");
        //sensorColor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance2");

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance1 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance1");
        //sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance2");
        // Send telemetry message to signify robot waiting;

        /*
         * main loop of autonomous
         */
        int same_color = 0;
        while (opModeIsActive()) {
            // hsvValues is an array that will hold the hue, saturation, and value information.
            //hsvValues1 = is_color_yellow(sensorColor1);
            //hsvValues2 = is_color_yellow(sensorColor2);
            same_color = is_color_same(0.1);

             //send the info back to driver station using telemetry function.
            /*telemetry.addData("Distance (cm) 1",
                    String.format(Locale.US, "%.02f", sensorDistance1.getDistance(DistanceUnit.CM)));
            telemetry.addData("Distance (cm)2",
                    String.format(Locale.US, "%.02f", sensorDistance2.getDistance(DistanceUnit.CM)));*/
            telemetry.addData("Is it same color ", same_color);
            //h telemetry.addData("Hue2", hsvValues2[0]);

            telemetry.update();

        }
    }



    /* FIXME: add code here
    public double query_distant()
    {

    }
    */
}

