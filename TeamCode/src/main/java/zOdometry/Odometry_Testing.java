package zOdometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;
import java.lang.Math;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
//Totally original code (part of it at least)
/*Odometry info using accurate measurements:

        43(.18)cm width (from wheel to wheel)
        24.5cm length (from wheel to middle)
 */

@TeleOp
public class Odometry_Testing extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf, lr, rf, rr;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        lf  = hardwareMap.get(DcMotor.class, "lf");
        lr  = hardwareMap.get(DcMotor.class, "lr");
        rf= hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        //Test after
        //leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Program Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Program Running");
        telemetry.update();

        //Initialization
        double deltax1 =0 ;
        double deltax2 = 0;
        double deltay1 = 0 ;
        double prevx1 =0, prevx2=0, prevy1=0;
        double heading = 0 ; //this is in radians


        double x_pos = 0 ;
        double y_pos = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            //calculation of raw encoder movement
            //lf --> x1 on the left
            //rf --> x2 on the right
            //lr --> y1
            deltax1 = lf.getCurrentPosition() - prevx1;
            deltax2 = rf.getCurrentPosition() - prevx2;
            deltay1 = lr.getCurrentPosition() - prevy1;

            //additional rotation added this iteration in radians (this should work for angles < 0)
            double rotation = ((deltax1 - deltax2) / 43.18);
            //middle of robot (X-axis)
            double middle_pos = (deltax1+ deltax2) / 2;
            /*calculate the actual y axis movement perpendicular(actual movement - predicted curve)
             The predicted curve part is the arc of the circle with a radius of
             distance from Y-wheel to centre of robot(24.5)
             circle with radius 24.5 ==> circumference: 49pi * rotation of robot/360 = disired arc length */
            double delta_perp_pos = deltay1 - 24.5 * rotation;

            double delta_x = middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading);
            double delta_y = middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading);

            //this is in terms of encoder count rn
            x_pos += delta_x;
            y_pos += delta_y;
            heading += rotation;


            double x, y;
            //convert encoder count to cm  Diameter = 3.8cm
            //display x and y

            x = 3.8 * Math.PI * x_pos/8192 ;
            y = 3.8 * Math.PI * y_pos/8192 ;

            telemetry.addData("x position: ", x );
            telemetry.addData("y position: " , y);


            //for next iteration of the loop
            prevx1 = lf.getCurrentPosition();
            prevx2 = rf.getCurrentPosition();
            prevy1 = lr.getCurrentPosition();

            //------------------------odometry ends ------------------------------------------

            /*
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            // Set up a variable for each drive wheel to save the power level for telemetry.


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);*/


            telemetry.update();
        }
    }

    static int odom_calc(){

        return 1;
    }

}
