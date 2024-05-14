package zOdometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Odometry_Testing extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor odom_l, odom_h, odom_r;
    private BNO055IMU imu;
    private Orientation angles;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        odom_l = hardwareMap.get(DcMotor.class, "odom_l");
        odom_r = hardwareMap.get(DcMotor.class, "odom_r");
        odom_h = hardwareMap.get(DcMotor.class, "odom_h");

        // Initialize the IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Reset and initialize encoders
        odom_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom_h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odom_l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odom_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odom_h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odom_h.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Program Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Program Running");
        telemetry.update();

        // Initialization
        double deltax1 = 0, deltax2 = 0, deltay1 = 0;
        double prevx1 = 0, prevx2 = 0, prevy1 = 0;
        double heading = 0; // this is in radians

        double x_global_pos = 0;
        double y_global_pos = 0;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //IMU
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double IMUHeading = angles.firstAngle;

            // Calculation of raw encoder movement
            deltax1 = odom_l.getCurrentPosition() - prevx1;
            deltax2 = odom_r.getCurrentPosition() - prevx2;
            deltay1 = odom_h.getCurrentPosition() - prevy1;

            // Additional rotation added this iteration in radians
            double rotation = Math.atan((deltax1 - deltax2) / 43.18);
            // Middle of robot (X-axis)
            double delta_local_x = (deltax1 + deltax2) / 2;

            // Calculate the actual y axis movement
            double delta_perp_pos = deltay1 - 24.5 * rotation / (2 * Math.PI);
            double delta_x = delta_local_x * Math.cos(rotation) - delta_perp_pos * Math.sin(rotation);
            double delta_y = delta_local_x * Math.sin(rotation) + delta_perp_pos * Math.cos(rotation);

            // Update global positions
            x_global_pos += delta_x;
            y_global_pos += delta_y;
            heading += rotation;

            // Convert encoder count to cm (Wheel diameter = 3.8cm)
            double x = 3.8 * Math.PI * (x_global_pos / 8192);
            double y = 3.8 * Math.PI * (y_global_pos / 8192);

            // Telemetry updates
            telemetry.addData("x position (cm): ", x);
            telemetry.addData("y position (cm): ", y);
            telemetry.addData("x1 motor position: ", odom_l.getCurrentPosition());
            telemetry.addData("x1 delta: ", deltax1);
            telemetry.addData("x2 motor position: ", odom_r.getCurrentPosition());
            telemetry.addData("x2 delta: ", deltax2);
            telemetry.addData("y1 motor position: ", odom_h.getCurrentPosition());
            telemetry.addData("y1 delta: ", deltay1);
            telemetry.addData("rotation: ", rotation);
            telemetry.addData("heading: ", heading);
            telemetry.update();

            // Update previous values for next loop iteration
            prevx1 = odom_l.getCurrentPosition();
            prevx2 = odom_r.getCurrentPosition();
            prevy1 = odom_h.getCurrentPosition();


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

        }
    }
}


