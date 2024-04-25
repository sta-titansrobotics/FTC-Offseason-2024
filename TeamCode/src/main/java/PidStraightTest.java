package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp
public class PidTest4_24 extends LinearOpMode {

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    BNO055IMU imu;


    // Define constants for motor encoder counts
    public static final double TICKS_PER_REVOLUTION = 537.7; // Replace with your motor's ticks per revolution
    public static final double WHEEL_DIAMETER_CM = 9.6; // Replace with your wheel diameter
    public static final double DRIVE_SPEED = 0.5; // Adjust the speed as needed
    public static final double RADIUS = 1.875;

    double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    @Override
    public void runOpMode() {
        // Initialize your motors and other hardware components
        motorFrontLeft = hardwareMap.get(DcMotor.class, "lf");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rf");
        motorBackLeft = hardwareMap.get(DcMotor.class, "lr");
        motorBackRight = hardwareMap.get(DcMotor.class, "rr");

        // Reverse motors if needed
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor modes to run using encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Start IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        waitForStart();
        //Drive forward 1 tile
        //PIDStraight(100, 200);;
        pidTurn(90);
        // Stop the robot
        stopRobot();
    }

    //public void PIDStraight(double target_encoder, double timeout, double kMaxSpeed, double heading) {
    public void PIDStraight(double target_cm, double kMaxSpeed) {
        // Constants for PID control
        final double kPDrive = 1.65 ;
        final double kIDrive = 0.0;
        final double kDDrive = 1.0;
        //final double Kpturn_correction = 4;
        final double move_error = 1.0;
        int end_count = 0;

        // Variables for tracking progress
        double startTime = timer.milliseconds();
        double error_drive = 0;
        double integral_drive = 0.0;
        double derivative_drive = 0;
        double prevError_drive = 0.0;

        // Reset the drive motor encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Main PID control loop
        while (opModeIsActive() && (end_count < 1)) {
            //&& (timer.milliseconds() - startTime < timeout) && opModeIsActive()) {
            //double headingError = heading - imu.getAngularOrientation().firstAngle;
            //double headingCorrection = headingError * Kpturn_correction;

            // Calculate the error, integral, and derivative terms for PID control
            double currentTime = timer.milliseconds();
            double deltaTime = currentTime-startTime;
            prevError_drive = error_drive;
            int leftEncoderValue = (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition())/2;
            int rightEncoderValue = (motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition())/2;
            //double current_tick = (leftEncoderValue + rightEncoderValue);
            double current_tick = (leftEncoderValue + rightEncoderValue) / 2.0;
            double current_dist = (current_tick * WHEEL_DIAMETER_CM * Math.PI) / TICKS_PER_REVOLUTION;
            error_drive = target_cm - current_dist;

            // Limit the heading correction to be within the acceptable range
            //headingCorrection = Math.max(-Kpturn_correction, Math.min(headingCorrection, Kpturn_correction));

            // Calculate the integral term
            integral_drive += error_drive * deltaTime; // deltaTime is the time since the last iteration

            // Calculate the derivative term
            derivative_drive = (error_drive - prevError_drive) / deltaTime;

            // Apply heading correction to motor speeds
            double leftSpeed = kPDrive * error_drive + kIDrive * integral_drive + kDDrive * derivative_drive;
            double rightSpeed = kPDrive * error_drive + kIDrive * integral_drive + kDDrive * derivative_drive ;

            // Limit the motor speeds to be within the acceptable range
            leftSpeed = Math.max(-kMaxSpeed, Math.min(leftSpeed, kMaxSpeed));
            rightSpeed = Math.max(-kMaxSpeed, Math.min(rightSpeed, kMaxSpeed));

            // Adjust for turning correction
            //leftSpeed += Kpturn_correction * (heading - imu.getAngularOrientation().firstAngle);
            //rightSpeed -= Kpturn_correction * (heading - imu.getAngularOrientation().firstAngle);

            // Set the motor speeds
            motorFrontLeft.setPower(leftSpeed);
            motorFrontRight.setPower(rightSpeed);
            motorBackLeft.setPower(leftSpeed);
            motorBackRight.setPower(rightSpeed);

            // Check if the error is within the specified move error
            if (Math.abs(error_drive) < move_error) {
                end_count++;
            } else {
                end_count = 0;
            }
            sleep(15);

            // Update telemetry or print to the screen
            telemetry.addData("Total Distance: ", current_dist);
            telemetry.update();

        }

        // Stop the motors once the target distance has been reached
        stopRobot();
    }

    public void stopRobot() {
        // Stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(100);
    }

    public void pidTurn(double targetAngle) {
        // Initialize hardware components
        motorFrontLeft = hardwareMap.get(DcMotor.class, "lf");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rf");
        motorBackLeft = hardwareMap.get(DcMotor.class, "lr");
        motorBackRight = hardwareMap.get(DcMotor.class, "rr");

        // Define constants for PID algorithm
        // double Kp = 0.0012 * error + 2.39; <-- not needed
        double Kp = 2.39; // <-- seems large af my friend :)
        double Ki = 0.0;
        double Kd = 0.0;
        double minspeed = 10;

        // Reset the PID variables
        double error = 0;
        double prevError = 0.0;
        double integral = 0.0;
        double derivative = 0.0;
        double output = 0.0;

        // Initialize time-related variables
        double startTime = timer.milliseconds();
        int end_count = 0;

        // Loop until the robot reaches the target angle
        while ((end_count < 2 || Math.abs(error) > Math.abs(prevError)) && opModeIsActive()) {
            error = targetAngle - imu.getAngularOrientation().firstAngle;
            // Update prevError
            prevError = error;
            double currentTime = timer.milliseconds();
            double deltaTime = currentTime-startTime;
            // Calculate the integral term
            integral += error * deltaTime; // deltaTime is the time since the last iteration

            // Calculate the derivative term
            derivative = (error - prevError) / deltaTime;
            // Calculate error
//            double elapsedTime = timer.milliseconds() - startTime;
//            if (elapsedTime > timeout * 1000) {
//                break;
//            }

            if (Math.abs(error) < 0.7) {
                end_count += 1;
            } else {
                end_count = 0;
            }

            // Calculate the proportional component of the PID algorithm
            double proportional = Kp * error;

            // Calculate the integral component of the PID algorithm
            if (prevError * error < 0) {
                integral = 0;
            }

            // Calculate the derivative component of the PID algorithm
            derivative = error - prevError;

            if (Math.abs(output) < 45) {
                integral += Ki * error;
            }

            // Calculate the output of the PID algorithm
            output = proportional + integral + derivative * Kd;

            // Set the motor speeds based on the output
            motorFrontRight.setPower(output);
            motorBackRight.setPower(output);
            motorFrontLeft.setPower(-output);
            motorBackLeft.setPower(-output);

            // Wait a short time before looping again
            sleep(15);
        }

        stopRobot();

    }

}
