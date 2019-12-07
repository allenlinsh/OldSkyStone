package org.firstinspires.ftc.teamcode;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.opencsv.CSVReader;

@TeleOp
public class TeleOp2Auto extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor gripMotor;
    private DcMotor armMotor;
    private Servo leftServo;
    private Servo rightServo;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        imu             = hardwareMap.get(BNO055IMU.class, "imu");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        gripMotor       = hardwareMap.get(DcMotor.class, "gripMotor");
        armMotor        = hardwareMap.get(DcMotor.class, "armMotor");
        leftServo       = hardwareMap.get(Servo.class, "leftServo");
        rightServo      = hardwareMap.get(Servo.class, "rightServo");
        colorSensor     = hardwareMap.get(ColorSensor.class,"colorSensor");

        // set motor direction
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gripMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // set motor zero power behavior
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize the hook
        leftServo.setPosition(0.37);
        rightServo.setPosition(0.7);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for the game to start
        waitForStart();

        double driveAxial       = 0;
        double driveLateral     = 0;
        double driveYaw         = 0;
        double gripPower        = 0;
        double armPower         = 0;
        double leftServoState   = 0.37;
        double rightServoState  = 0.7;

        // set up the elapsed timer
        ElapsedTime timer = new ElapsedTime();
        double elapsedTime = timer.time();
        Runtime run = Runtime.getRuntime();
        // set up the csv writer
        String content = "elapsed time,driveAxial,driveLateral,driveYaw";
        StringBuilder jihad = new StringBuilder();
        try (FileWriter writer = new FileWriter("TeleOp2Auto.csv", true);
             // if file doesn't exist, create a file and write "content" to the file

            // else, purge the file and write "content" to the file.

        } catch (IOException e) {
            System.err.format("IOException: %s%n", e);
        }

        // run until the end of the match
        while (opModeIsActive()) {
            // assign controller power values
            driveAxial      = 0;
            driveLateral    = 0;
            driveYaw        = 0;
            armPower        = this.gamepad2.left_stick_y;
            gripPower       = 0;

            if (this.gamepad1.right_bumper) {
                // hook on
                leftServoState = 1;
                rightServoState = 1;
            } else if (this.gamepad1.left_bumper) {
                // hook off
                leftServoState = 0.35;
                rightServoState = 0.61;
            }

            if (this.gamepad2.right_bumper) {
                // grip hold
                gripPower = 0.3;
            } else if (this.gamepad2.left_bumper) {
                // grip release
                gripPower = -0.3;
            }

            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0) {
                // dpad_left = slow left
                if (gamepad1.dpad_left) {
                    driveLateral = -0.6;
                }
                // dpad_right = slow right
                if (gamepad1.dpad_right) {
                    driveLateral = 0.6;
                }
                // dpad_up = slow forward
                if (gamepad1.dpad_up) {
                    driveAxial = -0.3;
                }
                // dpad_down = slow backward
                if (gamepad1.dpad_down) {
                    driveAxial = 0.3;
                }
                // x = slow rotate ccw
                if (gamepad1.x) {
                    driveYaw = -0.35;
                }
                // b = slow rotate cw
                if (gamepad1.b) {
                    driveYaw = 0.35;
                }
            }
            else {
                // set axial movement to 0.5 if the stick value is less than 0.75
                if (Math.abs(this.gamepad1.left_stick_y) > 0.75) {
                    driveAxial = this.gamepad1.left_stick_y;
                }
                else {
                    driveAxial = 0.5 * this.gamepad1.left_stick_y;
                }
                // set axial movement to 0 if lateral movement is greater than 0.25
                if (Math.abs(this.gamepad1.left_stick_x) > 0.25 && Math.abs(this.gamepad1.left_stick_y) < 0.5) {
                    driveAxial = 0;
                    driveLateral = this.gamepad1.left_stick_x;
                }
                else {
                    driveLateral = this.gamepad1.left_stick_x;
                }
                // set yaw movement to half if the stick value is less than 0.75
                if (Math.abs(this.gamepad1.right_stick_x) > 0.75) {
                    driveYaw = this.gamepad1.right_stick_x;
                }
                else {
                    driveYaw = 0.5 * this.gamepad1.right_stick_x;
                }
            }

            leftBackMotor.setPower(-driveLateral - driveAxial + driveYaw);
            rightBackMotor.setPower(driveLateral - driveAxial - driveYaw);
            leftFrontMotor.setPower(driveLateral - driveAxial + driveYaw);
            rightFrontMotor.setPower(-driveLateral - driveAxial - driveYaw);
            gripMotor.setPower(gripPower);
            armMotor.setPower(armPower);

            leftServo.setPosition(leftServoState);
            rightServo.setPosition(rightServoState);

            telemetry.addData("Status", "Running");
            telemetry.addData("driveAxial", driveAxial);
            telemetry.addData("driveLateral", driveLateral);
            telemetry.addData("driveYaw", driveYaw);
            telemetry.update();

            // append each variable to the csv file: elapsedTime, driveAxial, driveLatearl, driveYaw

        }
        // close and export the file
    }
}
