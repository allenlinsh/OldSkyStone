package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class MainTeleOp extends LinearOpMode {
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
        // run until the end of the match
        while (opModeIsActive()) {
            // assign controller power values
            driveAxial      = gamepad1.left_stick_y;
            driveLateral    = gamepad1.left_stick_x;
            driveYaw        = gamepad1.right_stick_x;
            armPower        = this.gamepad2.left_stick_y;
            gripPower       = 0;

            if (this.gamepad1.right_bumper) {
                // hook on
                leftServoState = 1;
                rightServoState = 1;
            }
            else if (this.gamepad1.left_bumper) {
                // hook off
                leftServoState = 0.35;
                rightServoState = 0.61;
            }

            if (this.gamepad2.right_bumper) {
                // grip hold
                gripPower = 0.3;
            }
            else if (this.gamepad2.left_bumper) {
                // grip release
                gripPower = -0.3;
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
            telemetry.addData("armPos", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
