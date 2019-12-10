package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TeleOp2AutoLog extends LinearOpMode {
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
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0, correction;
    double elapsedTime;

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
        hookOff();

        // initialize imu
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double driveAxial       = 0;
        double driveLateral     = 0;
        double driveYaw         = 0;
        double gripPower        = 0;
        double armPower         = 0;

        double leftBackPower = 0;
        double rightBackPower = 0;
        double leftFrontPower = 0;
        double rightFrontPower = 0;

        // set up the elapsed timer
        ElapsedTime timer = new ElapsedTime();

        // adding header to log
        String header = "elapsedTime, leftBackPower, rightBackPower, leftFrontPower, rightFrontPower, gripPower, armPower";
        RobotLog.v(header);

        // wait for the game to start
        waitForStart();

        // run until the end of the match
        while (opModeIsActive()) {
            elapsedTime = timer.time();

            // Use gyro to drive in a straight line.
            if (gamepad1.right_stick_x != 0 || gamepad1.x || gamepad1.b){
                resetAngle();
                correction = 0;
            }
            else {
                correction = checkDirection();
            }

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 leftBackPower", leftBackPower);
            telemetry.addData("5 rightBackPower", rightBackPower);
            telemetry.addData("6 leftFrontPower", leftFrontPower);
            telemetry.addData("7 rightFrontPower", rightBackPower);
            telemetry.addData("8 gripPower", gripPower);
            telemetry.addData("9 armPower", armPower);
            telemetry.update();

            // assign controller power values
            driveAxial      = 0;
            driveLateral    = 0;
            driveYaw        = 0;
            armPower        = this.gamepad2.left_stick_y;
            gripPower       = 0;

            if (this.gamepad1.right_bumper) {
                hookOn();
            } else if (this.gamepad1.left_bumper) {
                hookOff();
            }

            if (this.gamepad2.right_bumper) {
                // grip hold
                gripPower = 0.3;
            } else if (this.gamepad2.left_bumper) {
                // grip release
                gripPower = -0.3;
            }

            if (this.gamepad1.left_stick_y == 0 && this.gamepad1.left_stick_x == 0 && this.gamepad1.right_stick_x == 0) {
                // dpad_left = slow left
                if (gamepad1.dpad_left) {
                    driveLateral = -0.5;
                }
                // dpad_right = slow right
                if (gamepad1.dpad_right) {
                    driveLateral = 0.5;
                }
                // dpad_up = slow forward
                if (gamepad1.dpad_up) {
                    driveAxial = -0.25;
                }
                // dpad_down = slow backward
                if (gamepad1.dpad_down) {
                    driveAxial = 0.25;
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
                // set axial movement to logarithmic values and set a dead zone
                driveAxial = this.gamepad1.left_stick_y;
                if (Math.abs(driveAxial) < Math.sqrt(0.1)) {
                    driveAxial = 0;
                }
                else {
                    driveAxial = driveAxial * 110 /127;
                    driveAxial = driveAxial * driveAxial * Math.signum(driveAxial) / 1.0;
                }
                // set lateral movement to logarithmic values and set a dead zone
                driveLateral = this.gamepad1.left_stick_x;
                if (Math.abs(driveLateral) < Math.sqrt(0.1)) {
                    driveLateral = 0;
                }
                else {
                    driveLateral = driveLateral * 100 / 127;
                    driveLateral = driveLateral * driveLateral * Math.signum(driveLateral) / 1.0;
                }
                // set yaw movement to logarithmic values and set a dead zone
                driveYaw = this.gamepad1.right_stick_x;
                if (Math.abs(driveYaw) < Math.sqrt(0.1)) {
                    driveYaw = 0;
                }
                else {
                    driveYaw = driveYaw * 110 / 127;
                    driveYaw = driveYaw * driveYaw * Math.signum(driveYaw) / 1.0;
                }
            }

            leftBackPower = -driveLateral - driveAxial + driveYaw - correction;
            rightBackPower = driveLateral - driveAxial - driveYaw + correction;
            leftFrontPower = driveLateral - driveAxial + driveYaw - correction;
            rightFrontPower = -driveLateral - driveAxial - driveYaw + correction;

            if (this.gamepad1.left_stick_y != 0 || this.gamepad1.left_stick_x != 0 || this.gamepad1.right_stick_x != 0 || this. gamepad1.dpad_up || this. gamepad1.dpad_down || this. gamepad1.dpad_left || this. gamepad1.dpad_right || this. gamepad1.x || this. gamepad1.b) {
                leftBackMotor.setPower(leftBackPower);
                rightBackMotor.setPower(rightBackPower);
                leftFrontMotor.setPower(leftFrontPower);
                rightFrontMotor.setPower(rightFrontPower);
            }
            else {
                leftBackMotor.setPower(0);
                rightBackMotor.setPower(0);
                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);
            }
            gripMotor.setPower(gripPower);
            armMotor.setPower(armPower);

            // adding motor values to csv
            String values = elapsedTime + ", " + leftBackPower + ", " + rightBackPower + ", " + leftFrontPower + ", " + rightFrontPower + ", " + gripPower + ", " + armPower;
            RobotLog.v(values);
        }
    }
    public void hookOn() {
        leftServo.setPosition(1);
        rightServo.setPosition(0);
    }
    public void hookOff() {
        leftServo.setPosition(0.1);
        rightServo.setPosition(0.9);
    }
    // resets the cumulative angle tracking to zero.
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private double getAngle()
    {
        // z axis is the axis for heading angle.
        // convert the euler angles to relative angles

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        double correction, angle, gain = 0.02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}
