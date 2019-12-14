package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class DumbAutonomous extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor gripMotor;
    private DcMotor armMotor;
    private Servo leftServo;
    private Servo rightServo;
    private Servo leftSkystoneServo;
    private Servo rightSkystoneServo;
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;
    private DigitalChannel topLimit, bottomLimit;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.4, correction;
    double slowPower = 0.25;
    double leftColorThreshold, rightColorThreshold;
    int currPos, startPos, endPos;
    double slowFactor = 0.25;
    double leftServoState, rightServoState, leftSkystoneServoState, rightSkystoneServoState;
    double leftBackPower, rightBackPower, leftFrontPower, rightFrontPower = 0;
    boolean runAutonomous = true;
    int tempRunTime;

    double lateralPower         = 0.25;
    double timePerInchLateral   = 1225;
    double axialPower           = lateralPower * 2;
    double timePerInchAxial     = 1225;
    double turnPower            = 0.5;
    double timePerDegreeCCW     = 11.15;
    double timePerDegreeCW      = 11.6;
    double dragPower            = lateralPower * 1.5;

    int ticksPerRev             = 480;
    String alliance             = "red";
    String mode                 = "foundation";

    @Override
    public void runOpMode() {
        imu                     = hardwareMap.get(BNO055IMU.class, "imu");
        leftBackMotor           = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor          = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftFrontMotor          = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor         = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        gripMotor               = hardwareMap.get(DcMotor.class, "gripMotor");
        armMotor                = hardwareMap.get(DcMotor.class, "armMotor");
        leftServo               = hardwareMap.get(Servo.class, "leftServo");
        rightServo              = hardwareMap.get(Servo.class, "rightServo");
        leftSkystoneServo       = hardwareMap.get(Servo.class, "leftSkystoneServo");
        rightSkystoneServo      = hardwareMap.get(Servo.class, "rightSkystoneServo");
        leftColorSensor         = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor        = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        topLimit                = hardwareMap.get(DigitalChannel.class, "topLimit");
        bottomLimit             = hardwareMap.get(DigitalChannel.class, "bottomLimit");

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

        // initialize servos
        hookOff();
        leftSkystoneServoState = 0.52;
        rightSkystoneServoState = 0.98;
        leftSkystoneServo.setPosition(leftSkystoneServoState);
        rightSkystoneServo.setPosition(rightSkystoneServoState);

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
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Mode", mode);
        telemetry.update();

        // wait for the game to start
        waitForStart();

        telemetry.addData("Status", "Running");

        while (opModeIsActive()) {
            leftColorThreshold = leftColorSensor.red() * leftColorSensor.green() / (leftColorSensor.blue() * leftColorSensor.blue());
            rightColorThreshold = rightColorSensor.red() * rightColorSensor.green() / (rightColorSensor.blue() * rightColorSensor.blue());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 power", power);
            telemetry.addData("left color", leftColorThreshold);
            telemetry.addData("right color", rightColorThreshold);
            telemetry.update();

            switch (alliance) {
                case "blue":
                    switch (mode) {
                        case "full":

                        case "foundation":
                            driveRight(0.5, axialPower);
                            // driveBackward slow
                            leftBackMotor.setPower(-lateralPower * 0.8);
                            rightBackMotor.setPower(-lateralPower * 0.8);
                            leftFrontMotor.setPower(-lateralPower * 0.8);
                            rightFrontMotor.setPower(-lateralPower * 0.8);
                            tempRunTime = (int)(1.15 * timePerInchAxial * 2);
                            sleep(tempRunTime);
                            stopMotor();
                            //end here
                            hookOn();
                            stopMotor();
                            sleep(1000);
                            driveForward(1.6, dragPower);
                            hookOff();
                            stopMotor();
                            sleep(1000);
                            driveLeft(1.5, axialPower);
                            driveBackward(0.75, lateralPower);
                            driveRight(0.75, axialPower);
                            driveBackward(0.5, lateralPower);
                            driveLeft(1, axialPower);
                            stopMotor();
                            sleep(30000);
                        case "skystone":
                            driveForward(1, lateralPower);
                            // driveLeft slow
                            leftBackMotor.setPower(axialPower * 0.8);
                            rightBackMotor.setPower(-axialPower * 0.8);
                            leftFrontMotor.setPower(-axialPower * 0.8);
                            rightFrontMotor.setPower(axialPower * 0.8);
                            tempRunTime = (int)(1 * timePerInchAxial * 1.6);
                            sleep(tempRunTime);
                            stopMotor();
                            // end here
                            driveForward(0.25, lateralPower);
                            driveLeft(0.75, axialPower * 0.8);
                            leftSkystoneOn();
                            stopMotor();
                            sleep(2000);
                            driveRight(0.75, axialPower);
                            driveForward(0.25, lateralPower);
                            driveBackward(3.25, lateralPower);
                            leftSkystoneOff();
                            stopMotor();
                            sleep(2000);
                            driveForward(2.9, lateralPower);
                            stopMotor();
                            sleep(1000);
                            driveBackward(1.15, lateralPower);
                            // driveLeft slow
                            leftBackMotor.setPower(axialPower * 0.8);
                            rightBackMotor.setPower(-axialPower * 0.8);
                            leftFrontMotor.setPower(-axialPower * 0.8);
                            rightFrontMotor.setPower(axialPower * 0.8);
                            tempRunTime = (int)(0.75 * timePerInchAxial * 1.6);
                            sleep(tempRunTime);
                            stopMotor();
                            // end here
                            leftSkystoneOn();
                            stopMotor();
                            sleep(1000);
                            driveRight(0.9, axialPower);
                            driveForward(1.75, lateralPower);
                            driveBackward(3, (lateralPower + 0.15));
                            driveForward(0.25, lateralPower);
                            stopMotor();
                            sleep(30000);
                        case "park close":
                            //stopMotor();
                            //sleep(20000);
                            driveBackward(1.5, lateralPower);
                            stopMotor();
                            sleep(30000);
                        case "park far":
                            //stopMotor();
                            //sleep(20000);
                            driveLeft(1.2, axialPower);
                            driveBackward(1.5, lateralPower);
                            stopMotor();
                            sleep(30000);
                    }
                case "red":
                    switch (mode) {
                        case "full":

                        case "foundation":
                            driveLeft(0.5, axialPower);
                            // driveBackward slow
                            leftBackMotor.setPower(-lateralPower * 0.8);
                            rightBackMotor.setPower(-lateralPower * 0.8);
                            leftFrontMotor.setPower(-lateralPower * 0.8);
                            rightFrontMotor.setPower(-lateralPower * 0.8);
                            int tempRunTime = (int)(1.15 * timePerInchAxial * 2);
                            sleep(tempRunTime);
                            stopMotor();
                            // end here
                            hookOn();
                            stopMotor();
                            sleep(1000);
                            driveForward(1.6, dragPower);
                            hookOff();
                            stopMotor();
                            sleep(1000);
                            driveRight(1.5, axialPower);
                            driveBackward(0.75, lateralPower);
                            driveLeft(0.75, axialPower);
                            driveBackward(0.5, lateralPower);
                            driveRight(1, axialPower);
                            stopMotor();
                            sleep(30000);
                        case "skystone":
                            driveForward(1, lateralPower);
                            driveRight(1.25, axialPower);
                            rightSkystoneOn();
                            driveLeft(0.75, axialPower);
                            driveBackward(2, lateralPower);
                            rightSkystoneOff();
                            sleep(3000);
                        case "park":
                            runAutonomous = false;
                    }
            }
        }
        stopMotor();
    }

    public void motorPause() {
        sleep(250);
    }
    public void servoPause() {
        sleep(500);
    }
    public void hookOn() {
        leftServoState = 1;
        rightServoState = 0;
        leftServo.setPosition(leftServoState);
        rightServo.setPosition(rightServoState);
        while (leftServo.getPosition()  != 1) {
            stopMotor();
        }
    }
    public void hookOff() {
        leftServoState = 0.1;
        rightServoState = 0.9;
        leftServo.setPosition(leftServoState);
        rightServo.setPosition(rightServoState);
        while (leftServo.getPosition() != 0.1) {
            stopMotor();
        }
    }
    public void leftSkystoneOn() {
        leftSkystoneServoState = 0.98;
        leftSkystoneServo.setPosition(leftSkystoneServoState);
        while (leftSkystoneServo.getPosition() != 0.98) {
            stopMotor();
        }
    }
    public void rightSkystoneOn() {
        rightSkystoneServoState = 0.52;
        rightSkystoneServo.setPosition(rightSkystoneServoState);
        while (rightSkystoneServo.getPosition() != 0.52) {
            stopMotor();
        }
    }
    public void leftSkystoneOff() {
        leftSkystoneServoState = 0.52;
        leftSkystoneServo.setPosition(leftSkystoneServoState);
        while (leftSkystoneServo.getPosition() != 0.52) {
            stopMotor();
        }
    }
    public void rightSkystoneOff() {
        rightSkystoneServoState = 0.98;
        rightSkystoneServo.setPosition(rightSkystoneServoState);
        while (rightSkystoneServo.getPosition() != 0.98) {
            stopMotor();
        }
    }
    public boolean topPressed() {
        return !topLimit.getState();
    }
    public boolean bottomPressed() {
        return !bottomLimit.getState();
    }
    public void gripHold() {
        gripMotor.setPower(0.3);
    }
    public void gripRelease() {
        gripMotor.setPower(-0.3);
        sleep(300);
        gripMotor.setPower(0);
    }
    public void stopMotor() {
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }
    public void armBackward() {
        armMotor.setPower(0.25);
        while (opModeIsActive() && armMotor.isBusy()) {
            if (topPressed()) {
                armMotor.setPower(0);
            }
            else {
                armMotor.setPower(1);
            }
        }
    }
    public void armForward() {
        armMotor.setPower(-0.25);
        while (opModeIsActive() && armMotor.isBusy()) {
            if (bottomPressed()) {
                armMotor.setPower(0);
            }
            else {
                armMotor.setPower(-1);
            }
        }
    }
    public void driveForward(double block, double power) {
        int runTime = (int)(block * timePerInchLateral);

        // set motor speed
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        sleep(runTime);

        stopMotor();
    }
    public void driveBackward(double block, double power) {
        driveForward(block, -power);
    }
    public void driveLeft(double block, double power) {
        int runTime = (int)(block * timePerInchAxial);

        // set motor speed
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(-power);
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        sleep(runTime);

        stopMotor();
    }
    public void driveRight(double block, double power) {
        driveLeft(block, -power);
    }
    // resets the cumulative angle tracking to zero.
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public void turn(int angle, double power) {
        double leftPower, rightPower;
        int runTimeCW   = (int)(Math.abs(angle) * timePerDegreeCW);
        int runTimeCCW  = (int)(Math.abs(angle) * timePerDegreeCCW);

        if (angle < 0)
        {   // turn counterclockwise.
            leftPower   = -power;
            rightPower  = power;

            // set power to rotate.
            leftBackMotor.setPower(leftPower);
            leftFrontMotor.setPower(leftPower);
            rightBackMotor.setPower(rightPower);
            rightFrontMotor.setPower(rightPower);
            sleep(runTimeCCW);
        }
        else if (angle > 0)
        {   // turn clockwise.
            leftPower   = power;
            rightPower  = -power;

            // set power to rotate.
            leftBackMotor.setPower(leftPower);
            leftFrontMotor.setPower(leftPower);
            rightBackMotor.setPower(rightPower);
            rightFrontMotor.setPower(rightPower);
            sleep(runTimeCW);
        }

        stopMotor();
        motorPause();
    }
}
