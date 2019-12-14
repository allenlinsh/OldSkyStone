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
public class TestTele2Auto extends LinearOpMode {
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
    double globalAngle, power = 0.2, correction;
    double elapsedTime, endTime;
    double leftServoState, rightServoState, leftSkystoneServoState, rightSkystoneServoState;
    boolean startAutonomous = true;

    int ticksPerRev             = 480;
    String alliance             = "blue";

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
        leftSkystoneOff();
        rightSkystoneOff();

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

        // set up the elapsed timer
        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for the game to start
        waitForStart();
        timer.reset();

        telemetry.addData("Status", "Running");
        telemetry.addData("Alliance", alliance);

        while (opModeIsActive() && startAutonomous) {
            elapsedTime = timer.time();

            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("0 elapsed time", elapsedTime);
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 power", power);
            telemetry.update();

            if (startAutonomous) { // https://stemrobotics.cs.pdx.edu/node/5184
                teleOp2Auto();
                if(elapsedTime > endTime) {
                    startAutonomous = false;
                }
            }
        }
        stopMotor();
    }
    public void pause() {
        sleep(100);
    }
    public void motorPause() {
        sleep(750);
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
        servoPause();
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
        double power = 0.3;
        gripMotor.setPower(power);
        sleep(400);
        gripMotor.setPower(0);
        pause();
    }
    public void gripRelease() {
        double power = -0.3;
        gripMotor.setPower(power);
        sleep(300);
        gripMotor.setPower(0);
        pause();
    }
    public void stopMotor() {
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }
    public void armBackward() {
        double power = 1;
        // set motor speed
        armMotor.setPower(power);
        sleep(1600);
        armMotor.setPower(0);
        pause();
    }
    public void armForward() {
        double power = -1;
        // set motor speed
        armMotor.setPower(power);
        sleep(500);
        armMotor.setPower(0);
        pause();
    }
    public void driveForward(double block, double power, double correction) {
        power = power * 100 / 127;

        // calculate target position
        double circumference = Math.PI * 3.75; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(block * 24 / inPerRev);

        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setTargetPosition(targetPos);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor speed
        leftBackMotor.setPower(power - correction);
        rightBackMotor.setPower(power + correction);
        leftFrontMotor.setPower(power - correction);
        rightFrontMotor.setPower(power + correction);

        while(opModeIsActive() && leftBackMotor.isBusy()) {}
        stopMotor();
        pause();
    }
    public void driveBackward(double block, double power, double correction) {
        power = power * 100 / 127;

        // calculate target position
        double circumference = Math.PI * 3.75; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(block * 24 / inPerRev);

        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setTargetPosition(-targetPos);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor speed
        leftBackMotor.setPower(-power - correction);
        rightBackMotor.setPower(-power + correction);
        leftFrontMotor.setPower(-power - correction);
        rightFrontMotor.setPower(-power + correction);

        while(opModeIsActive() && leftBackMotor.isBusy()) {}
        stopMotor();
        pause();
    }
    public void driveLeft(double block, double power, double correction) {
        power = power * 100 / 127;

        // calculate target position
        double circumference = Math.PI * 3.75; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(block * 24 / inPerRev * 7 / 6);

        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackMotor.setTargetPosition(targetPos);
        rightBackMotor.setTargetPosition(-targetPos);
        leftFrontMotor.setTargetPosition(-targetPos);
        rightFrontMotor.setTargetPosition(targetPos);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor speed
        leftBackMotor.setPower(power - correction);
        rightBackMotor.setPower(-power + correction);
        leftFrontMotor.setPower(-power - correction);
        rightFrontMotor.setPower(power + correction);

        while(opModeIsActive() && leftBackMotor.isBusy()) {}
        stopMotor();
        pause();
    }
    public void driveRight(double block, double power, double correction) {
        power = power * 100 / 127;

        // calculate target position
        double circumference = Math.PI * 3.75; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(block * 24 / inPerRev * 7 / 6);

        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackMotor.setTargetPosition(-targetPos);
        rightBackMotor.setTargetPosition(targetPos);
        leftFrontMotor.setTargetPosition(targetPos);
        rightFrontMotor.setTargetPosition(-targetPos);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor speed
        leftBackMotor.setPower(power - correction);
        rightBackMotor.setPower(-power + correction);
        leftFrontMotor.setPower(-power - correction);
        rightFrontMotor.setPower(power + correction);

        while(opModeIsActive() && leftBackMotor.isBusy()) {}
        stopMotor();
        pause();
    }
    // resets the cumulative angle tracking to zero.
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private double getAngle() {
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
        double correction, angle, gain = .025;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    // rotate clockwise(+) or counterclockwise(-). operates in the range -180 to 180
    private void rotate(int degrees, double power) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        if (degrees < 0) {
            // turn counterclockwise.
            leftPower   = -power;
            rightPower  = power;
        }
        else if (degrees > 0) {
            // turn clockwise.
            leftPower   = power;
            rightPower  = -power;
        }
        else return;

        // set power to rotate.
        leftBackMotor.setPower(leftPower);
        rightBackMotor.setPower(rightPower);
        leftFrontMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        stopMotor();
        pause();

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void teleOp2Auto() {
        if(elapsedTime > 0 && elapsedTime < 0.746566637) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 0.746566637 && elapsedTime < 0.780578098) {
            leftBackMotor.setPower(0.4898928970762456);
            rightBackMotor.setPower(0.4898928970762456);
            leftFrontMotor.setPower(0.4898928970762456);
            rightFrontMotor.setPower(0.4898928970762456);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 0.780578098 && elapsedTime < 0.83172102) {
            leftBackMotor.setPower(0.7502015004030008);
            rightBackMotor.setPower(0.7502015004030008);
            leftFrontMotor.setPower(0.7502015004030008);
            rightFrontMotor.setPower(0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 0.83172102 && elapsedTime < 0.855830554) {
            leftBackMotor.setPower(0.7489515004030008);
            rightBackMotor.setPower(0.7514515004030008);
            leftFrontMotor.setPower(0.7489515004030008);
            rightFrontMotor.setPower(0.7514515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 0.855830554 && elapsedTime < 0.879561962) {
            leftBackMotor.setPower(0.7477015004030009);
            rightBackMotor.setPower(0.7527015004030008);
            leftFrontMotor.setPower(0.7477015004030009);
            rightFrontMotor.setPower(0.7527015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 0.879561962 && elapsedTime < 0.906602955) {
            leftBackMotor.setPower(0.7452015004030008);
            rightBackMotor.setPower(0.7552015004030008);
            leftFrontMotor.setPower(0.7452015004030008);
            rightFrontMotor.setPower(0.7552015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 0.906602955 && elapsedTime < 0.932162384) {
            leftBackMotor.setPower(0.7389515004030008);
            rightBackMotor.setPower(0.7614515004030008);
            leftFrontMotor.setPower(0.7389515004030008);
            rightFrontMotor.setPower(0.7614515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 0.932162384 && elapsedTime < 0.960068481) {
            leftBackMotor.setPower(0.7352015004030008);
            rightBackMotor.setPower(0.7652015004030008);
            leftFrontMotor.setPower(0.7352015004030008);
            rightFrontMotor.setPower(0.7652015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 0.960068481 && elapsedTime < 0.984815879) {
            leftBackMotor.setPower(0.7264515004030008);
            rightBackMotor.setPower(0.7739515004030009);
            leftFrontMotor.setPower(0.7264515004030008);
            rightFrontMotor.setPower(0.7739515004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 0.984815879 && elapsedTime < 1.013046403) {
            leftBackMotor.setPower(0.7189515004030008);
            rightBackMotor.setPower(0.7814515004030008);
            leftFrontMotor.setPower(0.7189515004030008);
            rightFrontMotor.setPower(0.7814515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.013046403 && elapsedTime < 1.041231874) {
            leftBackMotor.setPower(0.7102015004030008);
            rightBackMotor.setPower(0.7902015004030009);
            leftFrontMotor.setPower(0.7102015004030008);
            rightFrontMotor.setPower(0.7902015004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.041231874 && elapsedTime < 1.136392717) {
            leftBackMotor.setPower(0.7027015004030008);
            rightBackMotor.setPower(0.7977015004030008);
            leftFrontMotor.setPower(0.7027015004030008);
            rightFrontMotor.setPower(0.7977015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.136392717 && elapsedTime < 1.16230673) {
            leftBackMotor.setPower(-0.07375);
            rightBackMotor.setPower(0.07375);
            leftFrontMotor.setPower(-0.07375);
            rightFrontMotor.setPower(0.07375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 1.16230673 && elapsedTime < 1.188347202) {
            leftBackMotor.setPower(-0.08125);
            rightBackMotor.setPower(0.08125);
            leftFrontMotor.setPower(-0.08125);
            rightFrontMotor.setPower(0.08125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.188347202 && elapsedTime < 1.201206786) {
            leftBackMotor.setPower(-0.08625000000000001);
            rightBackMotor.setPower(0.08625000000000001);
            leftFrontMotor.setPower(-0.08625000000000001);
            rightFrontMotor.setPower(0.08625000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.201206786 && elapsedTime < 1.214106892) {
            leftBackMotor.setPower(-0.09);
            rightBackMotor.setPower(0.09);
            leftFrontMotor.setPower(-0.09);
            rightFrontMotor.setPower(0.09);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.214106892 && elapsedTime < 1.230165279) {
            leftBackMotor.setPower(-0.08625000000000001);
            rightBackMotor.setPower(0.08625000000000001);
            leftFrontMotor.setPower(-0.08625000000000001);
            rightFrontMotor.setPower(0.08625000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 1.230165279 && elapsedTime < 1.254832781) {
            leftBackMotor.setPower(-0.08750000000000001);
            rightBackMotor.setPower(0.08750000000000001);
            leftFrontMotor.setPower(-0.08750000000000001);
            rightFrontMotor.setPower(0.08750000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 1.254832781 && elapsedTime < 1.2803432) {
            leftBackMotor.setPower(-0.08625000000000001);
            rightBackMotor.setPower(0.08625000000000001);
            leftFrontMotor.setPower(-0.08625000000000001);
            rightFrontMotor.setPower(0.08625000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.2803432 && elapsedTime < 1.292129399) {
            leftBackMotor.setPower(-0.0925);
            rightBackMotor.setPower(0.0925);
            leftFrontMotor.setPower(-0.0925);
            rightFrontMotor.setPower(0.0925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.292129399 && elapsedTime < 1.30809763) {
            leftBackMotor.setPower(-0.09375);
            rightBackMotor.setPower(0.09375);
            leftFrontMotor.setPower(-0.09375);
            rightFrontMotor.setPower(0.09375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.30809763 && elapsedTime < 1.321986017) {
            leftBackMotor.setPower(-0.09625);
            rightBackMotor.setPower(0.09625);
            leftFrontMotor.setPower(-0.09625);
            rightFrontMotor.setPower(0.09625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 1.321986017 && elapsedTime < 1.356956593) {
            leftBackMotor.setPower(-0.09875);
            rightBackMotor.setPower(0.09875);
            leftFrontMotor.setPower(-0.09875);
            rightFrontMotor.setPower(0.09875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.356956593 && elapsedTime < 1.368961959) {
            leftBackMotor.setPower(-0.1);
            rightBackMotor.setPower(0.1);
            leftFrontMotor.setPower(-0.1);
            rightFrontMotor.setPower(0.1);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.368961959 && elapsedTime < 1.380460398) {
            leftBackMotor.setPower(-0.10375000000000001);
            rightBackMotor.setPower(0.10375000000000001);
            leftFrontMotor.setPower(-0.10375000000000001);
            rightFrontMotor.setPower(0.10375000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.380460398 && elapsedTime < 1.391685972) {
            leftBackMotor.setPower(-0.105);
            rightBackMotor.setPower(0.105);
            leftFrontMotor.setPower(-0.105);
            rightFrontMotor.setPower(0.105);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.391685972 && elapsedTime < 1.405963838) {
            leftBackMotor.setPower(-0.1075);
            rightBackMotor.setPower(0.1075);
            leftFrontMotor.setPower(-0.1075);
            rightFrontMotor.setPower(0.1075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.405963838 && elapsedTime < 1.419262381) {
            leftBackMotor.setPower(-0.10875);
            rightBackMotor.setPower(0.10875);
            leftFrontMotor.setPower(-0.10875);
            rightFrontMotor.setPower(0.10875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.419262381 && elapsedTime < 1.444896394) {
            leftBackMotor.setPower(-0.23083198814919587);
            rightBackMotor.setPower(-0.008331988149195863);
            leftFrontMotor.setPower(-0.23083198814919587);
            rightFrontMotor.setPower(-0.008331988149195863);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.444896394 && elapsedTime < 1.471627647) {
            leftBackMotor.setPower(-0.23458198814919587);
            rightBackMotor.setPower(-0.004581988149195859);
            leftFrontMotor.setPower(-0.23458198814919587);
            rightFrontMotor.setPower(-0.004581988149195859);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.471627647 && elapsedTime < 1.498975878) {
            leftBackMotor.setPower(-0.7237442507991073);
            rightBackMotor.setPower(-0.49124425079910733);
            leftFrontMotor.setPower(-0.7237442507991073);
            rightFrontMotor.setPower(-0.49124425079910733);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.498975878 && elapsedTime < 1.52598286) {
            leftBackMotor.setPower(-0.8639515004030008);
            rightBackMotor.setPower(-0.6364515004030008);
            leftFrontMotor.setPower(-0.8639515004030008);
            rightFrontMotor.setPower(-0.6364515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.52598286 && elapsedTime < 1.550427342) {
            leftBackMotor.setPower(-0.8627015004030009);
            rightBackMotor.setPower(-0.6377015004030008);
            leftFrontMotor.setPower(-0.8627015004030009);
            rightFrontMotor.setPower(-0.6377015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.550427342 && elapsedTime < 1.577171094) {
            leftBackMotor.setPower(-0.8602015004030008);
            rightBackMotor.setPower(-0.6402015004030008);
            leftFrontMotor.setPower(-0.8602015004030008);
            rightFrontMotor.setPower(-0.6402015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.577171094 && elapsedTime < 1.667180999) {
            leftBackMotor.setPower(-0.8539515004030008);
            rightBackMotor.setPower(-0.6464515004030008);
            leftFrontMotor.setPower(-0.8539515004030008);
            rightFrontMotor.setPower(-0.6464515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.667180999 && elapsedTime < 1.692808398) {
            leftBackMotor.setPower(-0.8502015004030008);
            rightBackMotor.setPower(-0.6502015004030008);
            leftFrontMotor.setPower(-0.8502015004030008);
            rightFrontMotor.setPower(-0.6502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.692808398 && elapsedTime < 1.721561265) {
            leftBackMotor.setPower(-0.8552015004030008);
            rightBackMotor.setPower(-0.6452015004030008);
            leftFrontMotor.setPower(-0.8552015004030008);
            rightFrontMotor.setPower(-0.6452015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.721561265 && elapsedTime < 1.748785278) {
            leftBackMotor.setPower(-0.8564515004030008);
            rightBackMotor.setPower(-0.6439515004030009);
            leftFrontMotor.setPower(-0.8564515004030008);
            rightFrontMotor.setPower(-0.6439515004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.748785278 && elapsedTime < 1.774233927) {
            leftBackMotor.setPower(-0.8489515004030008);
            rightBackMotor.setPower(-0.6514515004030008);
            leftFrontMotor.setPower(-0.8489515004030008);
            rightFrontMotor.setPower(-0.6514515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.774233927 && elapsedTime < 1.805586482) {
            leftBackMotor.setPower(-0.8552015004030008);
            rightBackMotor.setPower(-0.6452015004030008);
            leftFrontMotor.setPower(-0.8552015004030008);
            rightFrontMotor.setPower(-0.6452015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.805586482 && elapsedTime < 1.839074923) {
            leftBackMotor.setPower(-0.7225620091307232);
            rightBackMotor.setPower(-0.5150620091307232);
            leftFrontMotor.setPower(-0.7225620091307232);
            rightFrontMotor.setPower(-0.5150620091307232);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.839074923 && elapsedTime < 1.863446644) {
            leftBackMotor.setPower(-0.09125);
            rightBackMotor.setPower(0.09125);
            leftFrontMotor.setPower(-0.09125);
            rightFrontMotor.setPower(0.09125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.863446644 && elapsedTime < 1.889461334) {
            leftBackMotor.setPower(-0.08375);
            rightBackMotor.setPower(0.08375);
            leftFrontMotor.setPower(-0.08375);
            rightFrontMotor.setPower(0.08375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.889461334 && elapsedTime < 1.903379773) {
            leftBackMotor.setPower(-0.08125);
            rightBackMotor.setPower(0.08125);
            leftFrontMotor.setPower(-0.08125);
            rightFrontMotor.setPower(0.08125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.903379773 && elapsedTime < 1.913701909) {
            leftBackMotor.setPower(-0.0725);
            rightBackMotor.setPower(0.0725);
            leftFrontMotor.setPower(-0.0725);
            rightFrontMotor.setPower(0.0725);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.913701909 && elapsedTime < 1.926581181) {
            leftBackMotor.setPower(-0.065);
            rightBackMotor.setPower(0.065);
            leftFrontMotor.setPower(-0.065);
            rightFrontMotor.setPower(0.065);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.926581181 && elapsedTime < 1.940793527) {
            leftBackMotor.setPower(-0.06);
            rightBackMotor.setPower(0.06);
            leftFrontMotor.setPower(-0.06);
            rightFrontMotor.setPower(0.06);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.940793527 && elapsedTime < 1.953417434) {
            leftBackMotor.setPower(-0.051250000000000004);
            rightBackMotor.setPower(0.051250000000000004);
            leftFrontMotor.setPower(-0.051250000000000004);
            rightFrontMotor.setPower(0.051250000000000004);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.953417434 && elapsedTime < 1.964443321) {
            leftBackMotor.setPower(-0.043750000000000004);
            rightBackMotor.setPower(0.043750000000000004);
            leftFrontMotor.setPower(-0.043750000000000004);
            rightFrontMotor.setPower(0.043750000000000004);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.964443321 && elapsedTime < 1.977451447) {
            leftBackMotor.setPower(-0.04);
            rightBackMotor.setPower(0.04);
            leftFrontMotor.setPower(-0.04);
            rightFrontMotor.setPower(0.04);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.977451447 && elapsedTime < 1.990486604) {
            leftBackMotor.setPower(-0.03375);
            rightBackMotor.setPower(0.03375);
            leftFrontMotor.setPower(-0.03375);
            rightFrontMotor.setPower(0.03375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 1.990486604 && elapsedTime < 2.003703064) {
            leftBackMotor.setPower(-0.02125);
            rightBackMotor.setPower(0.02125);
            leftFrontMotor.setPower(-0.02125);
            rightFrontMotor.setPower(0.02125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.003703064 && elapsedTime < 2.016806972) {
            leftBackMotor.setPower(-0.01625);
            rightBackMotor.setPower(0.01625);
            leftFrontMotor.setPower(-0.01625);
            rightFrontMotor.setPower(0.01625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.016806972 && elapsedTime < 2.030240983) {
            leftBackMotor.setPower(-0.01);
            rightBackMotor.setPower(0.01);
            leftFrontMotor.setPower(-0.01);
            rightFrontMotor.setPower(0.01);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.030240983 && elapsedTime < 2.042709839) {
            leftBackMotor.setPower(0.0025);
            rightBackMotor.setPower(-0.0025);
            leftFrontMotor.setPower(0.0025);
            rightFrontMotor.setPower(-0.0025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.042709839 && elapsedTime < 2.054410465) {
            leftBackMotor.setPower(0.0075);
            rightBackMotor.setPower(-0.0075);
            leftFrontMotor.setPower(0.0075);
            rightFrontMotor.setPower(-0.0075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.054410465 && elapsedTime < 2.06907781) {
            leftBackMotor.setPower(0.01375);
            rightBackMotor.setPower(-0.01375);
            leftFrontMotor.setPower(0.01375);
            rightFrontMotor.setPower(-0.01375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.06907781 && elapsedTime < 2.084235364) {
            leftBackMotor.setPower(0.02);
            rightBackMotor.setPower(-0.02);
            leftFrontMotor.setPower(0.02);
            rightFrontMotor.setPower(-0.02);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.084235364 && elapsedTime < 2.103838907) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.103838907 && elapsedTime < 2.117614169) {
            leftBackMotor.setPower(0.0375);
            rightBackMotor.setPower(-0.0375);
            leftFrontMotor.setPower(0.0375);
            rightFrontMotor.setPower(-0.0375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.117614169 && elapsedTime < 2.175865112) {
            leftBackMotor.setPower(0.0425);
            rightBackMotor.setPower(-0.0425);
            leftFrontMotor.setPower(0.0425);
            rightFrontMotor.setPower(-0.0425);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.175865112 && elapsedTime < 2.189059489) {
            leftBackMotor.setPower(0.058750000000000004);
            rightBackMotor.setPower(-0.058750000000000004);
            leftFrontMotor.setPower(0.058750000000000004);
            rightFrontMotor.setPower(-0.058750000000000004);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 2.189059489 && elapsedTime < 2.212888918) {
            leftBackMotor.setPower(0.06125);
            rightBackMotor.setPower(-0.06125);
            leftFrontMotor.setPower(0.06125);
            rightFrontMotor.setPower(-0.06125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.212888918 && elapsedTime < 2.22567418) {
            leftBackMotor.setPower(0.0625);
            rightBackMotor.setPower(-0.0625);
            leftFrontMotor.setPower(0.0625);
            rightFrontMotor.setPower(-0.0625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.22567418 && elapsedTime < 2.266416371) {
            leftBackMotor.setPower(0.1830609014382064);
            rightBackMotor.setPower(-0.1830609014382064);
            leftFrontMotor.setPower(-0.05806090143820643);
            rightFrontMotor.setPower(0.05806090143820643);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.266416371 && elapsedTime < 2.29301226) {
            leftBackMotor.setPower(0.68375124000248);
            rightBackMotor.setPower(-0.68375124000248);
            leftFrontMotor.setPower(-0.55625124000248);
            rightFrontMotor.setPower(0.55625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.29301226 && elapsedTime < 2.326157054) {
            leftBackMotor.setPower(0.68250124000248);
            rightBackMotor.setPower(-0.68250124000248);
            leftFrontMotor.setPower(-0.55750124000248);
            rightFrontMotor.setPower(0.55750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.326157054 && elapsedTime < 2.352722057) {
            leftBackMotor.setPower(0.68000124000248);
            rightBackMotor.setPower(-0.68000124000248);
            leftFrontMotor.setPower(-0.5600012400024801);
            rightFrontMotor.setPower(0.5600012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.352722057 && elapsedTime < 2.375908466) {
            leftBackMotor.setPower(0.67875124000248);
            rightBackMotor.setPower(-0.67875124000248);
            leftFrontMotor.setPower(-0.5612512400024801);
            rightFrontMotor.setPower(0.5612512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.375908466 && elapsedTime < 2.402412479) {
            leftBackMotor.setPower(0.67250124000248);
            rightBackMotor.setPower(-0.67250124000248);
            leftFrontMotor.setPower(-0.56750124000248);
            rightFrontMotor.setPower(0.56750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.402412479 && elapsedTime < 2.430343419) {
            leftBackMotor.setPower(0.6700012400024801);
            rightBackMotor.setPower(-0.6700012400024801);
            leftFrontMotor.setPower(-0.57000124000248);
            rightFrontMotor.setPower(0.57000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.430343419 && elapsedTime < 2.455035609) {
            leftBackMotor.setPower(0.6650012400024801);
            rightBackMotor.setPower(-0.6650012400024801);
            leftFrontMotor.setPower(-0.57500124000248);
            rightFrontMotor.setPower(0.57500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.455035609 && elapsedTime < 2.482132487) {
            leftBackMotor.setPower(0.65750124000248);
            rightBackMotor.setPower(-0.65750124000248);
            leftFrontMotor.setPower(-0.58250124000248);
            rightFrontMotor.setPower(0.58250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.482132487 && elapsedTime < 2.50920676) {
            leftBackMotor.setPower(0.6550012400024801);
            rightBackMotor.setPower(-0.6550012400024801);
            leftFrontMotor.setPower(-0.58500124000248);
            rightFrontMotor.setPower(0.58500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.50920676 && elapsedTime < 2.536093534) {
            leftBackMotor.setPower(0.64625124000248);
            rightBackMotor.setPower(-0.64625124000248);
            leftFrontMotor.setPower(-0.59375124000248);
            rightFrontMotor.setPower(0.59375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.536093534 && elapsedTime < 2.565001141) {
            leftBackMotor.setPower(0.64000124000248);
            rightBackMotor.setPower(-0.64000124000248);
            leftFrontMotor.setPower(-0.60000124000248);
            rightFrontMotor.setPower(0.60000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.565001141 && elapsedTime < 2.594198696) {
            leftBackMotor.setPower(0.62750124000248);
            rightBackMotor.setPower(-0.62750124000248);
            leftFrontMotor.setPower(-0.6125012400024801);
            rightFrontMotor.setPower(0.6125012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.594198696 && elapsedTime < 2.629144012) {
            leftBackMotor.setPower(0.61125124000248);
            rightBackMotor.setPower(-0.61125124000248);
            leftFrontMotor.setPower(-0.6287512400024801);
            rightFrontMotor.setPower(0.6287512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.629144012 && elapsedTime < 2.698352769) {
            leftBackMotor.setPower(0.60500124000248);
            rightBackMotor.setPower(-0.60500124000248);
            leftFrontMotor.setPower(-0.63500124000248);
            rightFrontMotor.setPower(0.63500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.698352769 && elapsedTime < 2.724129386) {
            leftBackMotor.setPower(0.57750124000248);
            rightBackMotor.setPower(-0.57750124000248);
            leftFrontMotor.setPower(-0.66250124000248);
            rightFrontMotor.setPower(0.66250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.724129386 && elapsedTime < 2.751115066) {
            leftBackMotor.setPower(0.56250124000248);
            rightBackMotor.setPower(-0.56250124000248);
            leftFrontMotor.setPower(-0.67750124000248);
            rightFrontMotor.setPower(0.67750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.751115066 && elapsedTime < 2.792468351) {
            leftBackMotor.setPower(0.55500124000248);
            rightBackMotor.setPower(-0.55500124000248);
            leftFrontMotor.setPower(-0.6850012400024801);
            rightFrontMotor.setPower(0.6850012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.792468351 && elapsedTime < 2.820691635) {
            leftBackMotor.setPower(0.3880104913681688);
            rightBackMotor.setPower(-0.3880104913681688);
            leftFrontMotor.setPower(-0.5430104913681688);
            rightFrontMotor.setPower(0.5430104913681688);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.820691635 && elapsedTime < 2.852013253) {
            leftBackMotor.setPower(-0.08);
            rightBackMotor.setPower(0.08);
            leftFrontMotor.setPower(-0.08);
            rightFrontMotor.setPower(0.08);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.852013253 && elapsedTime < 2.87324513) {
            leftBackMotor.setPower(-0.09);
            rightBackMotor.setPower(0.09);
            leftFrontMotor.setPower(-0.09);
            rightFrontMotor.setPower(0.09);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.87324513 && elapsedTime < 2.897863726) {
            leftBackMotor.setPower(-0.09125);
            rightBackMotor.setPower(0.09125);
            leftFrontMotor.setPower(-0.09125);
            rightFrontMotor.setPower(0.09125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.897863726 && elapsedTime < 2.910398571) {
            leftBackMotor.setPower(-0.09);
            rightBackMotor.setPower(0.09);
            leftFrontMotor.setPower(-0.09);
            rightFrontMotor.setPower(0.09);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.910398571 && elapsedTime < 2.922258103) {
            leftBackMotor.setPower(-0.085);
            rightBackMotor.setPower(0.085);
            leftFrontMotor.setPower(-0.085);
            rightFrontMotor.setPower(0.085);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.922258103 && elapsedTime < 2.933100917) {
            leftBackMotor.setPower(-0.08625000000000001);
            rightBackMotor.setPower(0.08625000000000001);
            leftFrontMotor.setPower(-0.08625000000000001);
            rightFrontMotor.setPower(0.08625000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.933100917 && elapsedTime < 2.948926752) {
            leftBackMotor.setPower(-0.0775);
            rightBackMotor.setPower(0.0775);
            leftFrontMotor.setPower(-0.0775);
            rightFrontMotor.setPower(0.0775);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.948926752 && elapsedTime < 2.961204722) {
            leftBackMotor.setPower(-0.07125000000000001);
            rightBackMotor.setPower(0.07125000000000001);
            leftFrontMotor.setPower(-0.07125000000000001);
            rightFrontMotor.setPower(0.07125000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.961204722 && elapsedTime < 2.973332171) {
            leftBackMotor.setPower(-0.06375);
            rightBackMotor.setPower(0.06375);
            leftFrontMotor.setPower(-0.06375);
            rightFrontMotor.setPower(0.06375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.973332171 && elapsedTime < 2.984201912) {
            leftBackMotor.setPower(-0.055);
            rightBackMotor.setPower(0.055);
            leftFrontMotor.setPower(-0.055);
            rightFrontMotor.setPower(0.055);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.984201912 && elapsedTime < 2.997208944) {
            leftBackMotor.setPower(-0.05);
            rightBackMotor.setPower(0.05);
            leftFrontMotor.setPower(-0.05);
            rightFrontMotor.setPower(0.05);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 2.997208944 && elapsedTime < 3.010107123) {
            leftBackMotor.setPower(-0.0475);
            rightBackMotor.setPower(0.0475);
            leftFrontMotor.setPower(-0.0475);
            rightFrontMotor.setPower(0.0475);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.010107123 && elapsedTime < 3.023534364) {
            leftBackMotor.setPower(-0.045);
            rightBackMotor.setPower(0.045);
            leftFrontMotor.setPower(-0.045);
            rightFrontMotor.setPower(0.045);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.023534364 && elapsedTime < 3.036024209) {
            leftBackMotor.setPower(-0.03625);
            rightBackMotor.setPower(0.03625);
            leftFrontMotor.setPower(-0.03625);
            rightFrontMotor.setPower(0.03625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 3.036024209 && elapsedTime < 3.062338065) {
            leftBackMotor.setPower(-0.035);
            rightBackMotor.setPower(0.035);
            leftFrontMotor.setPower(-0.035);
            rightFrontMotor.setPower(0.035);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.062338065 && elapsedTime < 3.074873848) {
            leftBackMotor.setPower(-0.03125);
            rightBackMotor.setPower(0.03125);
            leftFrontMotor.setPower(-0.03125);
            rightFrontMotor.setPower(0.03125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.074873848 && elapsedTime < 3.089160776) {
            leftBackMotor.setPower(-0.03375);
            rightBackMotor.setPower(0.03375);
            leftFrontMotor.setPower(-0.03375);
            rightFrontMotor.setPower(0.03375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.089160776 && elapsedTime < 3.10502484) {
            leftBackMotor.setPower(-0.035);
            rightBackMotor.setPower(0.035);
            leftFrontMotor.setPower(-0.035);
            rightFrontMotor.setPower(0.035);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.10502484 && elapsedTime < 3.122450728) {
            leftBackMotor.setPower(-0.0325);
            rightBackMotor.setPower(0.0325);
            leftFrontMotor.setPower(-0.0325);
            rightFrontMotor.setPower(0.0325);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.122450728 && elapsedTime < 3.137467552) {
            leftBackMotor.setPower(-0.03375);
            rightBackMotor.setPower(0.03375);
            leftFrontMotor.setPower(-0.03375);
            rightFrontMotor.setPower(0.03375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 3.137467552 && elapsedTime < 3.226344123) {
            leftBackMotor.setPower(-0.0325);
            rightBackMotor.setPower(0.0325);
            leftFrontMotor.setPower(-0.0325);
            rightFrontMotor.setPower(0.0325);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.226344123 && elapsedTime < 3.252988188) {
            leftBackMotor.setPower(-0.65125124000248);
            rightBackMotor.setPower(0.65125124000248);
            leftFrontMotor.setPower(0.58875124000248);
            rightFrontMotor.setPower(-0.58875124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 3.252988188 && elapsedTime < 3.29283262) {
            leftBackMotor.setPower(-0.65000124000248);
            rightBackMotor.setPower(0.65000124000248);
            leftFrontMotor.setPower(0.59000124000248);
            rightFrontMotor.setPower(-0.59000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.29283262 && elapsedTime < 3.343449031) {
            leftBackMotor.setPower(-0.6487512400024801);
            rightBackMotor.setPower(0.6487512400024801);
            leftFrontMotor.setPower(0.59125124000248);
            rightFrontMotor.setPower(-0.59125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.343449031 && elapsedTime < 3.370691481) {
            leftBackMotor.setPower(-0.63750124000248);
            rightBackMotor.setPower(0.63750124000248);
            leftFrontMotor.setPower(0.6025012400024801);
            rightFrontMotor.setPower(-0.6025012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.370691481 && elapsedTime < 3.396489609) {
            leftBackMotor.setPower(-0.6337512400024801);
            rightBackMotor.setPower(0.6337512400024801);
            leftFrontMotor.setPower(0.60625124000248);
            rightFrontMotor.setPower(-0.60625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.396489609 && elapsedTime < 3.420540757) {
            leftBackMotor.setPower(-0.62125124000248);
            rightBackMotor.setPower(0.62125124000248);
            leftFrontMotor.setPower(0.61875124000248);
            rightFrontMotor.setPower(-0.61875124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.420540757 && elapsedTime < 3.448835448) {
            leftBackMotor.setPower(-0.6125012400024801);
            rightBackMotor.setPower(0.6125012400024801);
            leftFrontMotor.setPower(0.62750124000248);
            rightFrontMotor.setPower(-0.62750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.448835448 && elapsedTime < 3.477256128) {
            leftBackMotor.setPower(-0.59500124000248);
            rightBackMotor.setPower(0.59500124000248);
            leftFrontMotor.setPower(0.64500124000248);
            rightFrontMotor.setPower(-0.64500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.477256128 && elapsedTime < 3.50019764) {
            leftBackMotor.setPower(-0.57500124000248);
            rightBackMotor.setPower(0.57500124000248);
            leftFrontMotor.setPower(0.6650012400024801);
            rightFrontMotor.setPower(-0.6650012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.50019764 && elapsedTime < 3.528176758) {
            leftBackMotor.setPower(-0.56500124000248);
            rightBackMotor.setPower(0.56500124000248);
            leftFrontMotor.setPower(0.6750012400024801);
            rightFrontMotor.setPower(-0.6750012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.528176758 && elapsedTime < 3.55243202) {
            leftBackMotor.setPower(-0.5400012400024801);
            rightBackMotor.setPower(0.5400012400024801);
            leftFrontMotor.setPower(0.70000124000248);
            rightFrontMotor.setPower(-0.70000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.55243202 && elapsedTime < 3.574397179) {
            leftBackMotor.setPower(-0.52750124000248);
            rightBackMotor.setPower(0.52750124000248);
            leftFrontMotor.setPower(0.71250124000248);
            rightFrontMotor.setPower(-0.71250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.574397179 && elapsedTime < 3.60274015) {
            leftBackMotor.setPower(-0.5137512400024801);
            rightBackMotor.setPower(0.5137512400024801);
            leftFrontMotor.setPower(0.72625124000248);
            rightFrontMotor.setPower(-0.72625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.60274015 && elapsedTime < 3.628166143) {
            leftBackMotor.setPower(-0.49750124000248);
            rightBackMotor.setPower(0.49750124000248);
            leftFrontMotor.setPower(0.74250124000248);
            rightFrontMotor.setPower(-0.74250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.628166143 && elapsedTime < 3.661525885) {
            leftBackMotor.setPower(-0.48750124000248);
            rightBackMotor.setPower(0.48750124000248);
            leftFrontMotor.setPower(0.75250124000248);
            rightFrontMotor.setPower(-0.75250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.661525885 && elapsedTime < 3.738834956) {
            leftBackMotor.setPower(-0.47625124000248);
            rightBackMotor.setPower(0.47625124000248);
            leftFrontMotor.setPower(0.7637512400024801);
            rightFrontMotor.setPower(-0.7637512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.738834956 && elapsedTime < 3.765402146) {
            leftBackMotor.setPower(-0.46375124000248);
            rightBackMotor.setPower(0.46375124000248);
            leftFrontMotor.setPower(0.77625124000248);
            rightFrontMotor.setPower(-0.77625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.765402146 && elapsedTime < 3.791945846) {
            leftBackMotor.setPower(-0.46500124000248);
            rightBackMotor.setPower(0.46500124000248);
            leftFrontMotor.setPower(0.77500124000248);
            rightFrontMotor.setPower(-0.77500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.791945846 && elapsedTime < 3.818159911) {
            leftBackMotor.setPower(-0.46375124000248);
            rightBackMotor.setPower(0.46375124000248);
            leftFrontMotor.setPower(0.77625124000248);
            rightFrontMotor.setPower(-0.77625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.818159911 && elapsedTime < 3.862474395) {
            leftBackMotor.setPower(-0.47000124000248);
            rightBackMotor.setPower(0.47000124000248);
            leftFrontMotor.setPower(0.77000124000248);
            rightFrontMotor.setPower(-0.77000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.862474395 && elapsedTime < 3.888831481) {
            leftBackMotor.setPower(-0.48375124000248004);
            rightBackMotor.setPower(0.48375124000248004);
            leftFrontMotor.setPower(0.75625124000248);
            rightFrontMotor.setPower(-0.75625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.888831481 && elapsedTime < 3.913926692) {
            leftBackMotor.setPower(-0.49750124000248);
            rightBackMotor.setPower(0.49750124000248);
            leftFrontMotor.setPower(0.74250124000248);
            rightFrontMotor.setPower(-0.74250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.913926692 && elapsedTime < 3.940989663) {
            leftBackMotor.setPower(-0.004791672340984279);
            rightBackMotor.setPower(0.004791672340984279);
            leftFrontMotor.setPower(0.2272916723409843);
            rightFrontMotor.setPower(-0.2272916723409843);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.940989663 && elapsedTime < 3.968162374) {
            leftBackMotor.setPower(0.0975);
            rightBackMotor.setPower(-0.0975);
            leftFrontMotor.setPower(0.0975);
            rightFrontMotor.setPower(-0.0975);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.968162374 && elapsedTime < 3.980875553) {
            leftBackMotor.setPower(0.0825);
            rightBackMotor.setPower(-0.0825);
            leftFrontMotor.setPower(0.0825);
            rightFrontMotor.setPower(-0.0825);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.980875553 && elapsedTime < 3.994568731) {
            leftBackMotor.setPower(0.07375);
            rightBackMotor.setPower(-0.07375);
            leftFrontMotor.setPower(0.07375);
            rightFrontMotor.setPower(-0.07375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 3.994568731 && elapsedTime < 4.006295972) {
            leftBackMotor.setPower(0.0675);
            rightBackMotor.setPower(-0.0675);
            leftFrontMotor.setPower(0.0675);
            rightFrontMotor.setPower(-0.0675);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.006295972 && elapsedTime < 4.019089671) {
            leftBackMotor.setPower(0.05625);
            rightBackMotor.setPower(-0.05625);
            leftFrontMotor.setPower(0.05625);
            rightFrontMotor.setPower(-0.05625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.019089671 && elapsedTime < 4.0339516) {
            leftBackMotor.setPower(0.05);
            rightBackMotor.setPower(-0.05);
            leftFrontMotor.setPower(0.05);
            rightFrontMotor.setPower(-0.05);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.0339516 && elapsedTime < 4.048171757) {
            leftBackMotor.setPower(0.0425);
            rightBackMotor.setPower(-0.0425);
            leftFrontMotor.setPower(0.0425);
            rightFrontMotor.setPower(-0.0425);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.048171757 && elapsedTime < 4.060979884) {
            leftBackMotor.setPower(0.03875);
            rightBackMotor.setPower(-0.03875);
            leftFrontMotor.setPower(0.03875);
            rightFrontMotor.setPower(-0.03875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.060979884 && elapsedTime < 4.074126239) {
            leftBackMotor.setPower(0.035);
            rightBackMotor.setPower(-0.035);
            leftFrontMotor.setPower(0.035);
            rightFrontMotor.setPower(-0.035);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 4.074126239 && elapsedTime < 4.110168482) {
            leftBackMotor.setPower(0.03);
            rightBackMotor.setPower(-0.03);
            leftFrontMotor.setPower(0.03);
            rightFrontMotor.setPower(-0.03);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.110168482 && elapsedTime < 4.125053432) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 4.125053432 && elapsedTime < 4.2478311) {
            leftBackMotor.setPower(0.025);
            rightBackMotor.setPower(-0.025);
            leftFrontMotor.setPower(0.025);
            rightFrontMotor.setPower(-0.025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 4.2478311 && elapsedTime < 4.383371739) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 4.383371739 && elapsedTime < 4.423084607) {
            leftBackMotor.setPower(-0.24757672719749152);
            rightBackMotor.setPower(0.24757672719749152);
            leftFrontMotor.setPower(-0.24757672719749152);
            rightFrontMotor.setPower(0.24757672719749152);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 4.423084607 && elapsedTime < 4.755388859) {
            leftBackMotor.setPower(-0.7502015004030008);
            rightBackMotor.setPower(0.7502015004030008);
            leftFrontMotor.setPower(-0.7502015004030008);
            rightFrontMotor.setPower(0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.755388859 && elapsedTime < 4.782306101) {
            leftBackMotor.setPower(0.28375);
            rightBackMotor.setPower(-0.28375);
            leftFrontMotor.setPower(0.28375);
            rightFrontMotor.setPower(-0.28375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.782306101 && elapsedTime < 4.795667405) {
            leftBackMotor.setPower(0.3725);
            rightBackMotor.setPower(-0.3725);
            leftFrontMotor.setPower(0.3725);
            rightFrontMotor.setPower(-0.3725);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.795667405 && elapsedTime < 4.809095635) {
            leftBackMotor.setPower(0.45875);
            rightBackMotor.setPower(-0.45875);
            leftFrontMotor.setPower(0.45875);
            rightFrontMotor.setPower(-0.45875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.809095635 && elapsedTime < 4.821331782) {
            leftBackMotor.setPower(0.50125);
            rightBackMotor.setPower(-0.50125);
            leftFrontMotor.setPower(0.50125);
            rightFrontMotor.setPower(-0.50125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.821331782 && elapsedTime < 4.835203242) {
            leftBackMotor.setPower(0.54125);
            rightBackMotor.setPower(-0.54125);
            leftFrontMotor.setPower(0.54125);
            rightFrontMotor.setPower(-0.54125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.835203242 && elapsedTime < 4.847799285) {
            leftBackMotor.setPower(0.57875);
            rightBackMotor.setPower(-0.57875);
            leftFrontMotor.setPower(0.57875);
            rightFrontMotor.setPower(-0.57875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.847799285 && elapsedTime < 4.860526474) {
            leftBackMotor.setPower(0.6425);
            rightBackMotor.setPower(-0.6425);
            leftFrontMotor.setPower(0.6425);
            rightFrontMotor.setPower(-0.6425);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.860526474 && elapsedTime < 4.875623506) {
            leftBackMotor.setPower(0.67);
            rightBackMotor.setPower(-0.67);
            leftFrontMotor.setPower(0.67);
            rightFrontMotor.setPower(-0.67);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.875623506 && elapsedTime < 4.891370696) {
            leftBackMotor.setPower(0.7175);
            rightBackMotor.setPower(-0.7175);
            leftFrontMotor.setPower(0.7175);
            rightFrontMotor.setPower(-0.7175);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.891370696 && elapsedTime < 4.905710957) {
            leftBackMotor.setPower(0.73625);
            rightBackMotor.setPower(-0.73625);
            leftFrontMotor.setPower(0.73625);
            rightFrontMotor.setPower(-0.73625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.905710957 && elapsedTime < 4.939659815) {
            leftBackMotor.setPower(0.75125);
            rightBackMotor.setPower(-0.75125);
            leftFrontMotor.setPower(0.75125);
            rightFrontMotor.setPower(-0.75125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.939659815 && elapsedTime < 4.966138515) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 4.966138515 && elapsedTime < 4.993179976) {
            leftBackMotor.setPower(0.3004782393411393);
            rightBackMotor.setPower(-0.3004782393411393);
            leftFrontMotor.setPower(0.3004782393411393);
            rightFrontMotor.setPower(-0.3004782393411393);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 4.993179976 && elapsedTime < 5.300596413) {
            leftBackMotor.setPower(0.7502015004030008);
            rightBackMotor.setPower(-0.7502015004030008);
            leftFrontMotor.setPower(0.7502015004030008);
            rightFrontMotor.setPower(-0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 5.300596413 && elapsedTime < 5.342022199) {
            leftBackMotor.setPower(0.6074942507991073);
            rightBackMotor.setPower(-0.6074942507991073);
            leftFrontMotor.setPower(0.6074942507991073);
            rightFrontMotor.setPower(-0.6074942507991073);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 5.342022199 && elapsedTime < 5.389089391) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.389089391 && elapsedTime < 5.402017778) {
            leftBackMotor.setPower(-0.08375);
            rightBackMotor.setPower(0.08375);
            leftFrontMotor.setPower(-0.08375);
            rightFrontMotor.setPower(0.08375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.402017778 && elapsedTime < 5.418850748) {
            leftBackMotor.setPower(-0.125);
            rightBackMotor.setPower(0.125);
            leftFrontMotor.setPower(-0.125);
            rightFrontMotor.setPower(0.125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.418850748 && elapsedTime < 5.428390905) {
            leftBackMotor.setPower(-0.1975);
            rightBackMotor.setPower(0.1975);
            leftFrontMotor.setPower(-0.1975);
            rightFrontMotor.setPower(0.1975);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.428390905 && elapsedTime < 5.438848458) {
            leftBackMotor.setPower(-0.22875);
            rightBackMotor.setPower(0.22875);
            leftFrontMotor.setPower(-0.22875);
            rightFrontMotor.setPower(0.22875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 5.438848458 && elapsedTime < 5.457871169) {
            leftBackMotor.setPower(-0.25875);
            rightBackMotor.setPower(0.25875);
            leftFrontMotor.setPower(-0.25875);
            rightFrontMotor.setPower(0.25875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.457871169 && elapsedTime < 5.468039607) {
            leftBackMotor.setPower(-0.28625);
            rightBackMotor.setPower(0.28625);
            leftFrontMotor.setPower(-0.28625);
            rightFrontMotor.setPower(0.28625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.468039607 && elapsedTime < 5.478682421) {
            leftBackMotor.setPower(-0.32875);
            rightBackMotor.setPower(0.32875);
            leftFrontMotor.setPower(-0.32875);
            rightFrontMotor.setPower(0.32875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.478682421 && elapsedTime < 5.489324297) {
            leftBackMotor.setPower(-0.34625);
            rightBackMotor.setPower(0.34625);
            leftFrontMotor.setPower(-0.34625);
            rightFrontMotor.setPower(0.34625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.489324297 && elapsedTime < 5.500322735) {
            leftBackMotor.setPower(-0.36125);
            rightBackMotor.setPower(0.36125);
            leftFrontMotor.setPower(-0.36125);
            rightFrontMotor.setPower(0.36125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.500322735 && elapsedTime < 5.511474143) {
            leftBackMotor.setPower(-0.3725);
            rightBackMotor.setPower(0.3725);
            leftFrontMotor.setPower(-0.3725);
            rightFrontMotor.setPower(0.3725);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.511474143 && elapsedTime < 5.522782477) {
            leftBackMotor.setPower(-0.38125000000000003);
            rightBackMotor.setPower(0.38125000000000003);
            leftFrontMotor.setPower(-0.38125000000000003);
            rightFrontMotor.setPower(0.38125000000000003);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.522782477 && elapsedTime < 5.534248416) {
            leftBackMotor.setPower(-0.38625);
            rightBackMotor.setPower(0.38625);
            leftFrontMotor.setPower(-0.38625);
            rightFrontMotor.setPower(0.38625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.534248416 && elapsedTime < 5.548314667) {
            leftBackMotor.setPower(-0.38875);
            rightBackMotor.setPower(0.38875);
            leftFrontMotor.setPower(-0.38875);
            rightFrontMotor.setPower(0.38875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 5.548314667 && elapsedTime < 5.561231648) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 5.561231648 && elapsedTime < 5.589831546) {
            leftBackMotor.setPower(-0.39);
            rightBackMotor.setPower(0.39);
            leftFrontMotor.setPower(-0.39);
            rightFrontMotor.setPower(0.39);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 5.589831546 && elapsedTime < 5.619191497) {
            leftBackMotor.setPower(-0.38875);
            rightBackMotor.setPower(0.38875);
            leftFrontMotor.setPower(-0.38875);
            rightFrontMotor.setPower(0.38875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 5.619191497 && elapsedTime < 6.799711979) {
            leftBackMotor.setPower(-0.39);
            rightBackMotor.setPower(0.39);
            leftFrontMotor.setPower(-0.39);
            rightFrontMotor.setPower(0.39);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 6.799711979 && elapsedTime < 6.947711317) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 6.947711317 && elapsedTime < 6.985039706) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.507718026638031);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 6.985039706 && elapsedTime < 7.025698252) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.025698252 && elapsedTime < 7.061209662) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.8664997816085815);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.061209662 && elapsedTime < 7.103343208) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.19899874925613403);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 7.103343208 && elapsedTime < 7.123123366) {
            leftBackMotor.setPower(-0.39);
            rightBackMotor.setPower(0.39);
            leftFrontMotor.setPower(-0.39);
            rightFrontMotor.setPower(0.39);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.123123366 && elapsedTime < 7.148692014) {
            leftBackMotor.setPower(-0.38875);
            rightBackMotor.setPower(0.38875);
            leftFrontMotor.setPower(-0.38875);
            rightFrontMotor.setPower(0.38875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.148692014 && elapsedTime < 7.172668371) {
            leftBackMotor.setPower(-0.39);
            rightBackMotor.setPower(0.39);
            leftFrontMotor.setPower(-0.39);
            rightFrontMotor.setPower(0.39);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.172668371 && elapsedTime < 7.222714626) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.222714626 && elapsedTime < 7.339415783) {
            leftBackMotor.setPower(-0.39);
            rightBackMotor.setPower(0.39);
            leftFrontMotor.setPower(-0.39);
            rightFrontMotor.setPower(0.39);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 7.339415783 && elapsedTime < 7.362352452) {
            leftBackMotor.setPower(-0.39);
            rightBackMotor.setPower(0.39);
            leftFrontMotor.setPower(-0.39);
            rightFrontMotor.setPower(0.39);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.4910304844379425);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 7.362352452 && elapsedTime < 7.381023235) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.4910304844379425);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 7.381023235 && elapsedTime < 7.429044854) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.9415936470031738);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.429044854 && elapsedTime < 7.487308923) {
            leftBackMotor.setPower(-0.39);
            rightBackMotor.setPower(0.39);
            leftFrontMotor.setPower(-0.39);
            rightFrontMotor.setPower(0.39);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.487308923 && elapsedTime < 7.541969345) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.541969345 && elapsedTime < 7.5741944) {
            leftBackMotor.setPower(-0.39);
            rightBackMotor.setPower(0.39);
            leftFrontMotor.setPower(-0.39);
            rightFrontMotor.setPower(0.39);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.5741944 && elapsedTime < 7.929523186) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 7.929523186 && elapsedTime < 7.956476105) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 7.956476105 && elapsedTime < 7.976594805) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 7.976594805 && elapsedTime < 8.043890853) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 8.043890853 && elapsedTime < 8.06070148) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.06070148 && elapsedTime < 8.347763227) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.347763227 && elapsedTime < 8.389904638) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.8915311098098755);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.389904638 && elapsedTime < 8.497652305) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.497652305 && elapsedTime < 8.523964912) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.523964912 && elapsedTime < 8.574903406) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.574903406 && elapsedTime < 8.618043983) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.618043983 && elapsedTime < 8.668713207) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(-0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.668713207 && elapsedTime < 8.693420866) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(-0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.693420866 && elapsedTime < 8.852634736) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(-0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 8.852634736 && elapsedTime < 8.869457342) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(-0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 8.869457342 && elapsedTime < 9.024928868) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(-0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.024928868 && elapsedTime < 9.095949291) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.095949291 && elapsedTime < 9.126396951) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.126396951 && elapsedTime < 9.154179297) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.154179297 && elapsedTime < 9.464376411) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.464376411 && elapsedTime < 9.585589444) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.585589444 && elapsedTime < 9.626905386) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.19065499305725098);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.626905386 && elapsedTime < 9.664538359) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.4075928330421448);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.664538359 && elapsedTime < 9.704389977) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.07384230941534042);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.704389977 && elapsedTime < 9.745663627) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.6662495136260986);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.745663627 && elapsedTime < 9.793516288) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.793516288 && elapsedTime < 9.82817426) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 9.82817426 && elapsedTime < 9.879707599) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 9.879707599 && elapsedTime < 9.902589372) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.902589372 && elapsedTime < 9.962710576) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 9.962710576 && elapsedTime < 9.984503182) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 9.984503182 && elapsedTime < 10.388963222) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 10.388963222 && elapsedTime < 10.899439054) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 10.899439054 && elapsedTime < 10.923263692) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 10.923263692 && elapsedTime < 10.938098173) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 10.938098173 && elapsedTime < 10.98813198) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 10.98813198 && elapsedTime < 11.021374692) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 11.021374692 && elapsedTime < 11.03608261) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 11.03608261 && elapsedTime < 11.4022865) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(0.39125);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(0.39125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 11.4022865 && elapsedTime < 11.669001944) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 11.669001944 && elapsedTime < 12.501583485) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.98);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 12.501583485 && elapsedTime < 12.870901282) {
            leftBackMotor.setPower(-0.3925);
            rightBackMotor.setPower(0.3925);
            leftFrontMotor.setPower(-0.3925);
            rightFrontMotor.setPower(0.3925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 12.870901282 && elapsedTime < 13.027276298) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 13.027276298 && elapsedTime < 13.21069017) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 13.21069017 && elapsedTime < 13.857054766) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 13.857054766 && elapsedTime < 14.467181545) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 14.467181545 && elapsedTime < 15.290140013) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 15.290140013 && elapsedTime < 15.34215882) {
            leftBackMotor.setPower(-0.395);
            rightBackMotor.setPower(0.395);
            leftFrontMotor.setPower(-0.395);
            rightFrontMotor.setPower(0.395);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 15.34215882 && elapsedTime < 15.817724284) {
            leftBackMotor.setPower(-0.39375);
            rightBackMotor.setPower(0.39375);
            leftFrontMotor.setPower(-0.39375);
            rightFrontMotor.setPower(0.39375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 15.817724284 && elapsedTime < 15.845557776) {
            leftBackMotor.setPower(-0.7437499999999999);
            rightBackMotor.setPower(0.7437499999999999);
            leftFrontMotor.setPower(-0.7437499999999999);
            rightFrontMotor.setPower(0.7437499999999999);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 15.845557776 && elapsedTime < 16.018625085) {
            leftBackMotor.setPower(-0.35);
            rightBackMotor.setPower(0.35);
            leftFrontMotor.setPower(-0.35);
            rightFrontMotor.setPower(0.35);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.018625085 && elapsedTime < 16.043303004) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.043303004 && elapsedTime < 16.055732901) {
            leftBackMotor.setPower(0.06125);
            rightBackMotor.setPower(-0.06125);
            leftFrontMotor.setPower(0.06125);
            rightFrontMotor.setPower(-0.06125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.055732901 && elapsedTime < 16.068590611) {
            leftBackMotor.setPower(0.0825);
            rightBackMotor.setPower(-0.0825);
            leftFrontMotor.setPower(0.0825);
            rightFrontMotor.setPower(-0.0825);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.068590611 && elapsedTime < 16.082319831) {
            leftBackMotor.setPower(0.10250000000000001);
            rightBackMotor.setPower(-0.10250000000000001);
            leftFrontMotor.setPower(0.10250000000000001);
            rightFrontMotor.setPower(-0.10250000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.082319831 && elapsedTime < 16.094226082) {
            leftBackMotor.setPower(0.14375000000000002);
            rightBackMotor.setPower(-0.14375000000000002);
            leftFrontMotor.setPower(0.14375000000000002);
            rightFrontMotor.setPower(-0.14375000000000002);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.094226082 && elapsedTime < 16.106600719) {
            leftBackMotor.setPower(0.15625);
            rightBackMotor.setPower(-0.15625);
            leftFrontMotor.setPower(0.15625);
            rightFrontMotor.setPower(-0.15625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.106600719 && elapsedTime < 16.119682647) {
            leftBackMotor.setPower(0.16625);
            rightBackMotor.setPower(-0.16625);
            leftFrontMotor.setPower(0.16625);
            rightFrontMotor.setPower(-0.16625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.119682647 && elapsedTime < 16.132859263) {
            leftBackMotor.setPower(0.1775);
            rightBackMotor.setPower(-0.1775);
            leftFrontMotor.setPower(0.1775);
            rightFrontMotor.setPower(-0.1775);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.132859263 && elapsedTime < 16.147009056) {
            leftBackMotor.setPower(0.19125);
            rightBackMotor.setPower(-0.19125);
            leftFrontMotor.setPower(0.19125);
            rightFrontMotor.setPower(-0.19125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 16.147009056 && elapsedTime < 16.186251144) {
            leftBackMotor.setPower(0.19375);
            rightBackMotor.setPower(-0.19375);
            leftFrontMotor.setPower(0.19375);
            rightFrontMotor.setPower(-0.19375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.186251144 && elapsedTime < 16.233653752) {
            leftBackMotor.setPower(0.1925);
            rightBackMotor.setPower(-0.1925);
            leftFrontMotor.setPower(0.1925);
            rightFrontMotor.setPower(-0.1925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 16.233653752 && elapsedTime < 16.301341624) {
            leftBackMotor.setPower(0.19125);
            rightBackMotor.setPower(-0.19125);
            leftFrontMotor.setPower(0.19125);
            rightFrontMotor.setPower(-0.19125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.301341624 && elapsedTime < 16.339857617) {
            leftBackMotor.setPower(0.54125);
            rightBackMotor.setPower(-0.54125);
            leftFrontMotor.setPower(0.54125);
            rightFrontMotor.setPower(-0.54125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 16.339857617 && elapsedTime < 16.508203832) {
            leftBackMotor.setPower(0.35);
            rightBackMotor.setPower(-0.35);
            leftFrontMotor.setPower(0.35);
            rightFrontMotor.setPower(-0.35);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.508203832 && elapsedTime < 16.532703001) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.532703001 && elapsedTime < 16.545656909) {
            leftBackMotor.setPower(-0.058750000000000004);
            rightBackMotor.setPower(0.058750000000000004);
            leftFrontMotor.setPower(-0.058750000000000004);
            rightFrontMotor.setPower(0.058750000000000004);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.545656909 && elapsedTime < 16.558590712) {
            leftBackMotor.setPower(-0.08);
            rightBackMotor.setPower(0.08);
            leftFrontMotor.setPower(-0.08);
            rightFrontMotor.setPower(0.08);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.558590712 && elapsedTime < 16.569610609) {
            leftBackMotor.setPower(-0.10250000000000001);
            rightBackMotor.setPower(0.10250000000000001);
            leftFrontMotor.setPower(-0.10250000000000001);
            rightFrontMotor.setPower(0.10250000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.569610609 && elapsedTime < 16.582166912) {
            leftBackMotor.setPower(-0.12);
            rightBackMotor.setPower(0.12);
            leftFrontMotor.setPower(-0.12);
            rightFrontMotor.setPower(0.12);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.582166912 && elapsedTime < 16.596380039) {
            leftBackMotor.setPower(-0.135);
            rightBackMotor.setPower(0.135);
            leftFrontMotor.setPower(-0.135);
            rightFrontMotor.setPower(0.135);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.596380039 && elapsedTime < 16.608749779) {
            leftBackMotor.setPower(-0.1625);
            rightBackMotor.setPower(0.1625);
            leftFrontMotor.setPower(-0.1625);
            rightFrontMotor.setPower(0.1625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.608749779 && elapsedTime < 16.620678791) {
            leftBackMotor.setPower(-0.17125);
            rightBackMotor.setPower(0.17125);
            leftFrontMotor.setPower(-0.17125);
            rightFrontMotor.setPower(0.17125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.620678791 && elapsedTime < 16.632027334) {
            leftBackMotor.setPower(-0.17625);
            rightBackMotor.setPower(0.17625);
            leftFrontMotor.setPower(-0.17625);
            rightFrontMotor.setPower(0.17625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.632027334 && elapsedTime < 16.645397752) {
            leftBackMotor.setPower(-0.18);
            rightBackMotor.setPower(0.18);
            leftFrontMotor.setPower(-0.18);
            rightFrontMotor.setPower(0.18);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.645397752 && elapsedTime < 16.657028222) {
            leftBackMotor.setPower(-0.18125);
            rightBackMotor.setPower(0.18125);
            leftFrontMotor.setPower(-0.18125);
            rightFrontMotor.setPower(0.18125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if (elapsedTime > 16.657028222 && elapsedTime < 16.67685713) {
            leftBackMotor.setPower(-0.18);
            rightBackMotor.setPower(0.18);
            leftFrontMotor.setPower(-0.18);
            rightFrontMotor.setPower(0.18);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 16.67685713 && elapsedTime < 17.287507503) {
            leftBackMotor.setPower(-0.1775);
            rightBackMotor.setPower(0.1775);
            leftFrontMotor.setPower(-0.1775);
            rightFrontMotor.setPower(0.1775);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 16.67685713 && elapsedTime < 17.287507503) {
            leftBackMotor.setPower(-0.03375);
            rightBackMotor.setPower(0.03375);
            leftFrontMotor.setPower(-0.03375);
            rightFrontMotor.setPower(0.03375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            leftSkystoneServo.setPosition(0.52);
            rightSkystoneServo.setPosition(0.98);
        }
        endTime = 17.287507503;
    }
}
