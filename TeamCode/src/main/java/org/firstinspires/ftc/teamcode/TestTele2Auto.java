package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private ColorSensor colorSensor;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.2, correction;
    double elapsedTime;
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
        colorSensor             = hardwareMap.get(ColorSensor.class, "colorSensor");

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

            // **************************************************
            // autonomous path
            //
            // look for skystone
            // grab a stone
            // go over alliance bridge
            // place skystone on the foundation
            // re-position the foundation
            // park under the alliance bridge
            // stop
            //
            // **************************************************

            if (startAutonomous) { // https://stemrobotics.cs.pdx.edu/node/5184
                teleOp2Auto();
                if(elapsedTime > 17.158796605) {
                    stopMotor();
                    startAutonomous = false;
                }
            }
        }
        stopMotor();
    }
    public void pause() {
        sleep(100);
    }
    public void hookOn() {
        leftServo.setPosition(1);
        rightServo.setPosition(0);
    }
    public void hookOff() {
        leftServo.setPosition(0.1);
        rightServo.setPosition(0.9);
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
    private void rotate(int degrees, double power)
    {
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
        if (degrees < 0)
        {
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
        if(elapsedTime > 0 && elapsedTime < 1.586442658) {
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            gripMotor.setPower(0);
            armMotor.setPower(0);
        }
        if(elapsedTime > 1.586442658 && elapsedTime < 2.16636506) {
            leftBackMotor.setPower(0.00125);
            rightBackMotor.setPower(-0.00125);
            leftFrontMotor.setPower(0.00125);
            rightFrontMotor.setPower(-0.00125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.16636506 && elapsedTime < 2.206498814) {
            leftBackMotor.setPower(0.7377347312929418);
            rightBackMotor.setPower(0.7377347312929418);
            leftFrontMotor.setPower(0.7377347312929418);
            rightFrontMotor.setPower(0.7377347312929418);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.206498814 && elapsedTime < 2.233769754) {
            leftBackMotor.setPower(0.7502015004030008);
            rightBackMotor.setPower(0.7502015004030008);
            leftFrontMotor.setPower(0.7502015004030008);
            rightFrontMotor.setPower(0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.233769754 && elapsedTime < 2.289038145) {
            leftBackMotor.setPower(0.7489515004030008);
            rightBackMotor.setPower(0.7514515004030008);
            leftFrontMotor.setPower(0.7489515004030008);
            rightFrontMotor.setPower(0.7514515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.289038145 && elapsedTime < 2.320112158) {
            leftBackMotor.setPower(0.7514515004030008);
            rightBackMotor.setPower(0.7489515004030008);
            leftFrontMotor.setPower(0.7514515004030008);
            rightFrontMotor.setPower(0.7489515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.320112158 && elapsedTime < 2.359665027) {
            leftBackMotor.setPower(0.7552015004030008);
            rightBackMotor.setPower(0.7452015004030008);
            leftFrontMotor.setPower(0.7552015004030008);
            rightFrontMotor.setPower(0.7452015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.359665027 && elapsedTime < 2.399759406) {
            leftBackMotor.setPower(0.7539515004030009);
            rightBackMotor.setPower(0.7464515004030008);
            leftFrontMotor.setPower(0.7539515004030009);
            rightFrontMotor.setPower(0.7464515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.399759406 && elapsedTime < 2.428005398) {
            leftBackMotor.setPower(0.7527015004030008);
            rightBackMotor.setPower(0.7477015004030009);
            leftFrontMotor.setPower(0.7527015004030008);
            rightFrontMotor.setPower(0.7477015004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.428005398 && elapsedTime < 2.455505766) {
            leftBackMotor.setPower(0.7502015004030008);
            rightBackMotor.setPower(0.7502015004030008);
            leftFrontMotor.setPower(0.7502015004030008);
            rightFrontMotor.setPower(0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.455505766 && elapsedTime < 2.482988164) {
            leftBackMotor.setPower(0.7414515004030008);
            rightBackMotor.setPower(0.7589515004030009);
            leftFrontMotor.setPower(0.7414515004030008);
            rightFrontMotor.setPower(0.7589515004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.482988164 && elapsedTime < 2.510083219) {
            leftBackMotor.setPower(0.7377015004030009);
            rightBackMotor.setPower(0.7627015004030008);
            leftFrontMotor.setPower(0.7377015004030009);
            rightFrontMotor.setPower(0.7627015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.510083219 && elapsedTime < 2.538718482) {
            leftBackMotor.setPower(0.7302015004030008);
            rightBackMotor.setPower(0.7702015004030008);
            leftFrontMotor.setPower(0.7302015004030008);
            rightFrontMotor.setPower(0.7702015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.538718482 && elapsedTime < 2.562692547) {
            leftBackMotor.setPower(0.7277015004030009);
            rightBackMotor.setPower(0.7727015004030008);
            leftFrontMotor.setPower(0.7277015004030009);
            rightFrontMotor.setPower(0.7727015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.562692547 && elapsedTime < 2.585719476) {
            leftBackMotor.setPower(0.7227015004030009);
            rightBackMotor.setPower(0.7777015004030008);
            leftFrontMotor.setPower(0.7227015004030009);
            rightFrontMotor.setPower(0.7777015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.585719476 && elapsedTime < 2.609701979) {
            leftBackMotor.setPower(0.7189515004030008);
            rightBackMotor.setPower(0.7814515004030008);
            leftFrontMotor.setPower(0.7189515004030008);
            rightFrontMotor.setPower(0.7814515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.609701979 && elapsedTime < 2.681459903) {
            leftBackMotor.setPower(0.7139515004030008);
            rightBackMotor.setPower(0.7864515004030008);
            leftFrontMotor.setPower(0.7139515004030008);
            rightFrontMotor.setPower(0.7864515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.681459903 && elapsedTime < 2.702632509) {
            leftBackMotor.setPower(0.7102015004030008);
            rightBackMotor.setPower(0.7902015004030009);
            leftFrontMotor.setPower(0.7102015004030008);
            rightFrontMotor.setPower(0.7902015004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.702632509 && elapsedTime < 2.725758032) {
            leftBackMotor.setPower(0.7077015004030008);
            rightBackMotor.setPower(0.7927015004030008);
            leftFrontMotor.setPower(0.7077015004030008);
            rightFrontMotor.setPower(0.7927015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.725758032 && elapsedTime < 2.746381211) {
            leftBackMotor.setPower(0.7089515004030008);
            rightBackMotor.setPower(0.7914515004030008);
            leftFrontMotor.setPower(0.7089515004030008);
            rightFrontMotor.setPower(0.7914515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.746381211 && elapsedTime < 2.769636995) {
            leftBackMotor.setPower(0.7077015004030008);
            rightBackMotor.setPower(0.7927015004030008);
            leftFrontMotor.setPower(0.7077015004030008);
            rightFrontMotor.setPower(0.7927015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.769636995 && elapsedTime < 2.789697986) {
            leftBackMotor.setPower(0.7089515004030008);
            rightBackMotor.setPower(0.7914515004030008);
            leftFrontMotor.setPower(0.7089515004030008);
            rightFrontMotor.setPower(0.7914515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.789697986 && elapsedTime < 2.824170125) {
            leftBackMotor.setPower(0.7077015004030008);
            rightBackMotor.setPower(0.7927015004030008);
            leftFrontMotor.setPower(0.7077015004030008);
            rightFrontMotor.setPower(0.7927015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.824170125 && elapsedTime < 2.852154607) {
            leftBackMotor.setPower(0.7089515004030008);
            rightBackMotor.setPower(0.7914515004030008);
            leftFrontMotor.setPower(0.7089515004030008);
            rightFrontMotor.setPower(0.7914515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.852154607 && elapsedTime < 2.883072527) {
            leftBackMotor.setPower(0.7052015004030008);
            rightBackMotor.setPower(0.7952015004030009);
            leftFrontMotor.setPower(0.7052015004030008);
            rightFrontMotor.setPower(0.7952015004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.883072527 && elapsedTime < 2.924873) {
            leftBackMotor.setPower(0.7064515004030009);
            rightBackMotor.setPower(0.7939515004030008);
            leftFrontMotor.setPower(0.7064515004030009);
            rightFrontMotor.setPower(0.7939515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.924873 && elapsedTime < 2.965115087) {
            leftBackMotor.setPower(0.7077015004030008);
            rightBackMotor.setPower(0.7927015004030008);
            leftFrontMotor.setPower(0.7077015004030008);
            rightFrontMotor.setPower(0.7927015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.965115087 && elapsedTime < 2.988712069) {
            leftBackMotor.setPower(0.598010892166388);
            rightBackMotor.setPower(0.6855108921663879);
            leftFrontMotor.setPower(0.598010892166388);
            rightFrontMotor.setPower(0.6855108921663879);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 2.988712069 && elapsedTime < 3.008623477) {
            leftBackMotor.setPower(0.599260892166388);
            rightBackMotor.setPower(0.684260892166388);
            leftFrontMotor.setPower(0.599260892166388);
            rightFrontMotor.setPower(0.684260892166388);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.008623477 && elapsedTime < 3.029606656) {
            leftBackMotor.setPower(0.14963513821910332);
            rightBackMotor.setPower(0.23713513821910334);
            leftFrontMotor.setPower(0.14963513821910332);
            rightFrontMotor.setPower(0.23713513821910334);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.029606656 && elapsedTime < 3.049763221) {
            leftBackMotor.setPower(0.14838513821910332);
            rightBackMotor.setPower(0.23838513821910334);
            leftFrontMotor.setPower(0.14838513821910332);
            rightFrontMotor.setPower(0.23838513821910334);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.049763221 && elapsedTime < 3.113207029) {
            leftBackMotor.setPower(-0.0425);
            rightBackMotor.setPower(0.0425);
            leftFrontMotor.setPower(-0.0425);
            rightFrontMotor.setPower(0.0425);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.113207029 && elapsedTime < 3.123758697) {
            leftBackMotor.setPower(-0.03625);
            rightBackMotor.setPower(0.03625);
            leftFrontMotor.setPower(-0.03625);
            rightFrontMotor.setPower(0.03625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.123758697 && elapsedTime < 3.14273823) {
            leftBackMotor.setPower(-0.03125);
            rightBackMotor.setPower(0.03125);
            leftFrontMotor.setPower(-0.03125);
            rightFrontMotor.setPower(0.03125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.14273823 && elapsedTime < 3.157004273) {
            leftBackMotor.setPower(-0.02);
            rightBackMotor.setPower(0.02);
            leftFrontMotor.setPower(-0.02);
            rightFrontMotor.setPower(0.02);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.157004273 && elapsedTime < 3.176009639) {
            leftBackMotor.setPower(-0.01625);
            rightBackMotor.setPower(0.01625);
            leftFrontMotor.setPower(-0.01625);
            rightFrontMotor.setPower(0.01625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.176009639 && elapsedTime < 3.193704797) {
            leftBackMotor.setPower(-0.0075);
            rightBackMotor.setPower(0.0075);
            leftFrontMotor.setPower(-0.0075);
            rightFrontMotor.setPower(0.0075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.193704797 && elapsedTime < 3.20354558) {
            leftBackMotor.setPower(0.00125);
            rightBackMotor.setPower(-0.00125);
            leftFrontMotor.setPower(0.00125);
            rightFrontMotor.setPower(-0.00125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.20354558 && elapsedTime < 3.214456987) {
            leftBackMotor.setPower(0.00375);
            rightBackMotor.setPower(-0.00375);
            leftFrontMotor.setPower(0.00375);
            rightFrontMotor.setPower(-0.00375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.214456987 && elapsedTime < 3.225481415) {
            leftBackMotor.setPower(0.0075);
            rightBackMotor.setPower(-0.0075);
            leftFrontMotor.setPower(0.0075);
            rightFrontMotor.setPower(-0.0075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.225481415 && elapsedTime < 3.235578395) {
            leftBackMotor.setPower(0.01);
            rightBackMotor.setPower(-0.01);
            leftFrontMotor.setPower(0.01);
            rightFrontMotor.setPower(-0.01);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.235578395 && elapsedTime < 3.303561788) {
            leftBackMotor.setPower(0.0125);
            rightBackMotor.setPower(-0.0125);
            leftFrontMotor.setPower(0.0125);
            rightFrontMotor.setPower(-0.0125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.303561788 && elapsedTime < 3.316480174) {
            leftBackMotor.setPower(0.01125);
            rightBackMotor.setPower(-0.01125);
            leftFrontMotor.setPower(0.01125);
            rightFrontMotor.setPower(-0.01125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.316480174 && elapsedTime < 3.330100749) {
            leftBackMotor.setPower(0.01);
            rightBackMotor.setPower(-0.01);
            leftFrontMotor.setPower(0.01);
            rightFrontMotor.setPower(-0.01);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.330100749 && elapsedTime < 3.342613979) {
            leftBackMotor.setPower(0.00875);
            rightBackMotor.setPower(-0.00875);
            leftFrontMotor.setPower(0.00875);
            rightFrontMotor.setPower(-0.00875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.342613979 && elapsedTime < 3.355448511) {
            leftBackMotor.setPower(0.0075);
            rightBackMotor.setPower(-0.0075);
            leftFrontMotor.setPower(0.0075);
            rightFrontMotor.setPower(-0.0075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.355448511 && elapsedTime < 3.368227836) {
            leftBackMotor.setPower(0.00625);
            rightBackMotor.setPower(-0.00625);
            leftFrontMotor.setPower(0.00625);
            rightFrontMotor.setPower(-0.00625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.368227836 && elapsedTime < 3.379403722) {
            leftBackMotor.setPower(0.00375);
            rightBackMotor.setPower(-0.00375);
            leftFrontMotor.setPower(0.00375);
            rightFrontMotor.setPower(-0.00375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.379403722 && elapsedTime < 3.388606744) {
            leftBackMotor.setPower(0.0025);
            rightBackMotor.setPower(-0.0025);
            leftFrontMotor.setPower(0.0025);
            rightFrontMotor.setPower(-0.0025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.388606744 && elapsedTime < 3.40437461) {
            leftBackMotor.setPower(0.00125);
            rightBackMotor.setPower(-0.00125);
            leftFrontMotor.setPower(0.00125);
            rightFrontMotor.setPower(-0.00125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.40437461 && elapsedTime < 3.571786241) {
            leftBackMotor.setPower(-0.00125);
            rightBackMotor.setPower(0.00125);
            leftFrontMotor.setPower(-0.00125);
            rightFrontMotor.setPower(0.00125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.571786241 && elapsedTime < 3.609635412) {
            leftBackMotor.setPower(0.12463241380882835);
            rightBackMotor.setPower(-0.12463241380882835);
            leftFrontMotor.setPower(0.12463241380882835);
            rightFrontMotor.setPower(-0.12463241380882835);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.609635412 && elapsedTime < 3.731035163) {
            leftBackMotor.setPower(0.2196452633608522);
            rightBackMotor.setPower(-0.2196452633608522);
            leftFrontMotor.setPower(0.2196452633608522);
            rightFrontMotor.setPower(-0.2196452633608522);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.731035163 && elapsedTime < 3.765117771) {
            leftBackMotor.setPower(0.22647143328998318);
            rightBackMotor.setPower(-0.22647143328998318);
            leftFrontMotor.setPower(0.22647143328998318);
            rightFrontMotor.setPower(-0.22647143328998318);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.765117771 && elapsedTime < 3.80562314) {
            leftBackMotor.setPower(0.2621691676654319);
            rightBackMotor.setPower(-0.2621691676654319);
            leftFrontMotor.setPower(0.2621691676654319);
            rightFrontMotor.setPower(-0.2621691676654319);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.80562314 && elapsedTime < 3.855026686) {
            leftBackMotor.setPower(0.3004782393411393);
            rightBackMotor.setPower(-0.3004782393411393);
            leftFrontMotor.setPower(0.3004782393411393);
            rightFrontMotor.setPower(-0.3004782393411393);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.855026686 && elapsedTime < 3.886851273) {
            leftBackMotor.setPower(0.33300575449965014);
            rightBackMotor.setPower(-0.33300575449965014);
            leftFrontMotor.setPower(0.33300575449965014);
            rightFrontMotor.setPower(-0.33300575449965014);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.886851273 && elapsedTime < 3.919105026) {
            leftBackMotor.setPower(0.40307459913129406);
            rightBackMotor.setPower(-0.40307459913129406);
            leftFrontMotor.setPower(0.40307459913129406);
            rightFrontMotor.setPower(-0.40307459913129406);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.919105026 && elapsedTime < 3.97476149) {
            leftBackMotor.setPower(0.5103348476400692);
            rightBackMotor.setPower(-0.5103348476400692);
            leftFrontMotor.setPower(0.5103348476400692);
            rightFrontMotor.setPower(-0.5103348476400692);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 3.97476149 && elapsedTime < 4.127228588) {
            leftBackMotor.setPower(0.5851721005083078);
            rightBackMotor.setPower(-0.5851721005083078);
            leftFrontMotor.setPower(0.5851721005083078);
            rightFrontMotor.setPower(-0.5851721005083078);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.127228588 && elapsedTime < 4.375473926) {
            leftBackMotor.setPower(0.5962809479249688);
            rightBackMotor.setPower(-0.5962809479249688);
            leftFrontMotor.setPower(0.5962809479249688);
            rightFrontMotor.setPower(-0.5962809479249688);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.375473926 && elapsedTime < 4.415138617) {
            leftBackMotor.setPower(0.6302342229198168);
            rightBackMotor.setPower(-0.6302342229198168);
            leftFrontMotor.setPower(0.6302342229198168);
            rightFrontMotor.setPower(-0.6302342229198168);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.415138617 && elapsedTime < 4.490323468) {
            leftBackMotor.setPower(0.6533920168704366);
            rightBackMotor.setPower(-0.6533920168704366);
            leftFrontMotor.setPower(0.6533920168704366);
            rightFrontMotor.setPower(-0.6533920168704366);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.490323468 && elapsedTime < 4.853514234) {
            leftBackMotor.setPower(0.6651275970319626);
            rightBackMotor.setPower(-0.6651275970319626);
            leftFrontMotor.setPower(0.6651275970319626);
            rightFrontMotor.setPower(-0.6651275970319626);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.853514234 && elapsedTime < 4.893309029) {
            leftBackMotor.setPower(0.3760153254927976);
            rightBackMotor.setPower(-0.3760153254927976);
            leftFrontMotor.setPower(0.3760153254927976);
            rightFrontMotor.setPower(-0.3760153254927976);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.893309029 && elapsedTime < 4.915751271) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.915751271 && elapsedTime < 4.928506429) {
            leftBackMotor.setPower(-0.12125);
            rightBackMotor.setPower(0.12125);
            leftFrontMotor.setPower(-0.12125);
            rightFrontMotor.setPower(0.12125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.928506429 && elapsedTime < 4.941443722) {
            leftBackMotor.setPower(-0.16125);
            rightBackMotor.setPower(0.16125);
            leftFrontMotor.setPower(-0.16125);
            rightFrontMotor.setPower(0.16125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.941443722 && elapsedTime < 4.95442789) {
            leftBackMotor.setPower(-0.19875);
            rightBackMotor.setPower(0.19875);
            leftFrontMotor.setPower(-0.19875);
            rightFrontMotor.setPower(0.19875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.95442789 && elapsedTime < 4.965794297) {
            leftBackMotor.setPower(-0.2675);
            rightBackMotor.setPower(0.2675);
            leftFrontMotor.setPower(-0.2675);
            rightFrontMotor.setPower(0.2675);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.965794297 && elapsedTime < 4.978366382) {
            leftBackMotor.setPower(-0.29625);
            rightBackMotor.setPower(0.29625);
            leftFrontMotor.setPower(-0.29625);
            rightFrontMotor.setPower(0.29625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.978366382 && elapsedTime < 4.9923993) {
            leftBackMotor.setPower(-0.32);
            rightBackMotor.setPower(0.32);
            leftFrontMotor.setPower(-0.32);
            rightFrontMotor.setPower(0.32);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 4.9923993 && elapsedTime < 5.00359628) {
            leftBackMotor.setPower(-0.34);
            rightBackMotor.setPower(0.34);
            leftFrontMotor.setPower(-0.34);
            rightFrontMotor.setPower(0.34);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.00359628 && elapsedTime < 5.015225396) {
            leftBackMotor.setPower(-0.35625);
            rightBackMotor.setPower(0.35625);
            leftFrontMotor.setPower(-0.35625);
            rightFrontMotor.setPower(0.35625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.015225396 && elapsedTime < 5.027571543) {
            leftBackMotor.setPower(-0.3775);
            rightBackMotor.setPower(0.3775);
            leftFrontMotor.setPower(-0.3775);
            rightFrontMotor.setPower(0.3775);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.027571543 && elapsedTime < 5.039744252) {
            leftBackMotor.setPower(-0.38125000000000003);
            rightBackMotor.setPower(0.38125000000000003);
            leftFrontMotor.setPower(-0.38125000000000003);
            rightFrontMotor.setPower(0.38125000000000003);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.039744252 && elapsedTime < 5.052936024) {
            leftBackMotor.setPower(-0.3825);
            rightBackMotor.setPower(0.3825);
            leftFrontMotor.setPower(-0.3825);
            rightFrontMotor.setPower(0.3825);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.052936024 && elapsedTime < 5.063595036) {
            leftBackMotor.setPower(-0.38375000000000004);
            rightBackMotor.setPower(0.38375000000000004);
            leftFrontMotor.setPower(-0.38375000000000004);
            rightFrontMotor.setPower(0.38375000000000004);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.063595036 && elapsedTime < 5.07268988) {
            leftBackMotor.setPower(-0.3825);
            rightBackMotor.setPower(0.3825);
            leftFrontMotor.setPower(-0.3825);
            rightFrontMotor.setPower(0.3825);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.07268988 && elapsedTime < 5.163842077) {
            leftBackMotor.setPower(-0.38125000000000003);
            rightBackMotor.setPower(0.38125000000000003);
            leftFrontMotor.setPower(-0.38125000000000003);
            rightFrontMotor.setPower(0.38125000000000003);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.163842077 && elapsedTime < 5.17903312) {
            leftBackMotor.setPower(-0.38);
            rightBackMotor.setPower(0.38);
            leftFrontMotor.setPower(-0.38);
            rightFrontMotor.setPower(0.38);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.17903312 && elapsedTime < 5.229977865) {
            leftBackMotor.setPower(-0.38125000000000003);
            rightBackMotor.setPower(0.38125000000000003);
            leftFrontMotor.setPower(-0.38125000000000003);
            rightFrontMotor.setPower(0.38125000000000003);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.229977865 && elapsedTime < 5.27475313) {
            leftBackMotor.setPower(-0.38);
            rightBackMotor.setPower(0.38);
            leftFrontMotor.setPower(-0.38);
            rightFrontMotor.setPower(0.38);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.27475313 && elapsedTime < 5.294630267) {
            leftBackMotor.setPower(-0.38125000000000003);
            rightBackMotor.setPower(0.38125000000000003);
            leftFrontMotor.setPower(-0.38125000000000003);
            rightFrontMotor.setPower(0.38125000000000003);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.294630267 && elapsedTime < 5.326682041) {
            leftBackMotor.setPower(-0.10505742718182999);
            rightBackMotor.setPower(0.10505742718182999);
            leftFrontMotor.setPower(-0.10505742718182999);
            rightFrontMotor.setPower(0.10505742718182999);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.326682041 && elapsedTime < 5.372613192) {
            leftBackMotor.setPower(-0.6302342229198168);
            rightBackMotor.setPower(0.6302342229198168);
            leftFrontMotor.setPower(-0.6302342229198168);
            rightFrontMotor.setPower(0.6302342229198168);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.372613192 && elapsedTime < 5.407369237) {
            leftBackMotor.setPower(-0.7502015004030008);
            rightBackMotor.setPower(0.7502015004030008);
            leftFrontMotor.setPower(-0.7502015004030008);
            rightFrontMotor.setPower(0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.407369237 && elapsedTime < 5.463166013) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.463166013 && elapsedTime < 5.473158514) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.473158514 && elapsedTime < 5.48410414) {
            leftBackMotor.setPower(0.0525);
            rightBackMotor.setPower(-0.0525);
            leftFrontMotor.setPower(0.0525);
            rightFrontMotor.setPower(-0.0525);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.48410414 && elapsedTime < 5.494344089) {
            leftBackMotor.setPower(0.07375);
            rightBackMotor.setPower(-0.07375);
            leftFrontMotor.setPower(0.07375);
            rightFrontMotor.setPower(-0.07375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.494344089 && elapsedTime < 5.503854767) {
            leftBackMotor.setPower(0.08875);
            rightBackMotor.setPower(-0.08875);
            leftFrontMotor.setPower(0.08875);
            rightFrontMotor.setPower(-0.08875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.503854767 && elapsedTime < 5.512923362) {
            leftBackMotor.setPower(0.10125);
            rightBackMotor.setPower(-0.10125);
            leftFrontMotor.setPower(0.10125);
            rightFrontMotor.setPower(-0.10125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.512923362 && elapsedTime < 5.521743154) {
            leftBackMotor.setPower(0.11);
            rightBackMotor.setPower(-0.11);
            leftFrontMotor.setPower(0.11);
            rightFrontMotor.setPower(-0.11);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.521743154 && elapsedTime < 5.532100187) {
            leftBackMotor.setPower(0.115);
            rightBackMotor.setPower(-0.115);
            leftFrontMotor.setPower(0.115);
            rightFrontMotor.setPower(-0.115);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.532100187 && elapsedTime < 5.54260774) {
            leftBackMotor.setPower(0.11625);
            rightBackMotor.setPower(-0.11625);
            leftFrontMotor.setPower(0.11625);
            rightFrontMotor.setPower(-0.11625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.54260774 && elapsedTime < 5.563124981) {
            leftBackMotor.setPower(0.115);
            rightBackMotor.setPower(-0.115);
            leftFrontMotor.setPower(0.115);
            rightFrontMotor.setPower(-0.115);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.563124981 && elapsedTime < 5.575042951) {
            leftBackMotor.setPower(0.11375);
            rightBackMotor.setPower(-0.11375);
            leftFrontMotor.setPower(0.11375);
            rightFrontMotor.setPower(-0.11375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.575042951 && elapsedTime < 5.60303436) {
            leftBackMotor.setPower(0.1125);
            rightBackMotor.setPower(-0.1125);
            leftFrontMotor.setPower(0.1125);
            rightFrontMotor.setPower(-0.1125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.60303436 && elapsedTime < 5.641928218) {
            leftBackMotor.setPower(0.5935785334581443);
            rightBackMotor.setPower(0.3660785334581443);
            leftFrontMotor.setPower(0.5935785334581443);
            rightFrontMotor.setPower(0.3660785334581443);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.641928218 && elapsedTime < 5.66932843) {
            leftBackMotor.setPower(0.5923285334581443);
            rightBackMotor.setPower(0.3673285334581443);
            leftFrontMotor.setPower(0.5923285334581443);
            rightFrontMotor.setPower(0.3673285334581443);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.66932843 && elapsedTime < 5.726144789) {
            leftBackMotor.setPower(0.8402347312929418);
            rightBackMotor.setPower(0.6352347312929417);
            leftFrontMotor.setPower(0.8402347312929418);
            rightFrontMotor.setPower(0.6352347312929417);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.726144789 && elapsedTime < 5.766260054) {
            leftBackMotor.setPower(0.8264515004030009);
            rightBackMotor.setPower(0.6739515004030008);
            leftFrontMotor.setPower(0.8264515004030009);
            rightFrontMotor.setPower(0.6739515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.766260054 && elapsedTime < 5.795807661) {
            leftBackMotor.setPower(0.8064515004030008);
            rightBackMotor.setPower(0.6939515004030008);
            leftFrontMotor.setPower(0.8064515004030008);
            rightFrontMotor.setPower(0.6939515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.795807661 && elapsedTime < 5.822346726) {
            leftBackMotor.setPower(0.7877015004030008);
            rightBackMotor.setPower(0.7127015004030008);
            leftFrontMotor.setPower(0.7877015004030008);
            rightFrontMotor.setPower(0.7127015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.822346726 && elapsedTime < 5.847250999) {
            leftBackMotor.setPower(0.7739515004030009);
            rightBackMotor.setPower(0.7264515004030008);
            leftFrontMotor.setPower(0.7739515004030009);
            rightFrontMotor.setPower(0.7264515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.847250999 && elapsedTime < 5.874160898) {
            leftBackMotor.setPower(0.7564515004030008);
            rightBackMotor.setPower(0.7439515004030008);
            leftFrontMotor.setPower(0.7564515004030008);
            rightFrontMotor.setPower(0.7439515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.874160898 && elapsedTime < 5.910592881) {
            leftBackMotor.setPower(0.7427015004030009);
            rightBackMotor.setPower(0.7577015004030008);
            leftFrontMotor.setPower(0.7427015004030009);
            rightFrontMotor.setPower(0.7577015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.910592881 && elapsedTime < 5.930607362) {
            leftBackMotor.setPower(0.7227015004030009);
            rightBackMotor.setPower(0.7777015004030008);
            leftFrontMotor.setPower(0.7227015004030009);
            rightFrontMotor.setPower(0.7777015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.930607362 && elapsedTime < 5.951584864) {
            leftBackMotor.setPower(0.7139515004030008);
            rightBackMotor.setPower(0.7864515004030008);
            leftFrontMotor.setPower(0.7139515004030008);
            rightFrontMotor.setPower(0.7864515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.951584864 && elapsedTime < 5.971608355) {
            leftBackMotor.setPower(0.7077015004030008);
            rightBackMotor.setPower(0.7927015004030008);
            leftFrontMotor.setPower(0.7077015004030008);
            rightFrontMotor.setPower(0.7927015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.971608355 && elapsedTime < 5.99158367) {
            leftBackMotor.setPower(0.7027015004030008);
            rightBackMotor.setPower(0.7977015004030008);
            leftFrontMotor.setPower(0.7027015004030008);
            rightFrontMotor.setPower(0.7977015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 5.99158367 && elapsedTime < 6.012613203) {
            leftBackMotor.setPower(0.7002015004030008);
            rightBackMotor.setPower(0.8002015004030009);
            leftFrontMotor.setPower(0.7002015004030008);
            rightFrontMotor.setPower(0.8002015004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.012613203 && elapsedTime < 6.033720028) {
            leftBackMotor.setPower(0.6977015004030008);
            rightBackMotor.setPower(0.8027015004030008);
            leftFrontMotor.setPower(0.6977015004030008);
            rightFrontMotor.setPower(0.8027015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.033720028 && elapsedTime < 6.056230395) {
            leftBackMotor.setPower(0.6952015004030008);
            rightBackMotor.setPower(0.8052015004030009);
            leftFrontMotor.setPower(0.6952015004030008);
            rightFrontMotor.setPower(0.8052015004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.056230395 && elapsedTime < 6.085454669) {
            leftBackMotor.setPower(0.7502015004030008);
            rightBackMotor.setPower(0.7502015004030008);
            leftFrontMotor.setPower(0.7502015004030008);
            rightFrontMotor.setPower(0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.085454669 && elapsedTime < 6.136639414) {
            leftBackMotor.setPower(0.19772928614555474);
            rightBackMotor.setPower(1.3026737146604468);
            leftFrontMotor.setPower(0.19772928614555474);
            rightFrontMotor.setPower(1.3026737146604468);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.136639414 && elapsedTime < 6.218222963) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(1.5004030008060016);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(1.5004030008060016);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.218222963 && elapsedTime < 6.251093852) {
            leftBackMotor.setPower(-0.22948899429480374);
            rightBackMotor.setPower(1.270914006511198);
            leftFrontMotor.setPower(-0.22948899429480374);
            rightFrontMotor.setPower(1.270914006511198);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.251093852 && elapsedTime < 6.694758167) {
            leftBackMotor.setPower(-0.7502015004030008);
            rightBackMotor.setPower(0.7502015004030008);
            leftFrontMotor.setPower(-0.7502015004030008);
            rightFrontMotor.setPower(0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.694758167 && elapsedTime < 6.754583329) {
            leftBackMotor.setPower(-0.24757672719749152);
            rightBackMotor.setPower(0.24757672719749152);
            leftFrontMotor.setPower(-0.24757672719749152);
            rightFrontMotor.setPower(0.24757672719749152);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.754583329 && elapsedTime < 6.792209635) {
            leftBackMotor.setPower(0.25375000000000003);
            rightBackMotor.setPower(-0.25375000000000003);
            leftFrontMotor.setPower(0.25375000000000003);
            rightFrontMotor.setPower(-0.25375000000000003);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.792209635 && elapsedTime < 6.812387137) {
            leftBackMotor.setPower(1.1306145594452564);
            rightBackMotor.setPower(0.2956145594452563);
            leftFrontMotor.setPower(1.1306145594452564);
            rightFrontMotor.setPower(0.2956145594452563);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.812387137 && elapsedTime < 6.827236201) {
            leftBackMotor.setPower(1.2427015004030009);
            rightBackMotor.setPower(0.2577015004030008);
            leftFrontMotor.setPower(1.2427015004030009);
            rightFrontMotor.setPower(0.2577015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.827236201 && elapsedTime < 6.842078703) {
            leftBackMotor.setPower(1.3064515004030008);
            rightBackMotor.setPower(0.1939515004030008);
            leftFrontMotor.setPower(1.3064515004030008);
            rightFrontMotor.setPower(0.1939515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.842078703 && elapsedTime < 6.856226048) {
            leftBackMotor.setPower(1.3302015004030008);
            rightBackMotor.setPower(0.17020150040300086);
            leftFrontMotor.setPower(1.3302015004030008);
            rightFrontMotor.setPower(0.17020150040300086);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.856226048 && elapsedTime < 6.875767977) {
            leftBackMotor.setPower(1.366451500403001);
            rightBackMotor.setPower(0.13395150040300086);
            leftFrontMotor.setPower(1.366451500403001);
            rightFrontMotor.setPower(0.13395150040300086);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.875767977 && elapsedTime < 6.894856468) {
            leftBackMotor.setPower(1.3789515004030009);
            rightBackMotor.setPower(0.12145150040300079);
            leftFrontMotor.setPower(1.3789515004030009);
            rightFrontMotor.setPower(0.12145150040300079);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.894856468 && elapsedTime < 6.911377303) {
            leftBackMotor.setPower(1.3927015004030008);
            rightBackMotor.setPower(0.10770150040300086);
            leftFrontMotor.setPower(1.3927015004030008);
            rightFrontMotor.setPower(0.10770150040300086);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.911377303 && elapsedTime < 6.927352722) {
            leftBackMotor.setPower(1.4002015004030008);
            rightBackMotor.setPower(0.1002015004030008);
            leftFrontMotor.setPower(1.4002015004030008);
            rightFrontMotor.setPower(0.1002015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.927352722 && elapsedTime < 6.944367723) {
            leftBackMotor.setPower(1.4014515004030008);
            rightBackMotor.setPower(0.09895150040300082);
            leftFrontMotor.setPower(1.4014515004030008);
            rightFrontMotor.setPower(0.09895150040300082);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.944367723 && elapsedTime < 6.965562204) {
            leftBackMotor.setPower(1.3989515004030009);
            rightBackMotor.setPower(0.10145150040300077);
            leftFrontMotor.setPower(1.3989515004030009);
            rightFrontMotor.setPower(0.10145150040300077);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.965562204 && elapsedTime < 6.980249341) {
            leftBackMotor.setPower(1.3914515004030008);
            rightBackMotor.setPower(0.10895150040300083);
            leftFrontMotor.setPower(1.3914515004030008);
            rightFrontMotor.setPower(0.10895150040300083);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.980249341 && elapsedTime < 6.99564752) {
            leftBackMotor.setPower(1.3777015004030009);
            rightBackMotor.setPower(0.12270150040300076);
            leftFrontMotor.setPower(1.3777015004030009);
            rightFrontMotor.setPower(0.12270150040300076);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 6.99564752 && elapsedTime < 7.010335646) {
            leftBackMotor.setPower(1.3702015004030008);
            rightBackMotor.setPower(0.13020150040300082);
            leftFrontMotor.setPower(1.3702015004030008);
            rightFrontMotor.setPower(0.13020150040300082);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.010335646 && elapsedTime < 7.030008461) {
            leftBackMotor.setPower(1.3489515004030008);
            rightBackMotor.setPower(0.15145150040300082);
            leftFrontMotor.setPower(1.3489515004030008);
            rightFrontMotor.setPower(0.15145150040300082);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.030008461 && elapsedTime < 7.051006171) {
            leftBackMotor.setPower(1.3252015004030009);
            rightBackMotor.setPower(0.17520150040300075);
            leftFrontMotor.setPower(1.3252015004030009);
            rightFrontMotor.setPower(0.17520150040300075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.051006171 && elapsedTime < 7.072072215) {
            leftBackMotor.setPower(1.3002015004030008);
            rightBackMotor.setPower(0.20020150040300078);
            leftFrontMotor.setPower(1.3002015004030008);
            rightFrontMotor.setPower(0.20020150040300078);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.072072215 && elapsedTime < 7.091807634) {
            leftBackMotor.setPower(1.2714515004030007);
            rightBackMotor.setPower(0.22895150040300083);
            leftFrontMotor.setPower(1.2714515004030007);
            rightFrontMotor.setPower(0.22895150040300083);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.091807634 && elapsedTime < 7.113125396) {
            leftBackMotor.setPower(1.2402015004030007);
            rightBackMotor.setPower(0.26020150040300083);
            leftFrontMotor.setPower(1.2402015004030007);
            rightFrontMotor.setPower(0.26020150040300083);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.113125396 && elapsedTime < 7.132271023) {
            leftBackMotor.setPower(1.207701500403001);
            rightBackMotor.setPower(0.2927015004030008);
            leftFrontMotor.setPower(1.207701500403001);
            rightFrontMotor.setPower(0.2927015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.132271023 && elapsedTime < 7.150140296) {
            leftBackMotor.setPower(1.1739515004030008);
            rightBackMotor.setPower(0.3264515004030008);
            leftFrontMotor.setPower(1.1739515004030008);
            rightFrontMotor.setPower(0.3264515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.150140296 && elapsedTime < 7.169953527) {
            leftBackMotor.setPower(1.1402015004030008);
            rightBackMotor.setPower(0.3602015004030008);
            leftFrontMotor.setPower(1.1402015004030008);
            rightFrontMotor.setPower(0.3602015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.169953527 && elapsedTime < 7.18811155) {
            leftBackMotor.setPower(1.1064515004030009);
            rightBackMotor.setPower(0.3939515004030008);
            leftFrontMotor.setPower(1.1064515004030009);
            rightFrontMotor.setPower(0.3939515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.18811155 && elapsedTime < 7.207108791) {
            leftBackMotor.setPower(1.0727015004030007);
            rightBackMotor.setPower(0.4277015004030008);
            leftFrontMotor.setPower(1.0727015004030007);
            rightFrontMotor.setPower(0.4277015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.207108791 && elapsedTime < 7.247500514) {
            leftBackMotor.setPower(0.4696317864364227);
            rightBackMotor.setPower(-0.10786821356357729);
            leftFrontMotor.setPower(0.4696317864364227);
            rightFrontMotor.setPower(-0.10786821356357729);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.247500514 && elapsedTime < 7.27449385) {
            leftBackMotor.setPower(0.23);
            rightBackMotor.setPower(-0.23);
            leftFrontMotor.setPower(0.23);
            rightFrontMotor.setPower(-0.23);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.27449385 && elapsedTime < 7.31052552) {
            leftBackMotor.setPower(0.2);
            rightBackMotor.setPower(-0.2);
            leftFrontMotor.setPower(0.2);
            rightFrontMotor.setPower(-0.2);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.31052552 && elapsedTime < 7.332648179) {
            leftBackMotor.setPower(0.12625);
            rightBackMotor.setPower(-0.12625);
            leftFrontMotor.setPower(0.12625);
            rightFrontMotor.setPower(-0.12625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.332648179 && elapsedTime < 7.346436565) {
            leftBackMotor.setPower(0.08375);
            rightBackMotor.setPower(-0.08375);
            leftFrontMotor.setPower(0.08375);
            rightFrontMotor.setPower(-0.08375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.346436565 && elapsedTime < 7.356449431) {
            leftBackMotor.setPower(0.0625);
            rightBackMotor.setPower(-0.0625);
            leftFrontMotor.setPower(0.0625);
            rightFrontMotor.setPower(-0.0625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.356449431 && elapsedTime < 7.366196151) {
            leftBackMotor.setPower(0.04);
            rightBackMotor.setPower(-0.04);
            leftFrontMotor.setPower(0.04);
            rightFrontMotor.setPower(-0.04);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.366196151 && elapsedTime < 7.376475631) {
            leftBackMotor.setPower(0.0175);
            rightBackMotor.setPower(-0.0175);
            leftFrontMotor.setPower(0.0175);
            rightFrontMotor.setPower(-0.0175);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.376475631 && elapsedTime < 7.388334174) {
            leftBackMotor.setPower(-0.00375);
            rightBackMotor.setPower(0.00375);
            leftFrontMotor.setPower(-0.00375);
            rightFrontMotor.setPower(0.00375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.388334174 && elapsedTime < 7.398343602) {
            leftBackMotor.setPower(-0.03875);
            rightBackMotor.setPower(0.03875);
            leftFrontMotor.setPower(-0.03875);
            rightFrontMotor.setPower(0.03875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.398343602 && elapsedTime < 7.408297405) {
            leftBackMotor.setPower(-0.0525);
            rightBackMotor.setPower(0.0525);
            leftFrontMotor.setPower(-0.0525);
            rightFrontMotor.setPower(0.0525);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.408297405 && elapsedTime < 7.420416833) {
            leftBackMotor.setPower(-0.065);
            rightBackMotor.setPower(0.065);
            leftFrontMotor.setPower(-0.065);
            rightFrontMotor.setPower(0.065);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.420416833 && elapsedTime < 7.433144491) {
            leftBackMotor.setPower(-0.075);
            rightBackMotor.setPower(0.075);
            leftFrontMotor.setPower(-0.075);
            rightFrontMotor.setPower(0.075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.433144491 && elapsedTime < 7.44662345) {
            leftBackMotor.setPower(-0.08375);
            rightBackMotor.setPower(0.08375);
            leftFrontMotor.setPower(-0.08375);
            rightFrontMotor.setPower(0.08375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.44662345 && elapsedTime < 7.459071524) {
            leftBackMotor.setPower(-0.0925);
            rightBackMotor.setPower(0.0925);
            leftFrontMotor.setPower(-0.0925);
            rightFrontMotor.setPower(0.0925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.459071524 && elapsedTime < 7.469586578) {
            leftBackMotor.setPower(-0.105);
            rightBackMotor.setPower(0.105);
            leftFrontMotor.setPower(-0.105);
            rightFrontMotor.setPower(0.105);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.469586578 && elapsedTime < 7.480420797) {
            leftBackMotor.setPower(-0.10875);
            rightBackMotor.setPower(0.10875);
            leftFrontMotor.setPower(-0.10875);
            rightFrontMotor.setPower(0.10875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.480420797 && elapsedTime < 7.491395694) {
            leftBackMotor.setPower(-0.11);
            rightBackMotor.setPower(0.11);
            leftFrontMotor.setPower(-0.11);
            rightFrontMotor.setPower(0.11);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.491395694 && elapsedTime < 7.501490122) {
            leftBackMotor.setPower(-0.11125);
            rightBackMotor.setPower(0.11125);
            leftFrontMotor.setPower(-0.11125);
            rightFrontMotor.setPower(0.11125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.501490122 && elapsedTime < 7.512497884) {
            leftBackMotor.setPower(-0.1125);
            rightBackMotor.setPower(0.1125);
            leftFrontMotor.setPower(-0.1125);
            rightFrontMotor.setPower(0.1125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.512497884 && elapsedTime < 7.528406688) {
            leftBackMotor.setPower(-0.11375);
            rightBackMotor.setPower(0.11375);
            leftFrontMotor.setPower(-0.11375);
            rightFrontMotor.setPower(0.11375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.528406688 && elapsedTime < 7.539350907) {
            leftBackMotor.setPower(-0.115);
            rightBackMotor.setPower(0.115);
            leftFrontMotor.setPower(-0.115);
            rightFrontMotor.setPower(0.115);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.539350907 && elapsedTime < 7.599636955) {
            leftBackMotor.setPower(-0.11625);
            rightBackMotor.setPower(0.11625);
            leftFrontMotor.setPower(-0.11625);
            rightFrontMotor.setPower(0.11625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.599636955 && elapsedTime < 7.652988835) {
            leftBackMotor.setPower(-0.115);
            rightBackMotor.setPower(0.115);
            leftFrontMotor.setPower(-0.115);
            rightFrontMotor.setPower(0.115);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.652988835 && elapsedTime < 7.679519828) {
            leftBackMotor.setPower(0.14659761512787917);
            rightBackMotor.setPower(-0.14659761512787917);
            leftFrontMotor.setPower(-0.37659761512787915);
            rightFrontMotor.setPower(0.37659761512787915);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.679519828 && elapsedTime < 7.709399883) {
            leftBackMotor.setPower(0.14784761512787914);
            rightBackMotor.setPower(-0.14784761512787914);
            leftFrontMotor.setPower(-0.3753476151278792);
            rightFrontMotor.setPower(0.3753476151278792);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.709399883 && elapsedTime < 7.758789783) {
            leftBackMotor.setPower(0.51000124000248);
            rightBackMotor.setPower(-0.51000124000248);
            leftFrontMotor.setPower(-0.73000124000248);
            rightFrontMotor.setPower(0.73000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.758789783 && elapsedTime < 7.795107652) {
            leftBackMotor.setPower(0.52250124000248);
            rightBackMotor.setPower(-0.52250124000248);
            leftFrontMotor.setPower(-0.7175012400024801);
            rightFrontMotor.setPower(0.7175012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.795107652 && elapsedTime < 7.830349686) {
            leftBackMotor.setPower(0.5287512400024801);
            rightBackMotor.setPower(-0.5287512400024801);
            leftFrontMotor.setPower(-0.71125124000248);
            rightFrontMotor.setPower(0.71125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.830349686 && elapsedTime < 7.854688335) {
            leftBackMotor.setPower(0.53250124000248);
            rightBackMotor.setPower(-0.53250124000248);
            leftFrontMotor.setPower(-0.70750124000248);
            rightFrontMotor.setPower(0.70750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.854688335 && elapsedTime < 7.881613598) {
            leftBackMotor.setPower(0.53375124000248);
            rightBackMotor.setPower(-0.53375124000248);
            leftFrontMotor.setPower(-0.7062512400024801);
            rightFrontMotor.setPower(0.7062512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.881613598 && elapsedTime < 7.909749278) {
            leftBackMotor.setPower(0.53625124000248);
            rightBackMotor.setPower(-0.53625124000248);
            leftFrontMotor.setPower(-0.70375124000248);
            rightFrontMotor.setPower(0.70375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.909749278 && elapsedTime < 7.93639402) {
            leftBackMotor.setPower(0.53750124000248);
            rightBackMotor.setPower(-0.53750124000248);
            leftFrontMotor.setPower(-0.70250124000248);
            rightFrontMotor.setPower(0.70250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.93639402 && elapsedTime < 7.96052621) {
            leftBackMotor.setPower(0.54250124000248);
            rightBackMotor.setPower(-0.54250124000248);
            leftFrontMotor.setPower(-0.69750124000248);
            rightFrontMotor.setPower(0.69750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.96052621 && elapsedTime < 7.988524233) {
            leftBackMotor.setPower(0.54625124000248);
            rightBackMotor.setPower(-0.54625124000248);
            leftFrontMotor.setPower(-0.69375124000248);
            rightFrontMotor.setPower(0.69375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 7.988524233 && elapsedTime < 8.014787726) {
            leftBackMotor.setPower(0.55500124000248);
            rightBackMotor.setPower(-0.55500124000248);
            leftFrontMotor.setPower(-0.6850012400024801);
            rightFrontMotor.setPower(0.6850012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.014787726 && elapsedTime < 8.036187936) {
            leftBackMotor.setPower(0.55875124000248);
            rightBackMotor.setPower(-0.55875124000248);
            leftFrontMotor.setPower(-0.68125124000248);
            rightFrontMotor.setPower(0.68125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.036187936 && elapsedTime < 8.057273615) {
            leftBackMotor.setPower(0.5662512400024801);
            rightBackMotor.setPower(-0.5662512400024801);
            leftFrontMotor.setPower(-0.67375124000248);
            rightFrontMotor.setPower(0.67375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.057273615 && elapsedTime < 8.078096534) {
            leftBackMotor.setPower(0.57250124000248);
            rightBackMotor.setPower(-0.57250124000248);
            leftFrontMotor.setPower(-0.66750124000248);
            rightFrontMotor.setPower(0.66750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.078096534 && elapsedTime < 8.100094818) {
            leftBackMotor.setPower(0.57750124000248);
            rightBackMotor.setPower(-0.57750124000248);
            leftFrontMotor.setPower(-0.66250124000248);
            rightFrontMotor.setPower(0.66250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.100094818 && elapsedTime < 8.120088309) {
            leftBackMotor.setPower(0.58875124000248);
            rightBackMotor.setPower(-0.58875124000248);
            leftFrontMotor.setPower(-0.65125124000248);
            rightFrontMotor.setPower(0.65125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.120088309 && elapsedTime < 8.14410977) {
            leftBackMotor.setPower(0.5925012400024801);
            rightBackMotor.setPower(-0.5925012400024801);
            leftFrontMotor.setPower(-0.64750124000248);
            rightFrontMotor.setPower(0.64750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.14410977 && elapsedTime < 8.16883847) {
            leftBackMotor.setPower(0.60000124000248);
            rightBackMotor.setPower(-0.60000124000248);
            leftFrontMotor.setPower(-0.64000124000248);
            rightFrontMotor.setPower(0.64000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.16883847 && elapsedTime < 8.191096702) {
            leftBackMotor.setPower(0.6075012400024801);
            rightBackMotor.setPower(-0.6075012400024801);
            leftFrontMotor.setPower(-0.63250124000248);
            rightFrontMotor.setPower(0.63250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.191096702 && elapsedTime < 8.211118214) {
            leftBackMotor.setPower(0.61375124000248);
            rightBackMotor.setPower(-0.61375124000248);
            leftFrontMotor.setPower(-0.62625124000248);
            rightFrontMotor.setPower(0.62625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.211118214 && elapsedTime < 8.231113581) {
            leftBackMotor.setPower(0.61625124000248);
            rightBackMotor.setPower(-0.61625124000248);
            leftFrontMotor.setPower(-0.6237512400024801);
            rightFrontMotor.setPower(0.6237512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.231113581 && elapsedTime < 8.272785356) {
            leftBackMotor.setPower(0.61875124000248);
            rightBackMotor.setPower(-0.61875124000248);
            leftFrontMotor.setPower(-0.62125124000248);
            rightFrontMotor.setPower(0.62125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.272785356 && elapsedTime < 8.305189265) {
            leftBackMotor.setPower(0.62625124000248);
            rightBackMotor.setPower(-0.62625124000248);
            leftFrontMotor.setPower(-0.61375124000248);
            rightFrontMotor.setPower(0.61375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.305189265 && elapsedTime < 8.334942133) {
            leftBackMotor.setPower(0.63125124000248);
            rightBackMotor.setPower(-0.63125124000248);
            leftFrontMotor.setPower(-0.60875124000248);
            rightFrontMotor.setPower(0.60875124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.334942133 && elapsedTime < 8.365541302) {
            leftBackMotor.setPower(0.63500124000248);
            rightBackMotor.setPower(-0.63500124000248);
            leftFrontMotor.setPower(-0.60500124000248);
            rightFrontMotor.setPower(0.60500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.365541302 && elapsedTime < 8.391447867) {
            leftBackMotor.setPower(0.63625124000248);
            rightBackMotor.setPower(-0.63625124000248);
            leftFrontMotor.setPower(-0.60375124000248);
            rightFrontMotor.setPower(0.60375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.391447867 && elapsedTime < 8.412609536) {
            leftBackMotor.setPower(0.63750124000248);
            rightBackMotor.setPower(-0.63750124000248);
            leftFrontMotor.setPower(-0.6025012400024801);
            rightFrontMotor.setPower(0.6025012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.412609536 && elapsedTime < 8.444105685) {
            leftBackMotor.setPower(0.63625124000248);
            rightBackMotor.setPower(-0.63625124000248);
            leftFrontMotor.setPower(-0.60375124000248);
            rightFrontMotor.setPower(0.60375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.444105685 && elapsedTime < 8.466356) {
            leftBackMotor.setPower(0.63500124000248);
            rightBackMotor.setPower(-0.63500124000248);
            leftFrontMotor.setPower(-0.60500124000248);
            rightFrontMotor.setPower(0.60500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.466356 && elapsedTime < 8.507878087) {
            leftBackMotor.setPower(0.63625124000248);
            rightBackMotor.setPower(-0.63625124000248);
            leftFrontMotor.setPower(-0.60375124000248);
            rightFrontMotor.setPower(0.60375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.507878087 && elapsedTime < 8.527358037) {
            leftBackMotor.setPower(0.63750124000248);
            rightBackMotor.setPower(-0.63750124000248);
            leftFrontMotor.setPower(-0.6025012400024801);
            rightFrontMotor.setPower(0.6025012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.527358037 && elapsedTime < 8.553364394) {
            leftBackMotor.setPower(0.63500124000248);
            rightBackMotor.setPower(-0.63500124000248);
            leftFrontMotor.setPower(-0.60500124000248);
            rightFrontMotor.setPower(0.60500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.553364394 && elapsedTime < 8.590081481) {
            leftBackMotor.setPower(0.6337512400024801);
            rightBackMotor.setPower(-0.6337512400024801);
            leftFrontMotor.setPower(-0.60625124000248);
            rightFrontMotor.setPower(0.60625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.590081481 && elapsedTime < 8.614199556) {
            leftBackMotor.setPower(0.63500124000248);
            rightBackMotor.setPower(-0.63500124000248);
            leftFrontMotor.setPower(-0.60500124000248);
            rightFrontMotor.setPower(0.60500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.614199556 && elapsedTime < 8.654724821) {
            leftBackMotor.setPower(0.6337512400024801);
            rightBackMotor.setPower(-0.6337512400024801);
            leftFrontMotor.setPower(-0.60625124000248);
            rightFrontMotor.setPower(0.60625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.654724821 && elapsedTime < 8.682697063) {
            leftBackMotor.setPower(0.2958977337052269);
            rightBackMotor.setPower(-0.2958977337052269);
            leftFrontMotor.setPower(-0.2683977337052269);
            rightFrontMotor.setPower(0.2683977337052269);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.682697063 && elapsedTime < 8.709445347) {
            leftBackMotor.setPower(0.01375);
            rightBackMotor.setPower(-0.01375);
            leftFrontMotor.setPower(0.01375);
            rightFrontMotor.setPower(-0.01375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.709445347 && elapsedTime < 8.729245609) {
            leftBackMotor.setPower(0.0125);
            rightBackMotor.setPower(-0.0125);
            leftFrontMotor.setPower(0.0125);
            rightFrontMotor.setPower(-0.0125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.729245609 && elapsedTime < 8.762824779) {
            leftBackMotor.setPower(0.01375);
            rightBackMotor.setPower(-0.01375);
            leftFrontMotor.setPower(0.01375);
            rightFrontMotor.setPower(-0.01375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.762824779 && elapsedTime < 8.778753531) {
            leftBackMotor.setPower(0.01625);
            rightBackMotor.setPower(-0.01625);
            leftFrontMotor.setPower(0.01625);
            rightFrontMotor.setPower(-0.01625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.778753531 && elapsedTime < 8.787982282) {
            leftBackMotor.setPower(0.02);
            rightBackMotor.setPower(-0.02);
            leftFrontMotor.setPower(0.02);
            rightFrontMotor.setPower(-0.02);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.787982282 && elapsedTime < 8.797976554) {
            leftBackMotor.setPower(0.0225);
            rightBackMotor.setPower(-0.0225);
            leftFrontMotor.setPower(0.0225);
            rightFrontMotor.setPower(-0.0225);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.797976554 && elapsedTime < 8.814349212) {
            leftBackMotor.setPower(0.02375);
            rightBackMotor.setPower(-0.02375);
            leftFrontMotor.setPower(0.02375);
            rightFrontMotor.setPower(-0.02375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.814349212 && elapsedTime < 8.827963171) {
            leftBackMotor.setPower(0.03125);
            rightBackMotor.setPower(-0.03125);
            leftFrontMotor.setPower(0.03125);
            rightFrontMotor.setPower(-0.03125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.827963171 && elapsedTime < 8.844454944) {
            leftBackMotor.setPower(0.03625);
            rightBackMotor.setPower(-0.03625);
            leftFrontMotor.setPower(0.03625);
            rightFrontMotor.setPower(-0.03625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.844454944 && elapsedTime < 8.854277809) {
            leftBackMotor.setPower(0.045);
            rightBackMotor.setPower(-0.045);
            leftFrontMotor.setPower(0.045);
            rightFrontMotor.setPower(-0.045);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.854277809 && elapsedTime < 8.885502344) {
            leftBackMotor.setPower(0.051250000000000004);
            rightBackMotor.setPower(-0.051250000000000004);
            leftFrontMotor.setPower(0.051250000000000004);
            rightFrontMotor.setPower(-0.051250000000000004);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.885502344 && elapsedTime < 8.907606617) {
            leftBackMotor.setPower(-0.5055988625846672);
            rightBackMotor.setPower(0.5055988625846672);
            leftFrontMotor.setPower(0.6330988625846672);
            rightFrontMotor.setPower(-0.6330988625846672);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.907606617 && elapsedTime < 8.927608546) {
            leftBackMotor.setPower(-0.5030988625846672);
            rightBackMotor.setPower(0.5030988625846672);
            leftFrontMotor.setPower(0.6355988625846672);
            rightFrontMotor.setPower(-0.6355988625846672);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.927608546 && elapsedTime < 8.968123654) {
            leftBackMotor.setPower(-0.55375124000248);
            rightBackMotor.setPower(0.55375124000248);
            leftFrontMotor.setPower(0.6862512400024801);
            rightFrontMotor.setPower(-0.6862512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.968123654 && elapsedTime < 8.987891156) {
            leftBackMotor.setPower(-0.55250124000248);
            rightBackMotor.setPower(0.55250124000248);
            leftFrontMotor.setPower(0.68750124000248);
            rightFrontMotor.setPower(-0.68750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 8.987891156 && elapsedTime < 9.008964596) {
            leftBackMotor.setPower(-0.54750124000248);
            rightBackMotor.setPower(0.54750124000248);
            leftFrontMotor.setPower(0.69250124000248);
            rightFrontMotor.setPower(-0.69250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.008964596 && elapsedTime < 9.031894389) {
            leftBackMotor.setPower(-0.53375124000248);
            rightBackMotor.setPower(0.53375124000248);
            leftFrontMotor.setPower(0.7062512400024801);
            rightFrontMotor.setPower(-0.7062512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.031894389 && elapsedTime < 9.052246839) {
            leftBackMotor.setPower(-0.5137512400024801);
            rightBackMotor.setPower(0.5137512400024801);
            leftFrontMotor.setPower(0.72625124000248);
            rightFrontMotor.setPower(-0.72625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.052246839 && elapsedTime < 9.07691554) {
            leftBackMotor.setPower(-0.50500124000248);
            rightBackMotor.setPower(0.50500124000248);
            leftFrontMotor.setPower(0.73500124000248);
            rightFrontMotor.setPower(-0.73500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.07691554 && elapsedTime < 9.10531247) {
            leftBackMotor.setPower(-0.49375124000248005);
            rightBackMotor.setPower(0.49375124000248005);
            leftFrontMotor.setPower(0.74625124000248);
            rightFrontMotor.setPower(-0.74625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.10531247 && elapsedTime < 9.131341483) {
            leftBackMotor.setPower(-0.47500124000248);
            rightBackMotor.setPower(0.47500124000248);
            leftFrontMotor.setPower(0.76500124000248);
            rightFrontMotor.setPower(-0.76500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.131341483 && elapsedTime < 9.157282631) {
            leftBackMotor.setPower(-0.46375124000248);
            rightBackMotor.setPower(0.46375124000248);
            leftFrontMotor.setPower(0.77625124000248);
            rightFrontMotor.setPower(-0.77625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.157282631 && elapsedTime < 9.18418654) {
            leftBackMotor.setPower(-0.45625124000248);
            rightBackMotor.setPower(0.45625124000248);
            leftFrontMotor.setPower(0.78375124000248);
            rightFrontMotor.setPower(-0.78375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.18418654 && elapsedTime < 9.209979095) {
            leftBackMotor.setPower(-0.44875124000248);
            rightBackMotor.setPower(0.44875124000248);
            leftFrontMotor.setPower(0.79125124000248);
            rightFrontMotor.setPower(-0.79125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.209979095 && elapsedTime < 9.248529828) {
            leftBackMotor.setPower(-0.44500124000248);
            rightBackMotor.setPower(0.44500124000248);
            leftFrontMotor.setPower(0.7950012400024801);
            rightFrontMotor.setPower(-0.7950012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.248529828 && elapsedTime < 9.278943581) {
            leftBackMotor.setPower(-0.44625124000248);
            rightBackMotor.setPower(0.44625124000248);
            leftFrontMotor.setPower(0.79375124000248);
            rightFrontMotor.setPower(-0.79375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.278943581 && elapsedTime < 9.304733531) {
            leftBackMotor.setPower(-0.44750124000248004);
            rightBackMotor.setPower(0.44750124000248004);
            leftFrontMotor.setPower(0.79250124000248);
            rightFrontMotor.setPower(-0.79250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.304733531 && elapsedTime < 9.332484472) {
            leftBackMotor.setPower(-0.45625124000248);
            rightBackMotor.setPower(0.45625124000248);
            leftFrontMotor.setPower(0.78375124000248);
            rightFrontMotor.setPower(-0.78375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.332484472 && elapsedTime < 9.360750464) {
            leftBackMotor.setPower(-0.47000124000248);
            rightBackMotor.setPower(0.47000124000248);
            leftFrontMotor.setPower(0.77000124000248);
            rightFrontMotor.setPower(-0.77000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.360750464 && elapsedTime < 9.393840936) {
            leftBackMotor.setPower(-0.48375124000248004);
            rightBackMotor.setPower(0.48375124000248004);
            leftFrontMotor.setPower(0.75625124000248);
            rightFrontMotor.setPower(-0.75625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.393840936 && elapsedTime < 9.413956823) {
            leftBackMotor.setPower(-0.49875124000248006);
            rightBackMotor.setPower(0.49875124000248006);
            leftFrontMotor.setPower(0.74125124000248);
            rightFrontMotor.setPower(-0.74125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.413956823 && elapsedTime < 9.43485344) {
            leftBackMotor.setPower(-0.51125124000248);
            rightBackMotor.setPower(0.51125124000248);
            leftFrontMotor.setPower(0.72875124000248);
            rightFrontMotor.setPower(-0.72875124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.43485344 && elapsedTime < 9.457857974) {
            leftBackMotor.setPower(-0.52250124000248);
            rightBackMotor.setPower(0.52250124000248);
            leftFrontMotor.setPower(0.7175012400024801);
            rightFrontMotor.setPower(-0.7175012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.457857974 && elapsedTime < 9.483867091) {
            leftBackMotor.setPower(-0.53250124000248);
            rightBackMotor.setPower(0.53250124000248);
            leftFrontMotor.setPower(0.70750124000248);
            rightFrontMotor.setPower(-0.70750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.483867091 && elapsedTime < 9.510403031) {
            leftBackMotor.setPower(-0.5500012400024801);
            rightBackMotor.setPower(0.5500012400024801);
            leftFrontMotor.setPower(0.69000124000248);
            rightFrontMotor.setPower(-0.69000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.510403031 && elapsedTime < 9.534996367) {
            leftBackMotor.setPower(-0.55875124000248);
            rightBackMotor.setPower(0.55875124000248);
            leftFrontMotor.setPower(0.68125124000248);
            rightFrontMotor.setPower(-0.68125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.534996367 && elapsedTime < 9.555971473) {
            leftBackMotor.setPower(-0.5762512400024801);
            rightBackMotor.setPower(0.5762512400024801);
            leftFrontMotor.setPower(0.66375124000248);
            rightFrontMotor.setPower(-0.66375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.555971473 && elapsedTime < 9.577147204) {
            leftBackMotor.setPower(-0.58250124000248);
            rightBackMotor.setPower(0.58250124000248);
            leftFrontMotor.setPower(0.65750124000248);
            rightFrontMotor.setPower(-0.65750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.577147204 && elapsedTime < 9.602966217) {
            leftBackMotor.setPower(-0.59125124000248);
            rightBackMotor.setPower(0.59125124000248);
            leftFrontMotor.setPower(0.6487512400024801);
            rightFrontMotor.setPower(-0.6487512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.602966217 && elapsedTime < 9.622980178) {
            leftBackMotor.setPower(-0.60375124000248);
            rightBackMotor.setPower(0.60375124000248);
            leftFrontMotor.setPower(0.63625124000248);
            rightFrontMotor.setPower(-0.63625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.622980178 && elapsedTime < 9.645126378) {
            leftBackMotor.setPower(-0.61125124000248);
            rightBackMotor.setPower(0.61125124000248);
            leftFrontMotor.setPower(0.6287512400024801);
            rightFrontMotor.setPower(-0.6287512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.645126378 && elapsedTime < 9.676413412) {
            leftBackMotor.setPower(-0.61875124000248);
            rightBackMotor.setPower(0.61875124000248);
            leftFrontMotor.setPower(0.62125124000248);
            rightFrontMotor.setPower(-0.62125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.676413412 && elapsedTime < 9.703368415) {
            leftBackMotor.setPower(-0.62625124000248);
            rightBackMotor.setPower(0.62625124000248);
            leftFrontMotor.setPower(0.61375124000248);
            rightFrontMotor.setPower(-0.61375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.703368415 && elapsedTime < 9.731321751) {
            leftBackMotor.setPower(-0.6337512400024801);
            rightBackMotor.setPower(0.6337512400024801);
            leftFrontMotor.setPower(0.60625124000248);
            rightFrontMotor.setPower(-0.60625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.731321751 && elapsedTime < 9.782883214) {
            leftBackMotor.setPower(-0.64000124000248);
            rightBackMotor.setPower(0.64000124000248);
            leftFrontMotor.setPower(0.60000124000248);
            rightFrontMotor.setPower(-0.60000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.782883214 && elapsedTime < 9.811768425) {
            leftBackMotor.setPower(-0.64750124000248);
            rightBackMotor.setPower(0.64750124000248);
            leftFrontMotor.setPower(0.5925012400024801);
            rightFrontMotor.setPower(-0.5925012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.811768425 && elapsedTime < 9.845504991) {
            leftBackMotor.setPower(-0.64625124000248);
            rightBackMotor.setPower(0.64625124000248);
            leftFrontMotor.setPower(0.59375124000248);
            rightFrontMotor.setPower(-0.59375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.845504991 && elapsedTime < 9.881653537) {
            leftBackMotor.setPower(-0.65125124000248);
            rightBackMotor.setPower(0.65125124000248);
            leftFrontMotor.setPower(0.58875124000248);
            rightFrontMotor.setPower(-0.58875124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.881653537 && elapsedTime < 9.919924374) {
            leftBackMotor.setPower(-0.65000124000248);
            rightBackMotor.setPower(0.65000124000248);
            leftFrontMotor.setPower(0.59000124000248);
            rightFrontMotor.setPower(-0.59000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.919924374 && elapsedTime < 9.945991043) {
            leftBackMotor.setPower(-0.6487512400024801);
            rightBackMotor.setPower(0.6487512400024801);
            leftFrontMotor.setPower(0.59125124000248);
            rightFrontMotor.setPower(-0.59125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.945991043 && elapsedTime < 9.970524535) {
            leftBackMotor.setPower(-0.64625124000248);
            rightBackMotor.setPower(0.64625124000248);
            leftFrontMotor.setPower(0.59375124000248);
            rightFrontMotor.setPower(-0.59375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.970524535 && elapsedTime < 9.993884225) {
            leftBackMotor.setPower(-0.64250124000248);
            rightBackMotor.setPower(0.64250124000248);
            leftFrontMotor.setPower(0.5975012400024801);
            rightFrontMotor.setPower(-0.5975012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 9.993884225 && elapsedTime < 10.018970165) {
            leftBackMotor.setPower(-0.64000124000248);
            rightBackMotor.setPower(0.64000124000248);
            leftFrontMotor.setPower(0.60000124000248);
            rightFrontMotor.setPower(-0.60000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.018970165 && elapsedTime < 10.044377563) {
            leftBackMotor.setPower(-0.63625124000248);
            rightBackMotor.setPower(0.63625124000248);
            leftFrontMotor.setPower(0.60375124000248);
            rightFrontMotor.setPower(-0.60375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.044377563 && elapsedTime < 10.072089545) {
            leftBackMotor.setPower(-0.6337512400024801);
            rightBackMotor.setPower(0.6337512400024801);
            leftFrontMotor.setPower(0.60625124000248);
            rightFrontMotor.setPower(-0.60625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.072089545 && elapsedTime < 10.099162881) {
            leftBackMotor.setPower(-0.63250124000248);
            rightBackMotor.setPower(0.63250124000248);
            leftFrontMotor.setPower(0.6075012400024801);
            rightFrontMotor.setPower(-0.6075012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.099162881 && elapsedTime < 10.125117155) {
            leftBackMotor.setPower(-0.3268744678213716);
            rightBackMotor.setPower(0.3268744678213716);
            leftFrontMotor.setPower(0.3093744678213716);
            rightFrontMotor.setPower(-0.3093744678213716);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.125117155 && elapsedTime < 10.145575073) {
            leftBackMotor.setPower(-0.3243744678213716);
            rightBackMotor.setPower(0.3243744678213716);
            leftFrontMotor.setPower(0.3118744678213716);
            rightFrontMotor.setPower(-0.3118744678213716);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.145575073 && elapsedTime < 10.165370336) {
            leftBackMotor.setPower(-0.005);
            rightBackMotor.setPower(0.005);
            leftFrontMotor.setPower(-0.005);
            rightFrontMotor.setPower(0.005);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.165370336 && elapsedTime < 10.218048778) {
            leftBackMotor.setPower(-0.00125);
            rightBackMotor.setPower(0.00125);
            leftFrontMotor.setPower(-0.00125);
            rightFrontMotor.setPower(0.00125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.218048778 && elapsedTime < 10.230808832) {
            leftBackMotor.setPower(0.005);
            rightBackMotor.setPower(-0.005);
            leftFrontMotor.setPower(0.005);
            rightFrontMotor.setPower(-0.005);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.230808832 && elapsedTime < 10.257042116) {
            leftBackMotor.setPower(0.0075);
            rightBackMotor.setPower(-0.0075);
            leftFrontMotor.setPower(0.0075);
            rightFrontMotor.setPower(-0.0075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.257042116 && elapsedTime < 10.27043894) {
            leftBackMotor.setPower(0.00875);
            rightBackMotor.setPower(-0.00875);
            leftFrontMotor.setPower(0.00875);
            rightFrontMotor.setPower(-0.00875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.27043894 && elapsedTime < 10.290203369) {
            leftBackMotor.setPower(0.01);
            rightBackMotor.setPower(-0.01);
            leftFrontMotor.setPower(0.01);
            rightFrontMotor.setPower(-0.01);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.290203369 && elapsedTime < 10.32073931) {
            leftBackMotor.setPower(-0.34249999999999997);
            rightBackMotor.setPower(0.34249999999999997);
            leftFrontMotor.setPower(-0.34249999999999997);
            rightFrontMotor.setPower(0.34249999999999997);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 10.32073931 && elapsedTime < 11.334635088) {
            leftBackMotor.setPower(-0.35);
            rightBackMotor.setPower(0.35);
            leftFrontMotor.setPower(-0.35);
            rightFrontMotor.setPower(0.35);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.334635088 && elapsedTime < 11.45079812) {
            leftBackMotor.setPower(-0.09999999999999998);
            rightBackMotor.setPower(0.6);
            leftFrontMotor.setPower(-0.09999999999999998);
            rightFrontMotor.setPower(0.6);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.45079812 && elapsedTime < 11.473204476) {
            leftBackMotor.setPower(0.25);
            rightBackMotor.setPower(0.25);
            leftFrontMotor.setPower(0.25);
            rightFrontMotor.setPower(0.25);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.473204476 && elapsedTime < 11.493701041) {
            leftBackMotor.setPower(0.29625);
            rightBackMotor.setPower(0.20375);
            leftFrontMotor.setPower(0.29625);
            rightFrontMotor.setPower(0.20375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.493701041 && elapsedTime < 11.513183699) {
            leftBackMotor.setPower(0.34375);
            rightBackMotor.setPower(0.15625);
            leftFrontMotor.setPower(0.34375);
            rightFrontMotor.setPower(0.15625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.513183699 && elapsedTime < 11.53264943) {
            leftBackMotor.setPower(0.38875000000000004);
            rightBackMotor.setPower(0.11124999999999999);
            leftFrontMotor.setPower(0.38875000000000004);
            rightFrontMotor.setPower(0.11124999999999999);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.53264943 && elapsedTime < 11.560506829) {
            leftBackMotor.setPower(0.4275);
            rightBackMotor.setPower(0.07250000000000001);
            leftFrontMotor.setPower(0.4275);
            rightFrontMotor.setPower(0.07250000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.560506829 && elapsedTime < 11.583710321) {
            leftBackMotor.setPower(0.47);
            rightBackMotor.setPower(0.03);
            leftFrontMotor.setPower(0.47);
            rightFrontMotor.setPower(0.03);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.583710321 && elapsedTime < 11.610068657) {
            leftBackMotor.setPower(0.48750000000000004);
            rightBackMotor.setPower(0.012499999999999983);
            leftFrontMotor.setPower(0.48750000000000004);
            rightFrontMotor.setPower(0.012499999999999983);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.610068657 && elapsedTime < 11.635027253) {
            leftBackMotor.setPower(0.4975);
            rightBackMotor.setPower(0.0025000000000000022);
            leftFrontMotor.setPower(0.4975);
            rightFrontMotor.setPower(0.0025000000000000022);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.635027253 && elapsedTime < 11.658764235) {
            leftBackMotor.setPower(0.49375);
            rightBackMotor.setPower(0.0062500000000000056);
            leftFrontMotor.setPower(0.49375);
            rightFrontMotor.setPower(0.0062500000000000056);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.658764235 && elapsedTime < 11.678572466) {
            leftBackMotor.setPower(0.485);
            rightBackMotor.setPower(0.014999999999999986);
            leftFrontMotor.setPower(0.485);
            rightFrontMotor.setPower(0.014999999999999986);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.678572466 && elapsedTime < 11.701324968) {
            leftBackMotor.setPower(0.47375);
            rightBackMotor.setPower(0.026249999999999996);
            leftFrontMotor.setPower(0.47375);
            rightFrontMotor.setPower(0.026249999999999996);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.701324968 && elapsedTime < 11.722589814) {
            leftBackMotor.setPower(0.46125);
            rightBackMotor.setPower(0.03875000000000001);
            leftFrontMotor.setPower(0.46125);
            rightFrontMotor.setPower(0.03875000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.722589814 && elapsedTime < 11.742368409) {
            leftBackMotor.setPower(0.4425);
            rightBackMotor.setPower(0.057499999999999996);
            leftFrontMotor.setPower(0.4425);
            rightFrontMotor.setPower(0.057499999999999996);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.742368409 && elapsedTime < 11.762513151) {
            leftBackMotor.setPower(0.42500000000000004);
            rightBackMotor.setPower(0.07499999999999998);
            leftFrontMotor.setPower(0.42500000000000004);
            rightFrontMotor.setPower(0.07499999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.762513151 && elapsedTime < 11.793839873) {
            leftBackMotor.setPower(0.40375);
            rightBackMotor.setPower(0.09625);
            leftFrontMotor.setPower(0.40375);
            rightFrontMotor.setPower(0.09625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.793839873 && elapsedTime < 11.835103679) {
            leftBackMotor.setPower(0.375);
            rightBackMotor.setPower(0.125);
            leftFrontMotor.setPower(0.375);
            rightFrontMotor.setPower(0.125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.835103679 && elapsedTime < 11.889865403) {
            leftBackMotor.setPower(0.31875);
            rightBackMotor.setPower(0.18125);
            leftFrontMotor.setPower(0.31875);
            rightFrontMotor.setPower(0.18125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.889865403 && elapsedTime < 11.920380042) {
            leftBackMotor.setPower(0.26375);
            rightBackMotor.setPower(0.23625);
            leftFrontMotor.setPower(0.26375);
            rightFrontMotor.setPower(0.23625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.920380042 && elapsedTime < 11.946470982) {
            leftBackMotor.setPower(0.235);
            rightBackMotor.setPower(0.265);
            leftFrontMotor.setPower(0.235);
            rightFrontMotor.setPower(0.265);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.946470982 && elapsedTime < 11.966842651) {
            leftBackMotor.setPower(0.21375);
            rightBackMotor.setPower(0.28625);
            leftFrontMotor.setPower(0.21375);
            rightFrontMotor.setPower(0.28625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.966842651 && elapsedTime < 11.986571038) {
            leftBackMotor.setPower(0.19625);
            rightBackMotor.setPower(0.30375);
            leftFrontMotor.setPower(0.19625);
            rightFrontMotor.setPower(0.30375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 11.986571038 && elapsedTime < 12.007575519) {
            leftBackMotor.setPower(0.1875);
            rightBackMotor.setPower(0.3125);
            leftFrontMotor.setPower(0.1875);
            rightFrontMotor.setPower(0.3125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 12.007575519 && elapsedTime < 12.032156095) {
            leftBackMotor.setPower(0.17625000000000002);
            rightBackMotor.setPower(0.32375);
            leftFrontMotor.setPower(0.17625000000000002);
            rightFrontMotor.setPower(0.32375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 12.032156095 && elapsedTime < 12.060185264) {
            leftBackMotor.setPower(0.1725);
            rightBackMotor.setPower(0.3275);
            leftFrontMotor.setPower(0.1725);
            rightFrontMotor.setPower(0.3275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 12.060185264 && elapsedTime < 12.086173548) {
            leftBackMotor.setPower(0.16625);
            rightBackMotor.setPower(0.33375);
            leftFrontMotor.setPower(0.16625);
            rightFrontMotor.setPower(0.33375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 12.086173548 && elapsedTime < 12.122309437) {
            leftBackMotor.setPower(0.16875);
            rightBackMotor.setPower(0.33125);
            leftFrontMotor.setPower(0.16875);
            rightFrontMotor.setPower(0.33125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 12.122309437 && elapsedTime < 12.146598241) {
            leftBackMotor.setPower(0.16999999999999998);
            rightBackMotor.setPower(0.33);
            leftFrontMotor.setPower(0.16999999999999998);
            rightFrontMotor.setPower(0.33);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 12.146598241 && elapsedTime < 12.368316909) {
            leftBackMotor.setPower(0.6);
            rightBackMotor.setPower(-0.09999999999999998);
            leftFrontMotor.setPower(0.6);
            rightFrontMotor.setPower(-0.09999999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 12.368316909 && elapsedTime < 13.565371143) {
            leftBackMotor.setPower(0.35);
            rightBackMotor.setPower(-0.35);
            leftFrontMotor.setPower(0.35);
            rightFrontMotor.setPower(-0.35);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 13.565371143 && elapsedTime < 14.25456392) {
            leftBackMotor.setPower(0.6);
            rightBackMotor.setPower(-0.09999999999999998);
            leftFrontMotor.setPower(0.6);
            rightFrontMotor.setPower(-0.09999999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.25456392 && elapsedTime < 14.273829287) {
            leftBackMotor.setPower(0.23125);
            rightBackMotor.setPower(0.26875);
            leftFrontMotor.setPower(0.23125);
            rightFrontMotor.setPower(0.26875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.273829287 && elapsedTime < 14.294732518) {
            leftBackMotor.setPower(0.1925);
            rightBackMotor.setPower(0.3075);
            leftFrontMotor.setPower(0.1925);
            rightFrontMotor.setPower(0.3075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.294732518 && elapsedTime < 14.316099864) {
            leftBackMotor.setPower(0.155);
            rightBackMotor.setPower(0.345);
            leftFrontMotor.setPower(0.155);
            rightFrontMotor.setPower(0.345);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.316099864 && elapsedTime < 14.336443668) {
            leftBackMotor.setPower(0.11875);
            rightBackMotor.setPower(0.38125);
            leftFrontMotor.setPower(0.11875);
            rightFrontMotor.setPower(0.38125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.336443668 && elapsedTime < 14.359416274) {
            leftBackMotor.setPower(0.08875);
            rightBackMotor.setPower(0.41125);
            leftFrontMotor.setPower(0.08875);
            rightFrontMotor.setPower(0.41125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.359416274 && elapsedTime < 14.387513256) {
            leftBackMotor.setPower(0.065);
            rightBackMotor.setPower(0.435);
            leftFrontMotor.setPower(0.065);
            rightFrontMotor.setPower(0.435);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.387513256 && elapsedTime < 14.408494092) {
            leftBackMotor.setPower(0.04125000000000001);
            rightBackMotor.setPower(0.45875);
            leftFrontMotor.setPower(0.04125000000000001);
            rightFrontMotor.setPower(0.45875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.408494092 && elapsedTime < 14.431476438) {
            leftBackMotor.setPower(0.03375);
            rightBackMotor.setPower(0.46625);
            leftFrontMotor.setPower(0.03375);
            rightFrontMotor.setPower(0.46625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.431476438 && elapsedTime < 14.475128994) {
            leftBackMotor.setPower(0.026249999999999996);
            rightBackMotor.setPower(0.47375);
            leftFrontMotor.setPower(0.026249999999999996);
            rightFrontMotor.setPower(0.47375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.475128994 && elapsedTime < 14.505001185) {
            leftBackMotor.setPower(0.03375);
            rightBackMotor.setPower(0.46625);
            leftFrontMotor.setPower(0.03375);
            rightFrontMotor.setPower(0.46625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.505001185 && elapsedTime < 14.603831299) {
            leftBackMotor.setPower(0.6);
            rightBackMotor.setPower(-0.09999999999999998);
            leftFrontMotor.setPower(0.6);
            rightFrontMotor.setPower(-0.09999999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.603831299 && elapsedTime < 14.806240173) {
            leftBackMotor.setPower(0.35);
            rightBackMotor.setPower(-0.35);
            leftFrontMotor.setPower(0.35);
            rightFrontMotor.setPower(-0.35);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.806240173 && elapsedTime < 14.830852155) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.830852155 && elapsedTime < 14.840531166) {
            leftBackMotor.setPower(-0.0725);
            rightBackMotor.setPower(0.0725);
            leftFrontMotor.setPower(-0.0725);
            rightFrontMotor.setPower(0.0725);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.840531166 && elapsedTime < 14.850793042) {
            leftBackMotor.setPower(-0.0975);
            rightBackMotor.setPower(0.0975);
            leftFrontMotor.setPower(-0.0975);
            rightFrontMotor.setPower(0.0975);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.850793042 && elapsedTime < 14.873163982) {
            leftBackMotor.setPower(-0.12125);
            rightBackMotor.setPower(0.12125);
            leftFrontMotor.setPower(-0.12125);
            rightFrontMotor.setPower(0.12125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.873163982 && elapsedTime < 14.895041067) {
            leftBackMotor.setPower(-0.15875);
            rightBackMotor.setPower(0.15875);
            leftFrontMotor.setPower(-0.15875);
            rightFrontMotor.setPower(0.15875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.895041067 && elapsedTime < 14.904931485) {
            leftBackMotor.setPower(-0.185);
            rightBackMotor.setPower(0.185);
            leftFrontMotor.setPower(-0.185);
            rightFrontMotor.setPower(0.185);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.904931485 && elapsedTime < 14.920112528) {
            leftBackMotor.setPower(-0.19125);
            rightBackMotor.setPower(0.19125);
            leftFrontMotor.setPower(-0.19125);
            rightFrontMotor.setPower(0.19125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.920112528 && elapsedTime < 14.929180133) {
            leftBackMotor.setPower(-0.19375);
            rightBackMotor.setPower(0.19375);
            leftFrontMotor.setPower(-0.19375);
            rightFrontMotor.setPower(0.19375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.929180133 && elapsedTime < 14.955622844) {
            leftBackMotor.setPower(0.15499999999999997);
            rightBackMotor.setPower(-0.15499999999999997);
            leftFrontMotor.setPower(0.15499999999999997);
            rightFrontMotor.setPower(-0.15499999999999997);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 14.955622844 && elapsedTime < 15.810707513) {
            leftBackMotor.setPower(0.35);
            rightBackMotor.setPower(-0.35);
            leftFrontMotor.setPower(0.35);
            rightFrontMotor.setPower(-0.35);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.810707513 && elapsedTime < 15.8347809) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.8347809 && elapsedTime < 15.847702516) {
            leftBackMotor.setPower(-0.07125000000000001);
            rightBackMotor.setPower(0.07125000000000001);
            leftFrontMotor.setPower(-0.07125000000000001);
            rightFrontMotor.setPower(0.07125000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.847702516 && elapsedTime < 15.859831528) {
            leftBackMotor.setPower(-0.0925);
            rightBackMotor.setPower(0.0925);
            leftFrontMotor.setPower(-0.0925);
            rightFrontMotor.setPower(0.0925);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.859831528 && elapsedTime < 15.868824498) {
            leftBackMotor.setPower(-0.115);
            rightBackMotor.setPower(0.115);
            leftFrontMotor.setPower(-0.115);
            rightFrontMotor.setPower(0.115);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.868824498 && elapsedTime < 15.877854551) {
            leftBackMotor.setPower(-0.1375);
            rightBackMotor.setPower(0.1375);
            leftFrontMotor.setPower(-0.1375);
            rightFrontMotor.setPower(0.1375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.877854551 && elapsedTime < 15.894181531) {
            leftBackMotor.setPower(-0.155);
            rightBackMotor.setPower(0.155);
            leftFrontMotor.setPower(-0.155);
            rightFrontMotor.setPower(0.155);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.894181531 && elapsedTime < 15.921354451) {
            leftBackMotor.setPower(-0.4225);
            rightBackMotor.setPower(-0.07749999999999999);
            leftFrontMotor.setPower(-0.4225);
            rightFrontMotor.setPower(-0.07749999999999999);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.921354451 && elapsedTime < 15.942673151) {
            leftBackMotor.setPower(-0.43);
            rightBackMotor.setPower(-0.07);
            leftFrontMotor.setPower(-0.43);
            rightFrontMotor.setPower(-0.07);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.942673151 && elapsedTime < 15.965614039) {
            leftBackMotor.setPower(-0.42625);
            rightBackMotor.setPower(-0.07375000000000001);
            leftFrontMotor.setPower(-0.42625);
            rightFrontMotor.setPower(-0.07375000000000001);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.965614039 && elapsedTime < 15.999162167) {
            leftBackMotor.setPower(-0.42000000000000004);
            rightBackMotor.setPower(-0.07999999999999999);
            leftFrontMotor.setPower(-0.42000000000000004);
            rightFrontMotor.setPower(-0.07999999999999999);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 15.999162167 && elapsedTime < 16.038240348) {
            leftBackMotor.setPower(-0.405);
            rightBackMotor.setPower(-0.095);
            leftFrontMotor.setPower(-0.405);
            rightFrontMotor.setPower(-0.095);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.038240348 && elapsedTime < 16.066332382) {
            leftBackMotor.setPower(-0.385);
            rightBackMotor.setPower(-0.11499999999999999);
            leftFrontMotor.setPower(-0.385);
            rightFrontMotor.setPower(-0.11499999999999999);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.066332382 && elapsedTime < 16.086297696) {
            leftBackMotor.setPower(-0.36375);
            rightBackMotor.setPower(-0.13624999999999998);
            leftFrontMotor.setPower(-0.36375);
            rightFrontMotor.setPower(-0.13624999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.086297696 && elapsedTime < 16.107620042) {
            leftBackMotor.setPower(-0.34875);
            rightBackMotor.setPower(-0.15125);
            leftFrontMotor.setPower(-0.34875);
            rightFrontMotor.setPower(-0.15125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.107620042 && elapsedTime < 16.129614003) {
            leftBackMotor.setPower(-0.3325);
            rightBackMotor.setPower(-0.16749999999999998);
            leftFrontMotor.setPower(-0.3325);
            rightFrontMotor.setPower(-0.16749999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.129614003 && elapsedTime < 16.154733224) {
            leftBackMotor.setPower(-0.3175);
            rightBackMotor.setPower(-0.1825);
            leftFrontMotor.setPower(-0.3175);
            rightFrontMotor.setPower(-0.1825);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.154733224 && elapsedTime < 16.175885101) {
            leftBackMotor.setPower(-0.2975);
            rightBackMotor.setPower(-0.2025);
            leftFrontMotor.setPower(-0.2975);
            rightFrontMotor.setPower(-0.2025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.175885101 && elapsedTime < 16.198379166) {
            leftBackMotor.setPower(-0.28500000000000003);
            rightBackMotor.setPower(-0.215);
            leftFrontMotor.setPower(-0.28500000000000003);
            rightFrontMotor.setPower(-0.215);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.198379166 && elapsedTime < 16.222575366) {
            leftBackMotor.setPower(-0.27375);
            rightBackMotor.setPower(-0.22625);
            leftFrontMotor.setPower(-0.27375);
            rightFrontMotor.setPower(-0.22625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.222575366 && elapsedTime < 16.48561258) {
            leftBackMotor.setPower(0.09999999999999998);
            rightBackMotor.setPower(-0.6);
            leftFrontMotor.setPower(0.09999999999999998);
            rightFrontMotor.setPower(-0.6);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.48561258 && elapsedTime < 16.51969425) {
            leftBackMotor.setPower(-0.25);
            rightBackMotor.setPower(-0.25);
            leftFrontMotor.setPower(-0.25);
            rightFrontMotor.setPower(-0.25);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.51969425 && elapsedTime < 16.55451717) {
            leftBackMotor.setPower(-0.30375);
            rightBackMotor.setPower(-0.19625);
            leftFrontMotor.setPower(-0.30375);
            rightFrontMotor.setPower(-0.19625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.55451717 && elapsedTime < 16.579585089) {
            leftBackMotor.setPower(-0.36875);
            rightBackMotor.setPower(-0.13124999999999998);
            leftFrontMotor.setPower(-0.36875);
            rightFrontMotor.setPower(-0.13124999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.579585089 && elapsedTime < 16.606419832) {
            leftBackMotor.setPower(-0.39125);
            rightBackMotor.setPower(-0.10874999999999999);
            leftFrontMotor.setPower(-0.39125);
            rightFrontMotor.setPower(-0.10874999999999999);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.606419832 && elapsedTime < 16.634512803) {
            leftBackMotor.setPower(-0.41125);
            rightBackMotor.setPower(-0.08875);
            leftFrontMotor.setPower(-0.41125);
            rightFrontMotor.setPower(-0.08875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.634512803 && elapsedTime < 16.665663171) {
            leftBackMotor.setPower(-0.42375);
            rightBackMotor.setPower(-0.07624999999999998);
            leftFrontMotor.setPower(-0.42375);
            rightFrontMotor.setPower(-0.07624999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.665663171 && elapsedTime < 16.691475882) {
            leftBackMotor.setPower(-0.42500000000000004);
            rightBackMotor.setPower(-0.07499999999999998);
            leftFrontMotor.setPower(-0.42500000000000004);
            rightFrontMotor.setPower(-0.07499999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.691475882 && elapsedTime < 16.715113905) {
            leftBackMotor.setPower(-0.42125);
            rightBackMotor.setPower(-0.07874999999999999);
            leftFrontMotor.setPower(-0.42125);
            rightFrontMotor.setPower(-0.07874999999999999);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.715113905 && elapsedTime < 16.734770886) {
            leftBackMotor.setPower(-0.4125);
            rightBackMotor.setPower(-0.0875);
            leftFrontMotor.setPower(-0.4125);
            rightFrontMotor.setPower(-0.0875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.734770886 && elapsedTime < 16.756598961) {
            leftBackMotor.setPower(-0.40125);
            rightBackMotor.setPower(-0.09875);
            leftFrontMotor.setPower(-0.40125);
            rightFrontMotor.setPower(-0.09875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.756598961 && elapsedTime < 16.779258651) {
            leftBackMotor.setPower(-0.39375000000000004);
            rightBackMotor.setPower(-0.10624999999999998);
            leftFrontMotor.setPower(-0.39375000000000004);
            rightFrontMotor.setPower(-0.10624999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.779258651 && elapsedTime < 16.808623237) {
            leftBackMotor.setPower(-0.38);
            rightBackMotor.setPower(-0.12);
            leftFrontMotor.setPower(-0.38);
            rightFrontMotor.setPower(-0.12);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.808623237 && elapsedTime < 16.838220896) {
            leftBackMotor.setPower(-0.36375);
            rightBackMotor.setPower(-0.13624999999999998);
            leftFrontMotor.setPower(-0.36375);
            rightFrontMotor.setPower(-0.13624999999999998);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.838220896 && elapsedTime < 16.863774649) {
            leftBackMotor.setPower(-0.3425);
            rightBackMotor.setPower(-0.1575);
            leftFrontMotor.setPower(-0.3425);
            rightFrontMotor.setPower(-0.1575);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.863774649 && elapsedTime < 16.888699495) {
            leftBackMotor.setPower(-0.32);
            rightBackMotor.setPower(-0.18);
            leftFrontMotor.setPower(-0.32);
            rightFrontMotor.setPower(-0.18);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.888699495 && elapsedTime < 16.920390175) {
            leftBackMotor.setPower(-0.06);
            rightBackMotor.setPower(0.06);
            leftFrontMotor.setPower(-0.06);
            rightFrontMotor.setPower(0.06);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.920390175 && elapsedTime < 16.93780575) {
            leftBackMotor.setPower(-0.0375);
            rightBackMotor.setPower(0.0375);
            leftFrontMotor.setPower(-0.0375);
            rightFrontMotor.setPower(0.0375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.93780575 && elapsedTime < 16.946574553) {
            leftBackMotor.setPower(-0.02875);
            rightBackMotor.setPower(0.02875);
            leftFrontMotor.setPower(-0.02875);
            rightFrontMotor.setPower(0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.946574553 && elapsedTime < 16.955654971) {
            leftBackMotor.setPower(-0.02375);
            rightBackMotor.setPower(0.02375);
            leftFrontMotor.setPower(-0.02375);
            rightFrontMotor.setPower(0.02375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.955654971 && elapsedTime < 16.973600285) {
            leftBackMotor.setPower(-0.02);
            rightBackMotor.setPower(0.02);
            leftFrontMotor.setPower(-0.02);
            rightFrontMotor.setPower(0.02);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.973600285 && elapsedTime < 16.982676848) {
            leftBackMotor.setPower(-0.01625);
            rightBackMotor.setPower(0.01625);
            leftFrontMotor.setPower(-0.01625);
            rightFrontMotor.setPower(0.01625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.982676848 && elapsedTime < 16.997759975) {
            leftBackMotor.setPower(-0.01375);
            rightBackMotor.setPower(0.01375);
            leftFrontMotor.setPower(-0.01375);
            rightFrontMotor.setPower(0.01375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 16.997759975 && elapsedTime < 17.006597788) {
            leftBackMotor.setPower(-0.00875);
            rightBackMotor.setPower(0.00875);
            leftFrontMotor.setPower(-0.00875);
            rightFrontMotor.setPower(0.00875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 17.006597788 && elapsedTime < 17.02007555) {
            leftBackMotor.setPower(-0.00625);
            rightBackMotor.setPower(0.00625);
            leftFrontMotor.setPower(-0.00625);
            rightFrontMotor.setPower(0.00625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 17.02007555 && elapsedTime < 17.036489979) {
            leftBackMotor.setPower(-0.00375);
            rightBackMotor.setPower(0.00375);
            leftFrontMotor.setPower(-0.00375);
            rightFrontMotor.setPower(0.00375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 17.036489979 && elapsedTime < 17.058007064) {
            leftBackMotor.setPower(0.00125);
            rightBackMotor.setPower(-0.00125);
            leftFrontMotor.setPower(0.00125);
            rightFrontMotor.setPower(-0.00125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 17.058007064 && elapsedTime < 17.067724617) {
            leftBackMotor.setPower(0.005);
            rightBackMotor.setPower(-0.005);
            leftFrontMotor.setPower(0.005);
            rightFrontMotor.setPower(-0.005);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 17.067724617 && elapsedTime < 17.088954775) {
            leftBackMotor.setPower(0.0075);
            rightBackMotor.setPower(-0.0075);
            leftFrontMotor.setPower(0.0075);
            rightFrontMotor.setPower(-0.0075);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
        if(elapsedTime > 17.088954775 && elapsedTime < 17.158796605) {
            leftBackMotor.setPower(0.00875);
            rightBackMotor.setPower(-0.00875);
            leftFrontMotor.setPower(0.00875);
            rightFrontMotor.setPower(-0.00875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
        }
    }
}
