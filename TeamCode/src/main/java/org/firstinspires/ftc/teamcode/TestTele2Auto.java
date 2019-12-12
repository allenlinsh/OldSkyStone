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
    private Servo skystoneServo;
    private ColorSensor colorSensor;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.2, correction;
    double elapsedTime, endTime;
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
        skystoneServo           = hardwareMap.get(Servo.class, "skystoneServo");
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

        // initialize servos
        hookOff();
        skystoneOff();

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
            // place stone on the foundation
            // re-position the foundation
            // grab another stone
            // place stone on the foundation
            // park under the alliance bridge
            // stop
            //
            // **************************************************

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
    public void hookOn() {
        leftServo.setPosition(1);
        rightServo.setPosition(0);
    }
    public void hookOff() {
        leftServo.setPosition(0.1);
        rightServo.setPosition(0.9);
    }
    public void skystoneOn() {
        skystoneServo.setPosition(0.98);
    }
    public void skystoneOff() {
        skystoneServo.setPosition(0.52);
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
        if (elapsedTime > 0 && elapsedTime < 0.093763603) {
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            gripMotor.setPower(0);
            armMotor.setPower(0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.093763603 && elapsedTime < 0.121533554) {
            leftBackMotor.setPower(0.7502015004030008);
            rightBackMotor.setPower(0.7502015004030008);
            leftFrontMotor.setPower(0.7502015004030008);
            rightFrontMotor.setPower(0.7502015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.121533554 && elapsedTime < 0.148165744) {
            leftBackMotor.setPower(0.7477015004030009);
            rightBackMotor.setPower(0.7527015004030008);
            leftFrontMotor.setPower(0.7477015004030009);
            rightFrontMotor.setPower(0.7527015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.148165744 && elapsedTime < 0.176449236) {
            leftBackMotor.setPower(0.7452015004030008);
            rightBackMotor.setPower(0.7552015004030008);
            leftFrontMotor.setPower(0.7452015004030008);
            rightFrontMotor.setPower(0.7552015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.176449236 && elapsedTime < 0.202204499) {
            leftBackMotor.setPower(0.7427015004030009);
            rightBackMotor.setPower(0.7577015004030008);
            leftFrontMotor.setPower(0.7427015004030009);
            rightFrontMotor.setPower(0.7577015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.202204499 && elapsedTime < 0.229450335) {
            leftBackMotor.setPower(0.7364515004030008);
            rightBackMotor.setPower(0.7639515004030009);
            leftFrontMotor.setPower(0.7364515004030008);
            rightFrontMotor.setPower(0.7639515004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.229450335 && elapsedTime < 0.255154973) {
            leftBackMotor.setPower(0.7327015004030009);
            rightBackMotor.setPower(0.7677015004030008);
            leftFrontMotor.setPower(0.7327015004030009);
            rightFrontMotor.setPower(0.7677015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.255154973 && elapsedTime < 0.282315236) {
            leftBackMotor.setPower(0.7264515004030008);
            rightBackMotor.setPower(0.7739515004030009);
            leftFrontMotor.setPower(0.7264515004030008);
            rightFrontMotor.setPower(0.7739515004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.282315236 && elapsedTime < 0.309192895) {
            leftBackMotor.setPower(0.7189515004030008);
            rightBackMotor.setPower(0.7814515004030008);
            leftFrontMotor.setPower(0.7189515004030008);
            rightFrontMotor.setPower(0.7814515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.309192895 && elapsedTime < 0.335229773) {
            leftBackMotor.setPower(0.3209981307231061);
            rightBackMotor.setPower(0.3959981307231061);
            leftFrontMotor.setPower(0.3209981307231061);
            rightFrontMotor.setPower(0.3959981307231061);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.335229773 && elapsedTime < 0.36232113) {
            leftBackMotor.setPower(0.3134981307231061);
            rightBackMotor.setPower(0.4034981307231061);
            leftFrontMotor.setPower(0.3134981307231061);
            rightFrontMotor.setPower(0.4034981307231061);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.36232113 && elapsedTime < 0.391951966) {
            leftBackMotor.setPower(-0.0525);
            rightBackMotor.setPower(0.0525);
            leftFrontMotor.setPower(-0.0525);
            rightFrontMotor.setPower(0.0525);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.391951966 && elapsedTime < 0.405094832) {
            leftBackMotor.setPower(-0.06);
            rightBackMotor.setPower(0.06);
            leftFrontMotor.setPower(-0.06);
            rightFrontMotor.setPower(0.06);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.405094832 && elapsedTime < 0.419281604) {
            leftBackMotor.setPower(-0.058750000000000004);
            rightBackMotor.setPower(0.058750000000000004);
            leftFrontMotor.setPower(-0.058750000000000004);
            rightFrontMotor.setPower(0.058750000000000004);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.419281604 && elapsedTime < 0.431923064) {
            leftBackMotor.setPower(-0.06);
            rightBackMotor.setPower(0.06);
            leftFrontMotor.setPower(-0.06);
            rightFrontMotor.setPower(0.06);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.431923064 && elapsedTime < 0.444116763) {
            leftBackMotor.setPower(-0.0575);
            rightBackMotor.setPower(0.0575);
            leftFrontMotor.setPower(-0.0575);
            rightFrontMotor.setPower(0.0575);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.444116763 && elapsedTime < 0.457112597) {
            leftBackMotor.setPower(-0.0525);
            rightBackMotor.setPower(0.0525);
            leftFrontMotor.setPower(-0.0525);
            rightFrontMotor.setPower(0.0525);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.457112597 && elapsedTime < 0.470198797) {
            leftBackMotor.setPower(-0.05375);
            rightBackMotor.setPower(0.05375);
            leftFrontMotor.setPower(-0.05375);
            rightFrontMotor.setPower(0.05375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.470198797 && elapsedTime < 0.484263121) {
            leftBackMotor.setPower(-0.0525);
            rightBackMotor.setPower(0.0525);
            leftFrontMotor.setPower(-0.0525);
            rightFrontMotor.setPower(0.0525);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.484263121 && elapsedTime < 0.497094945) {
            leftBackMotor.setPower(-0.05);
            rightBackMotor.setPower(0.05);
            leftFrontMotor.setPower(-0.05);
            rightFrontMotor.setPower(0.05);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.497094945 && elapsedTime < 0.51212552) {
            leftBackMotor.setPower(-0.04875);
            rightBackMotor.setPower(0.04875);
            leftFrontMotor.setPower(-0.04875);
            rightFrontMotor.setPower(0.04875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.51212552 && elapsedTime < 0.529832761) {
            leftBackMotor.setPower(-0.045);
            rightBackMotor.setPower(0.045);
            leftFrontMotor.setPower(-0.045);
            rightFrontMotor.setPower(0.045);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.529832761 && elapsedTime < 0.552655888) {
            leftBackMotor.setPower(-0.0425);
            rightBackMotor.setPower(0.0425);
            leftFrontMotor.setPower(-0.0425);
            rightFrontMotor.setPower(0.0425);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.552655888 && elapsedTime < 0.616940009) {
            leftBackMotor.setPower(-0.11882862380946818);
            rightBackMotor.setPower(-0.03882862380946817);
            leftFrontMotor.setPower(-0.11882862380946818);
            rightFrontMotor.setPower(-0.03882862380946817);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.616940009 && elapsedTime < 0.64415897) {
            leftBackMotor.setPower(-0.5415848476400692);
            rightBackMotor.setPower(-0.4790848476400692);
            leftFrontMotor.setPower(-0.5415848476400692);
            rightFrontMotor.setPower(-0.4790848476400692);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.64415897 && elapsedTime < 0.671265015) {
            leftBackMotor.setPower(-0.7639847312929418);
            rightBackMotor.setPower(-0.7114847312929418);
            leftFrontMotor.setPower(-0.7639847312929418);
            rightFrontMotor.setPower(-0.7114847312929418);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.671265015 && elapsedTime < 0.698369913) {
            leftBackMotor.setPower(-0.7714515004030008);
            rightBackMotor.setPower(-0.7289515004030008);
            leftFrontMotor.setPower(-0.7714515004030008);
            rightFrontMotor.setPower(-0.7289515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.698369913 && elapsedTime < 0.733027104) {
            leftBackMotor.setPower(-0.7664515004030008);
            rightBackMotor.setPower(-0.7339515004030008);
            leftFrontMotor.setPower(-0.7664515004030008);
            rightFrontMotor.setPower(-0.7339515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.733027104 && elapsedTime < 0.761804607) {
            leftBackMotor.setPower(-0.7602015004030008);
            rightBackMotor.setPower(-0.7402015004030008);
            leftFrontMotor.setPower(-0.7602015004030008);
            rightFrontMotor.setPower(-0.7402015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.761804607 && elapsedTime < 0.788577266) {
            leftBackMotor.setPower(-0.7514515004030008);
            rightBackMotor.setPower(-0.7489515004030008);
            leftFrontMotor.setPower(-0.7514515004030008);
            rightFrontMotor.setPower(-0.7489515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.788577266 && elapsedTime < 0.815964977) {
            leftBackMotor.setPower(-0.7552015004030008);
            rightBackMotor.setPower(-0.7452015004030008);
            leftFrontMotor.setPower(-0.7552015004030008);
            rightFrontMotor.setPower(-0.7452015004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.815964977 && elapsedTime < 0.843031126) {
            leftBackMotor.setPower(-0.7527015004030008);
            rightBackMotor.setPower(-0.7477015004030009);
            leftFrontMotor.setPower(-0.7527015004030008);
            rightFrontMotor.setPower(-0.7477015004030009);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.843031126 && elapsedTime < 0.869769305) {
            leftBackMotor.setPower(-0.7439515004030008);
            rightBackMotor.setPower(-0.7564515004030008);
            leftFrontMotor.setPower(-0.7439515004030008);
            rightFrontMotor.setPower(-0.7564515004030008);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 0.869769305 && elapsedTime < 0.910498216) {
            leftBackMotor.setPower(-0.3547481307231061);
            rightBackMotor.setPower(-0.3622481307231061);
            leftFrontMotor.setPower(-0.3547481307231061);
            rightFrontMotor.setPower(-0.3622481307231061);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.910498216 && elapsedTime < 0.938525718) {
            leftBackMotor.setPower(0.01);
            rightBackMotor.setPower(-0.01);
            leftFrontMotor.setPower(0.01);
            rightFrontMotor.setPower(-0.01);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.938525718 && elapsedTime < 0.966744315) {
            leftBackMotor.setPower(0.00375);
            rightBackMotor.setPower(-0.00375);
            leftFrontMotor.setPower(0.00375);
            rightFrontMotor.setPower(-0.00375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.966744315 && elapsedTime < 0.992865828) {
            leftBackMotor.setPower(0.00125);
            rightBackMotor.setPower(-0.00125);
            leftFrontMotor.setPower(0.00125);
            rightFrontMotor.setPower(-0.00125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 0.992865828 && elapsedTime < 1.006530152) {
            leftBackMotor.setPower(-0.00625);
            rightBackMotor.setPower(0.00625);
            leftFrontMotor.setPower(-0.00625);
            rightFrontMotor.setPower(0.00625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.006530152 && elapsedTime < 1.021186091) {
            leftBackMotor.setPower(-0.01);
            rightBackMotor.setPower(0.01);
            leftFrontMotor.setPower(-0.01);
            rightFrontMotor.setPower(0.01);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.021186091 && elapsedTime < 1.040991718) {
            leftBackMotor.setPower(-0.02);
            rightBackMotor.setPower(0.02);
            leftFrontMotor.setPower(-0.02);
            rightFrontMotor.setPower(0.02);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.040991718 && elapsedTime < 1.059699116) {
            leftBackMotor.setPower(-0.025);
            rightBackMotor.setPower(0.025);
            leftFrontMotor.setPower(-0.025);
            rightFrontMotor.setPower(0.025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.059699116 && elapsedTime < 1.072476357) {
            leftBackMotor.setPower(-0.035);
            rightBackMotor.setPower(0.035);
            leftFrontMotor.setPower(-0.035);
            rightFrontMotor.setPower(0.035);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.072476357 && elapsedTime < 1.125556466) {
            leftBackMotor.setPower(-0.043750000000000004);
            rightBackMotor.setPower(0.043750000000000004);
            leftFrontMotor.setPower(-0.043750000000000004);
            rightFrontMotor.setPower(0.043750000000000004);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.125556466 && elapsedTime < 1.164756626) {
            leftBackMotor.setPower(-0.0675);
            rightBackMotor.setPower(0.0675);
            leftFrontMotor.setPower(-0.0675);
            rightFrontMotor.setPower(0.0675);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.164756626 && elapsedTime < 1.194407931) {
            leftBackMotor.setPower(-0.4614262951500551);
            rightBackMotor.setPower(0.4614262951500551);
            leftFrontMotor.setPower(0.29892629515005514);
            rightFrontMotor.setPower(-0.29892629515005514);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.194407931 && elapsedTime < 1.221841215) {
            leftBackMotor.setPower(-0.70375124000248);
            rightBackMotor.setPower(0.70375124000248);
            leftFrontMotor.setPower(0.53625124000248);
            rightFrontMotor.setPower(-0.53625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.221841215 && elapsedTime < 1.24790403) {
            leftBackMotor.setPower(-0.7012512400024801);
            rightBackMotor.setPower(0.7012512400024801);
            leftFrontMotor.setPower(0.53875124000248);
            rightFrontMotor.setPower(-0.53875124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.24790403 && elapsedTime < 1.274612731) {
            leftBackMotor.setPower(-0.68875124000248);
            rightBackMotor.setPower(0.68875124000248);
            leftFrontMotor.setPower(0.55125124000248);
            rightFrontMotor.setPower(-0.55125124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.274612731 && elapsedTime < 1.3009319) {
            leftBackMotor.setPower(-0.67625124000248);
            rightBackMotor.setPower(0.67625124000248);
            leftFrontMotor.setPower(0.56375124000248);
            rightFrontMotor.setPower(-0.56375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.3009319 && elapsedTime < 1.329018205) {
            leftBackMotor.setPower(-0.66250124000248);
            rightBackMotor.setPower(0.66250124000248);
            leftFrontMotor.setPower(0.57750124000248);
            rightFrontMotor.setPower(-0.57750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.329018205 && elapsedTime < 1.355998416) {
            leftBackMotor.setPower(-0.64500124000248);
            rightBackMotor.setPower(0.64500124000248);
            leftFrontMotor.setPower(0.59500124000248);
            rightFrontMotor.setPower(-0.59500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.355998416 && elapsedTime < 1.381867846) {
            leftBackMotor.setPower(-0.6175012400024801);
            rightBackMotor.setPower(0.6175012400024801);
            leftFrontMotor.setPower(0.62250124000248);
            rightFrontMotor.setPower(-0.62250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.381867846 && elapsedTime < 1.408151911) {
            leftBackMotor.setPower(-0.60000124000248);
            rightBackMotor.setPower(0.60000124000248);
            leftFrontMotor.setPower(0.64000124000248);
            rightFrontMotor.setPower(-0.64000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.408151911 && elapsedTime < 1.434700299) {
            leftBackMotor.setPower(-0.57250124000248);
            rightBackMotor.setPower(0.57250124000248);
            leftFrontMotor.setPower(0.66750124000248);
            rightFrontMotor.setPower(-0.66750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.434700299 && elapsedTime < 1.462449416) {
            leftBackMotor.setPower(-0.54375124000248);
            rightBackMotor.setPower(0.54375124000248);
            leftFrontMotor.setPower(0.6962512400024801);
            rightFrontMotor.setPower(-0.6962512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.462449416 && elapsedTime < 1.489091659) {
            leftBackMotor.setPower(-0.52625124000248);
            rightBackMotor.setPower(0.52625124000248);
            leftFrontMotor.setPower(0.71375124000248);
            rightFrontMotor.setPower(-0.71375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.489091659 && elapsedTime < 1.521136089) {
            leftBackMotor.setPower(-0.50000124000248);
            rightBackMotor.setPower(0.50000124000248);
            leftFrontMotor.setPower(0.74000124000248);
            rightFrontMotor.setPower(-0.74000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.521136089 && elapsedTime < 1.550000258) {
            leftBackMotor.setPower(-0.47375124000248003);
            rightBackMotor.setPower(0.47375124000248003);
            leftFrontMotor.setPower(0.76625124000248);
            rightFrontMotor.setPower(-0.76625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.550000258 && elapsedTime < 1.587995158) {
            leftBackMotor.setPower(-0.45250124000248004);
            rightBackMotor.setPower(0.45250124000248004);
            leftFrontMotor.setPower(0.78750124000248);
            rightFrontMotor.setPower(-0.78750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.587995158 && elapsedTime < 1.655025529) {
            leftBackMotor.setPower(-0.42875124000248);
            rightBackMotor.setPower(0.42875124000248);
            leftFrontMotor.setPower(0.8112512400024801);
            rightFrontMotor.setPower(-0.8112512400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.655025529 && elapsedTime < 1.69800095) {
            leftBackMotor.setPower(-0.40375124000248);
            rightBackMotor.setPower(0.40375124000248);
            leftFrontMotor.setPower(0.83625124000248);
            rightFrontMotor.setPower(-0.83625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.69800095 && elapsedTime < 1.725051265) {
            leftBackMotor.setPower(0.2175);
            rightBackMotor.setPower(-0.2175);
            leftFrontMotor.setPower(0.2175);
            rightFrontMotor.setPower(-0.2175);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.725051265 && elapsedTime < 1.752087362) {
            leftBackMotor.setPower(0.21625);
            rightBackMotor.setPower(-0.21625);
            leftFrontMotor.setPower(0.21625);
            rightFrontMotor.setPower(-0.21625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.752087362 && elapsedTime < 1.764877572) {
            leftBackMotor.setPower(0.21);
            rightBackMotor.setPower(-0.21);
            leftFrontMotor.setPower(0.21);
            rightFrontMotor.setPower(-0.21);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.764877572 && elapsedTime < 1.778060906) {
            leftBackMotor.setPower(0.19625);
            rightBackMotor.setPower(-0.19625);
            leftFrontMotor.setPower(0.19625);
            rightFrontMotor.setPower(-0.19625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.778060906 && elapsedTime < 1.791945647) {
            leftBackMotor.setPower(0.18625);
            rightBackMotor.setPower(-0.18625);
            leftFrontMotor.setPower(0.18625);
            rightFrontMotor.setPower(-0.18625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.791945647 && elapsedTime < 1.804899398) {
            leftBackMotor.setPower(0.17625);
            rightBackMotor.setPower(-0.17625);
            leftFrontMotor.setPower(0.17625);
            rightFrontMotor.setPower(-0.17625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.804899398 && elapsedTime < 1.817936535) {
            leftBackMotor.setPower(0.1575);
            rightBackMotor.setPower(-0.1575);
            leftFrontMotor.setPower(0.1575);
            rightFrontMotor.setPower(-0.1575);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.817936535 && elapsedTime < 1.830850547) {
            leftBackMotor.setPower(0.1475);
            rightBackMotor.setPower(-0.1475);
            leftFrontMotor.setPower(0.1475);
            rightFrontMotor.setPower(-0.1475);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.830850547 && elapsedTime < 1.844146017) {
            leftBackMotor.setPower(0.13875);
            rightBackMotor.setPower(-0.13875);
            leftFrontMotor.setPower(0.13875);
            rightFrontMotor.setPower(-0.13875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.844146017 && elapsedTime < 1.857394352) {
            leftBackMotor.setPower(0.12);
            rightBackMotor.setPower(-0.12);
            leftFrontMotor.setPower(0.12);
            rightFrontMotor.setPower(-0.12);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.857394352 && elapsedTime < 1.87150628) {
            leftBackMotor.setPower(0.11);
            rightBackMotor.setPower(-0.11);
            leftFrontMotor.setPower(0.11);
            rightFrontMotor.setPower(-0.11);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.87150628 && elapsedTime < 1.894220709) {
            leftBackMotor.setPower(0.10125);
            rightBackMotor.setPower(-0.10125);
            leftFrontMotor.setPower(0.10125);
            rightFrontMotor.setPower(-0.10125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.894220709 && elapsedTime < 1.911525919) {
            leftBackMotor.setPower(0.0825);
            rightBackMotor.setPower(-0.0825);
            leftFrontMotor.setPower(0.0825);
            rightFrontMotor.setPower(-0.0825);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.911525919 && elapsedTime < 1.936173161) {
            leftBackMotor.setPower(0.0775);
            rightBackMotor.setPower(-0.0775);
            leftFrontMotor.setPower(0.0775);
            rightFrontMotor.setPower(-0.0775);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.936173161 && elapsedTime < 1.962999362) {
            leftBackMotor.setPower(0.07);
            rightBackMotor.setPower(-0.07);
            leftFrontMotor.setPower(0.07);
            rightFrontMotor.setPower(-0.07);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.962999362 && elapsedTime < 1.993878792) {
            leftBackMotor.setPower(0.31707912342242917);
            rightBackMotor.setPower(-0.31707912342242917);
            leftFrontMotor.setPower(-0.17957912342242918);
            rightFrontMotor.setPower(0.17957912342242918);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 1.993878792 && elapsedTime < 2.022402806) {
            leftBackMotor.setPower(0.47112016287293035);
            rightBackMotor.setPower(-0.47112016287293035);
            leftFrontMotor.setPower(-0.3386201628729303);
            rightFrontMotor.setPower(0.3386201628729303);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.022402806 && elapsedTime < 2.060129216) {
            leftBackMotor.setPower(0.5953809026168495);
            rightBackMotor.setPower(-0.5953809026168495);
            leftFrontMotor.setPower(-0.46538090261684956);
            rightFrontMotor.setPower(0.46538090261684956);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.060129216 && elapsedTime < 2.092799948) {
            leftBackMotor.setPower(0.5878809026168496);
            rightBackMotor.setPower(-0.5878809026168496);
            leftFrontMotor.setPower(-0.47288090261684956);
            rightFrontMotor.setPower(0.47288090261684956);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.092799948 && elapsedTime < 2.165967091) {
            leftBackMotor.setPower(0.662198125035489);
            rightBackMotor.setPower(-0.662198125035489);
            leftFrontMotor.setPower(-0.557198125035489);
            rightFrontMotor.setPower(0.557198125035489);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.165967091 && elapsedTime < 2.203342928) {
            leftBackMotor.setPower(0.65000124000248);
            rightBackMotor.setPower(-0.65000124000248);
            leftFrontMotor.setPower(-0.59000124000248);
            rightFrontMotor.setPower(0.59000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.203342928 && elapsedTime < 2.229339389) {
            leftBackMotor.setPower(0.6337512400024801);
            rightBackMotor.setPower(-0.6337512400024801);
            leftFrontMotor.setPower(-0.60625124000248);
            rightFrontMotor.setPower(0.60625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.229339389 && elapsedTime < 2.25553111) {
            leftBackMotor.setPower(0.62250124000248);
            rightBackMotor.setPower(-0.62250124000248);
            leftFrontMotor.setPower(-0.6175012400024801);
            rightFrontMotor.setPower(0.6175012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.25553111 && elapsedTime < 2.283578144) {
            leftBackMotor.setPower(0.60375124000248);
            rightBackMotor.setPower(-0.60375124000248);
            leftFrontMotor.setPower(-0.63625124000248);
            rightFrontMotor.setPower(0.63625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.283578144 && elapsedTime < 2.31041122) {
            leftBackMotor.setPower(0.5925012400024801);
            rightBackMotor.setPower(-0.5925012400024801);
            leftFrontMotor.setPower(-0.64750124000248);
            rightFrontMotor.setPower(0.64750124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.31041122 && elapsedTime < 2.335559087) {
            leftBackMotor.setPower(0.57375124000248);
            rightBackMotor.setPower(-0.57375124000248);
            leftFrontMotor.setPower(-0.66625124000248);
            rightFrontMotor.setPower(0.66625124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.335559087 && elapsedTime < 2.364602163) {
            leftBackMotor.setPower(0.55750124000248);
            rightBackMotor.setPower(-0.55750124000248);
            leftFrontMotor.setPower(-0.68250124000248);
            rightFrontMotor.setPower(0.68250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.364602163 && elapsedTime < 2.391453259) {
            leftBackMotor.setPower(0.54750124000248);
            rightBackMotor.setPower(-0.54750124000248);
            leftFrontMotor.setPower(-0.69250124000248);
            rightFrontMotor.setPower(0.69250124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.391453259 && elapsedTime < 2.417433939) {
            leftBackMotor.setPower(0.53125124000248);
            rightBackMotor.setPower(-0.53125124000248);
            leftFrontMotor.setPower(-0.70875124000248);
            rightFrontMotor.setPower(0.70875124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.417433939 && elapsedTime < 2.446281441) {
            leftBackMotor.setPower(0.52250124000248);
            rightBackMotor.setPower(-0.52250124000248);
            leftFrontMotor.setPower(-0.7175012400024801);
            rightFrontMotor.setPower(0.7175012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.446281441 && elapsedTime < 2.478581236) {
            leftBackMotor.setPower(0.51625124000248);
            rightBackMotor.setPower(-0.51625124000248);
            leftFrontMotor.setPower(-0.72375124000248);
            rightFrontMotor.setPower(0.72375124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.478581236 && elapsedTime < 2.504601499) {
            leftBackMotor.setPower(0.50500124000248);
            rightBackMotor.setPower(-0.50500124000248);
            leftFrontMotor.setPower(-0.73500124000248);
            rightFrontMotor.setPower(0.73500124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.504601499 && elapsedTime < 2.528565721) {
            leftBackMotor.setPower(0.50250124000248);
            rightBackMotor.setPower(-0.50250124000248);
            leftFrontMotor.setPower(-0.7375012400024801);
            rightFrontMotor.setPower(0.7375012400024801);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.528565721 && elapsedTime < 2.564601193) {
            leftBackMotor.setPower(0.50000124000248);
            rightBackMotor.setPower(-0.50000124000248);
            leftFrontMotor.setPower(-0.74000124000248);
            rightFrontMotor.setPower(0.74000124000248);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.564601193 && elapsedTime < 2.603399113) {
            leftBackMotor.setPower(-0.125);
            rightBackMotor.setPower(0.125);
            leftFrontMotor.setPower(-0.125);
            rightFrontMotor.setPower(0.125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.603399113 && elapsedTime < 2.668100266) {
            leftBackMotor.setPower(-0.12375);
            rightBackMotor.setPower(0.12375);
            leftFrontMotor.setPower(-0.12375);
            rightFrontMotor.setPower(0.12375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.668100266 && elapsedTime < 2.68279058) {
            leftBackMotor.setPower(-0.08875);
            rightBackMotor.setPower(0.08875);
            leftFrontMotor.setPower(-0.08875);
            rightFrontMotor.setPower(0.08875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.68279058 && elapsedTime < 2.696642977) {
            leftBackMotor.setPower(-0.08);
            rightBackMotor.setPower(0.08);
            leftFrontMotor.setPower(-0.08);
            rightFrontMotor.setPower(0.08);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.696642977 && elapsedTime < 2.708371311) {
            leftBackMotor.setPower(-0.06125);
            rightBackMotor.setPower(0.06125);
            leftFrontMotor.setPower(-0.06125);
            rightFrontMotor.setPower(0.06125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.708371311 && elapsedTime < 2.722361469) {
            leftBackMotor.setPower(-0.05375);
            rightBackMotor.setPower(0.05375);
            leftFrontMotor.setPower(-0.05375);
            rightFrontMotor.setPower(0.05375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.722361469 && elapsedTime < 2.73516147) {
            leftBackMotor.setPower(-0.04625);
            rightBackMotor.setPower(0.04625);
            leftFrontMotor.setPower(-0.04625);
            rightFrontMotor.setPower(0.04625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.73516147 && elapsedTime < 2.748263503) {
            leftBackMotor.setPower(-0.03875);
            rightBackMotor.setPower(0.03875);
            leftFrontMotor.setPower(-0.03875);
            rightFrontMotor.setPower(0.03875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.748263503 && elapsedTime < 2.760776837) {
            leftBackMotor.setPower(-0.02125);
            rightBackMotor.setPower(0.02125);
            leftFrontMotor.setPower(-0.02125);
            rightFrontMotor.setPower(0.02125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.760776837 && elapsedTime < 2.77361137) {
            leftBackMotor.setPower(-0.0125);
            rightBackMotor.setPower(0.0125);
            leftFrontMotor.setPower(-0.0125);
            rightFrontMotor.setPower(0.0125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.77361137 && elapsedTime < 2.786620486) {
            leftBackMotor.setPower(-0.005);
            rightBackMotor.setPower(0.005);
            leftFrontMotor.setPower(-0.005);
            rightFrontMotor.setPower(0.005);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.786620486 && elapsedTime < 2.799560383) {
            leftBackMotor.setPower(0.00875);
            rightBackMotor.setPower(-0.00875);
            leftFrontMotor.setPower(0.00875);
            rightFrontMotor.setPower(-0.00875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.799560383 && elapsedTime < 2.812952051) {
            leftBackMotor.setPower(0.01375);
            rightBackMotor.setPower(-0.01375);
            leftFrontMotor.setPower(0.01375);
            rightFrontMotor.setPower(-0.01375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.812952051 && elapsedTime < 2.825374188) {
            leftBackMotor.setPower(0.01875);
            rightBackMotor.setPower(-0.01875);
            leftFrontMotor.setPower(0.01875);
            rightFrontMotor.setPower(-0.01875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.825374188 && elapsedTime < 2.838670803) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.838670803 && elapsedTime < 2.856656691) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(1.0);
            rightServo.setPosition(0.0);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 2.856656691 && elapsedTime < 2.895740705) {
            leftBackMotor.setPower(0.03);
            rightBackMotor.setPower(-0.03);
            leftFrontMotor.setPower(0.03);
            rightFrontMotor.setPower(-0.03);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(1.0);
            rightServo.setPosition(0.0);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 2.895740705 && elapsedTime < 2.908537425) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(1.0);
            rightServo.setPosition(0.0);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 2.908537425 && elapsedTime < 3.438883155) {
            leftBackMotor.setPower(0.03);
            rightBackMotor.setPower(-0.03);
            leftFrontMotor.setPower(0.03);
            rightFrontMotor.setPower(-0.03);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(1.0);
            rightServo.setPosition(0.0);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 3.438883155 && elapsedTime < 5.067905401) {
            leftBackMotor.setPower(0.03);
            rightBackMotor.setPower(-0.03);
            leftFrontMotor.setPower(0.03);
            rightFrontMotor.setPower(-0.03);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 5.067905401 && elapsedTime < 5.111433061) {
            leftBackMotor.setPower(0.03);
            rightBackMotor.setPower(-0.03);
            leftFrontMotor.setPower(0.03);
            rightFrontMotor.setPower(-0.03);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.5661243200302124);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 5.111433061 && elapsedTime < 5.153869107) {
            leftBackMotor.setPower(0.03);
            rightBackMotor.setPower(-0.03);
            leftFrontMotor.setPower(0.03);
            rightFrontMotor.setPower(-0.03);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.9916562438011169);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 5.153869107 && elapsedTime < 5.553240553) {
            leftBackMotor.setPower(0.03);
            rightBackMotor.setPower(-0.03);
            leftFrontMotor.setPower(0.03);
            rightFrontMotor.setPower(-0.03);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 5.553240553 && elapsedTime < 6.244016247) {
            leftBackMotor.setPower(0.03125);
            rightBackMotor.setPower(-0.03125);
            leftFrontMotor.setPower(0.03125);
            rightFrontMotor.setPower(-0.03125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 6.244016247 && elapsedTime < 6.272541979) {
            leftBackMotor.setPower(0.03125);
            rightBackMotor.setPower(-0.03125);
            leftFrontMotor.setPower(0.03125);
            rightFrontMotor.setPower(-0.03125);
            gripMotor.setPower(0.0);
            armMotor.setPower(-0.3158114552497864);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 6.272541979 && elapsedTime < 6.353199227) {
            leftBackMotor.setPower(0.03125);
            rightBackMotor.setPower(-0.03125);
            leftFrontMotor.setPower(0.03125);
            rightFrontMotor.setPower(-0.03125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.353199227 && elapsedTime < 6.366947717) {
            leftBackMotor.setPower(0.03375);
            rightBackMotor.setPower(-0.03375);
            leftFrontMotor.setPower(0.03375);
            rightFrontMotor.setPower(-0.03375);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.366947717 && elapsedTime < 6.379734437) {
            leftBackMotor.setPower(0.0325);
            rightBackMotor.setPower(-0.0325);
            leftFrontMotor.setPower(0.0325);
            rightFrontMotor.setPower(-0.0325);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.379734437 && elapsedTime < 6.392732564) {
            leftBackMotor.setPower(0.03125);
            rightBackMotor.setPower(-0.03125);
            leftFrontMotor.setPower(0.03125);
            rightFrontMotor.setPower(-0.03125);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.392732564 && elapsedTime < 6.405521315) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 6.405521315 && elapsedTime < 6.431826943) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.431826943 && elapsedTime < 6.448536007) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.06549853831529617);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.448536007 && elapsedTime < 6.461830175) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.06549853831529617);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.461830175 && elapsedTime < 6.473959655) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.06549853831529617);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.473959655 && elapsedTime < 6.490074292) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.18231122195720673);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 6.490074292 && elapsedTime < 6.515866066) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.18231122195720673);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 6.515866066 && elapsedTime < 6.554911903) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.18231122195720673);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 6.554911903 && elapsedTime < 6.597057167) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.02377972938120365);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 6.597057167 && elapsedTime < 6.796918437) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 6.796918437 && elapsedTime < 7.143353576) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(-0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 7.143353576 && elapsedTime < 7.549620544) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 7.549620544 && elapsedTime < 7.633819042) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.3);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 7.633819042 && elapsedTime < 7.750825772) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 7.750825772 && elapsedTime < 7.791247964) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.3158114552497864);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 7.791247964 && elapsedTime < 7.832749686) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.9165623784065247);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 7.832749686 && elapsedTime < 7.848901094) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 7.848901094 && elapsedTime < 7.861533752) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 7.861533752 && elapsedTime < 7.915577559) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 7.915577559 && elapsedTime < 7.929983759) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 7.929983759 && elapsedTime < 7.968797356) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 7.968797356 && elapsedTime < 8.006898662) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 8.006898662 && elapsedTime < 8.031943613) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 8.031943613 && elapsedTime < 8.057001688) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 8.057001688 && elapsedTime < 8.069963148) {
            leftBackMotor.setPower(0.02875);
            rightBackMotor.setPower(-0.02875);
            leftFrontMotor.setPower(0.02875);
            rightFrontMotor.setPower(-0.02875);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 8.069963148 && elapsedTime < 8.881130103) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 8.881130103 && elapsedTime < 8.894468751) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 8.894468751 && elapsedTime < 8.907562762) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 8.907562762 && elapsedTime < 8.950795162) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.41593658924102783);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 8.950795162 && elapsedTime < 9.019356263) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 9.019356263 && elapsedTime < 9.057434496) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 9.057434496 && elapsedTime < 9.109529397) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 9.109529397 && elapsedTime < 9.135583254) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.135583254 && elapsedTime < 9.148892943) {
            leftBackMotor.setPower(0.025);
            rightBackMotor.setPower(-0.025);
            leftFrontMotor.setPower(0.025);
            rightFrontMotor.setPower(-0.025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 9.148892943 && elapsedTime < 9.180596227) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.180596227 && elapsedTime < 9.194011385) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 9.194011385 && elapsedTime < 9.236992222) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.10721736401319504);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.236992222 && elapsedTime < 9.278045768) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.6161869168281555);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.278045768 && elapsedTime < 9.303853739) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 9.303853739 && elapsedTime < 9.35888838) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(1.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.35888838 && elapsedTime < 9.377567809) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.9249061346054077);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.377567809 && elapsedTime < 9.391725415) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.9249061346054077);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.391725415 && elapsedTime < 9.407882291) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.3491864800453186);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.407882291 && elapsedTime < 9.420838438) {
            leftBackMotor.setPower(0.0275);
            rightBackMotor.setPower(-0.0275);
            leftFrontMotor.setPower(0.0275);
            rightFrontMotor.setPower(-0.0275);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.3491864800453186);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if (elapsedTime > 9.420838438 && elapsedTime < 9.433757346) {
            leftBackMotor.setPower(0.025);
            rightBackMotor.setPower(-0.025);
            leftFrontMotor.setPower(0.025);
            rightFrontMotor.setPower(-0.025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.3491864800453186);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 9.433757346 && elapsedTime < 9.992128183) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 9.992128183 && elapsedTime < 10.250925813) {
            leftBackMotor.setPower(0.02625);
            rightBackMotor.setPower(-0.02625);
            leftFrontMotor.setPower(0.02625);
            rightFrontMotor.setPower(-0.02625);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 10.250925813 && elapsedTime < 10.548168342) {
            leftBackMotor.setPower(0.025);
            rightBackMotor.setPower(-0.025);
            leftFrontMotor.setPower(0.025);
            rightFrontMotor.setPower(-0.025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.98);
        }
        if(elapsedTime > 10.548168342 && elapsedTime < 10.700740701) {
            leftBackMotor.setPower(0.025);
            rightBackMotor.setPower(-0.025);
            leftFrontMotor.setPower(0.025);
            rightFrontMotor.setPower(-0.025);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
        if(elapsedTime > 10.700740701 && elapsedTime < 11.523819898) {
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            gripMotor.setPower(0.0);
            armMotor.setPower(0.0);
            leftServo.setPosition(0.1);
            rightServo.setPosition(0.9);
            skystoneServo.setPosition(0.52);
        }
    endTime = 11.523819898;
    }
}
