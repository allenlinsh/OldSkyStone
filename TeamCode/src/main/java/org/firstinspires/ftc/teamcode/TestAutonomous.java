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
public class TestAutonomous extends LinearOpMode {
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
    double globalAngle, power = 0.6, correction;
    double slowPower = 0.25;
    double leftColorThreshod, rightColorThreshold;
    int currPos, startPos, endPos;
    double slowFactor = 0.25;
    double leftServoState, rightServoState, leftSkystoneServoState, rightSkystoneServoState;
    double leftBackPower, rightBackPower, leftFrontPower, rightFrontPower = 0;
    boolean runAutonomous = true;

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
        rightColorSensor         = hardwareMap.get(ColorSensor.class, "rightColorSensor");
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
        telemetry.update();

        // wait for the game to start
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.addData("Alliance", alliance);

        while (opModeIsActive()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 power", power);
            telemetry.update();

            if(runAutonomous) {
                switch (alliance) {
                    case "blue":
                        switch (mode) {
                            case "full":
                                runAutonomous = false;
                            case "foundation":
                                runAutonomous = false;
                            case "skystone":
                                runAutonomous = false;
                            case "park":
                                runAutonomous = false;
                        }
                    case "red":
                        switch (mode) {
                            case "full":
                                runAutonomous = false;
                            case "foundation":
                                //driveLeft(0.5, power);
                                //driveBackward(1.25, power);
                                //hookOn();
                                //driveForward(1, power);
                                rotate(90, slowPower);
                                //hookOff();
                                //driveRight(1, power);
                                //driveForward(1.5, power);
                                runAutonomous = false;
                            case "skystone":
                                driveRight(1.25, power);
                                while(rightSkystoneServoState == 0.52) {
                                    driveBackward(0.1,slowPower);
                                    rightColorThreshold = rightColorSensor.red() * rightColorSensor.green() / (rightColorSensor.blue() * rightColorSensor.blue());
                                    if (rightColorThreshold <= 3) {
                                        rightSkystoneOn();
                                    }
                                }
                                runAutonomous = false;
                            case "park":
                                runAutonomous = false;
                        }
                }
            }
        }
        stopMotor();
    }

    public void motorPause() {
        sleep(75);
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
        servoPause();
    }
    public void leftSkystoneOn() {
        leftSkystoneServoState = 0.98;
        leftSkystoneServo.setPosition(leftSkystoneServoState);
        if (leftSkystoneServo.getPosition()  != 0.98) {
            stopMotor();
        }
        servoPause();
    }
    public void rightSkystoneOn() {
        rightSkystoneServoState = 0.98;
        rightSkystoneServo.setPosition(rightSkystoneServoState);
        if (rightSkystoneServo.getPosition()  != 0.98) {
            stopMotor();
        }
        servoPause();
    }
    public void leftSkystoneOff() {
        leftSkystoneServoState = 0.52;
        leftSkystoneServo.setPosition(leftSkystoneServoState);
        if (leftSkystoneServo.getPosition()  != 0.52) {
            stopMotor();
        }
        servoPause();
    }
    public void rightSkystoneOff() {
        rightSkystoneServoState = 0.52;
        rightSkystoneServo.setPosition(rightSkystoneServoState);
        if (rightSkystoneServo.getPosition()  != 0.52) {
            stopMotor();
        }
        servoPause();
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
        power = power * 100 / 127;

        // calculate target position
        double circumference = Math.PI * 3.75; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(block * 24 / inPerRev);

        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackMotor.setTargetPosition(targetPos);
        rightBackMotor.setTargetPosition(targetPos);
        leftFrontMotor.setTargetPosition(targetPos);
        rightFrontMotor.setTargetPosition(targetPos);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor speed
        leftBackMotor.setPower(slowPower);
        rightBackMotor.setPower(slowPower);
        leftFrontMotor.setPower(slowPower);
        rightFrontMotor.setPower(slowPower);

        while(opModeIsActive() && leftBackMotor.isBusy()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();
            currPos = (int)Math.abs(leftBackMotor.getCurrentPosition());
            startPos = (int)Math.abs(targetPos * slowFactor);
            endPos = (int)Math.abs(targetPos * (1-slowFactor));

            telemetry.addData("targetPos",targetPos);
            telemetry.addData("currPos", leftFrontMotor.getCurrentPosition());
            telemetry.update();

            // start
            if (currPos < startPos) {
                leftBackPower = slowPower + currPos / startPos * (power - slowPower) - correction;
                rightBackPower = slowPower + currPos / startPos * (power - slowPower) + correction;
                leftFrontPower = slowPower + currPos / startPos * (power - slowPower) - correction;
                rightFrontPower = slowPower + currPos / startPos * (power - slowPower) + correction;
            }
            // end
            else if (currPos > endPos) {
                int tempPos = currPos - endPos;
                int finalPos = Math.abs(targetPos) - endPos;
                leftBackPower = power - tempPos / finalPos * (power - slowPower) - correction;
                rightBackPower = power - tempPos / finalPos * (power - slowPower) + correction;
                leftFrontPower = power - tempPos / finalPos * (power - slowPower) - correction;
                rightFrontPower = power - tempPos / finalPos * (power - slowPower) + correction;
            }
            // running
            else {
                leftBackPower = power - correction;
                rightBackPower = power + correction;
                leftFrontPower = power - correction;
                rightFrontPower = power + correction;
            }

            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
        }
        stopMotor();
    }
    public void driveBackward(double block, double power) {
        power = power * 100 / 127;

        // calculate target position
        double circumference = Math.PI * 3.75; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(block * 24 / inPerRev);

        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackMotor.setTargetPosition(-targetPos);
        rightBackMotor.setTargetPosition(-targetPos);
        leftFrontMotor.setTargetPosition(-targetPos);
        rightFrontMotor.setTargetPosition(-targetPos);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor speed
        leftBackMotor.setPower(-slowPower);
        rightBackMotor.setPower(-slowPower);
        leftFrontMotor.setPower(-slowPower);
        rightFrontMotor.setPower(-slowPower);

        while(opModeIsActive() && leftBackMotor.isBusy()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();
            currPos = (int)Math.abs(leftBackMotor.getCurrentPosition());
            startPos = (int)Math.abs(targetPos * slowFactor);
            endPos = (int)Math.abs(targetPos * (1-slowFactor));

            // start
            if (currPos < startPos) {
                leftBackPower = -slowPower - currPos / startPos * (power - slowPower) - correction;
                rightBackPower = -slowPower - currPos / startPos * (power - slowPower) + correction;
                leftFrontPower = -slowPower - currPos / startPos * (power - slowPower) - correction;
                rightFrontPower = -slowPower - currPos / startPos * (power - slowPower) + correction;
            }
            // end
            else if (currPos > endPos) {
                int tempPos = currPos - endPos;
                int finalPos = Math.abs(targetPos) - endPos;
                leftBackPower = -power + tempPos / finalPos * (power - slowPower) - correction;
                rightBackPower = -power + tempPos / finalPos * (power - slowPower) + correction;
                leftFrontPower = -power + tempPos / finalPos * (power - slowPower) - correction;
                rightFrontPower = -power + tempPos / finalPos * (power - slowPower) + correction;
            }
            // running
            else {
                leftBackPower = -power - correction;
                rightBackPower = -power + correction;
                leftFrontPower = -power - correction;
                rightFrontPower = -power + correction;
            }

            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
        }
        stopMotor();
    }
    public void driveLeft(double block, double power) {
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
        leftBackMotor.setPower(slowPower);
        rightBackMotor.setPower(-slowPower);
        leftFrontMotor.setPower(-slowPower);
        rightFrontMotor.setPower(slowPower);

        while(opModeIsActive() && leftBackMotor.isBusy()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();
            currPos = (int)Math.abs(leftBackMotor.getCurrentPosition());
            startPos = (int)Math.abs(targetPos * slowFactor);
            endPos = (int)Math.abs(targetPos * (1-slowFactor));

            // start
            if (currPos < startPos) {
                leftBackPower = slowPower + currPos / startPos * (power - slowPower) - correction;
                rightBackPower = -slowPower - currPos / startPos * (power - slowPower) + correction;
                leftFrontPower = -slowPower - currPos / startPos * (power - slowPower) - correction;
                rightFrontPower = slowPower + currPos / startPos * (power - slowPower) + correction;
            }
            // end
            else if (currPos > endPos) {
                int tempPos = currPos - endPos;
                int finalPos = Math.abs(targetPos) - endPos;
                leftBackPower = power - tempPos / finalPos * (power - slowPower) - correction;
                rightBackPower = -power + tempPos / finalPos * (power - slowPower) + correction;
                leftFrontPower = -power + tempPos / finalPos * (power - slowPower) - correction;
                rightFrontPower = power - tempPos / finalPos * (power - slowPower) + correction;
            }
            // running
            else {
                leftBackPower = power - correction;
                rightBackPower = -power + correction;
                leftFrontPower = -power - correction;
                rightFrontPower = power + correction;
            }

            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
        }
        stopMotor();
    }
    public void driveRight(double block, double power) {
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
        leftBackMotor.setPower(-slowPower);
        rightBackMotor.setPower(slowPower);
        leftFrontMotor.setPower(slowPower);
        rightFrontMotor.setPower(-slowPower);

        while(opModeIsActive() && leftBackMotor.isBusy()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();
            currPos = (int)Math.abs(leftBackMotor.getCurrentPosition());
            startPos = (int)Math.abs(targetPos * slowFactor);
            endPos = (int)Math.abs(targetPos * (1-slowFactor));

            // start
            if (currPos < startPos) {
                leftBackPower = -slowPower - currPos / startPos * (power - slowPower) - correction;
                rightBackPower = slowPower + currPos / startPos * (power - slowPower) + correction;
                leftFrontPower = slowPower + currPos / startPos * (power - slowPower) - correction;
                rightFrontPower = -slowPower - currPos / startPos * (power - slowPower) + correction;
            }
            // end
            else if (currPos > endPos) {
                int tempPos = currPos - endPos;
                int finalPos = Math.abs(targetPos) - endPos;
                leftBackPower = -power + tempPos / finalPos * (power - slowPower) - correction;
                rightBackPower = power - tempPos / finalPos * (power - slowPower) + correction;
                leftFrontPower = power - tempPos / finalPos * (power - slowPower) - correction;
                rightFrontPower = -power + tempPos / finalPos * (power - slowPower) + correction;
            }
            // running
            else {
                leftBackPower = -power - correction;
                rightBackPower = power + correction;
                leftFrontPower = power - correction;
                rightFrontPower = -power + correction;
            }

            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
        }
        stopMotor();
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
            while (opModeIsActive() && getAngle() == 0) {
                // Use gyro to drive in a straight line.
                correction = checkDirection();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                // Use gyro to drive in a straight line.
                correction = checkDirection();
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                // Use gyro to drive in a straight line.
                correction = checkDirection();
            }

        // turn the motors off.
        stopMotor();

        // reset angle tracking on new heading.
        resetAngle();
    }
}
