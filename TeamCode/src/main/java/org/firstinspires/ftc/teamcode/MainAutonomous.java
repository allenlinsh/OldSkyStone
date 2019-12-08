package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class MainAutonomous extends LinearOpMode {
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

    int ticksPerRev             = 1440;
    double timePerInchLateral   = 1075/24;
    double timePerInchAxial     = 1275/24;
    double timePerDegreeCCW     = 11.15;
    double timePerDegreeCW     = 11.6;
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
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // wait for the game to start
        waitForStart();
        
        // program start here
        telemetry.addData("Status", "Running");
        telemetry.addData("Alliance", alliance);
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
        
        // autonomous start
        if (opModeIsActive() && alliance == "red") {
            armForward();
            driveRight(8, 1);
            driveForward(34, 1);
            hookOn();
            pause();
            driveBackward(35, 1);
            hookOff();
            driveLeft(32, 1);
            turn(90, 1);
            driveLeft(16, 1);
            driveForward(10, 1);
            driveLeft(2, 1);
            driveBackward(26, 1);
        }
        else if (opModeIsActive() && alliance == "blue") {
            armForward();
            driveLeft(8, 1);
            driveForward(34, 1);
            hookOn();
            pause();
            driveBackward(35, 1);
            hookOff();
            driveRight(32, 1);
            turn(-90, 1);
            driveRight(16, 1);
            driveForward(10, 1);
            driveRight(2, 1);
            driveBackward(26, 1);
        }      
    }
    public void pause() {
        sleep(100);
    }
    public void hookOn() {
        leftServo.setPosition(1);
        rightServo.setPosition(1);
        pause();
    }
    public void hookOff() {
        leftServo.setPosition(0.35);
        rightServo.setPosition(0.61);
        pause();
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
    public void driveForward(double distance, double power) {
        int runTime = (int)(distance * timePerInchLateral);
        
        // set motor speed
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        sleep(runTime);
        
        stopMotor();
        pause();
    }
    public void driveBackward(double distance, double power) {
        driveForward(distance, -power);
    }
    public void driveLeft(double distance, double power) {
        int runTime = (int)(distance * timePerInchAxial);
        
        // set motor speed
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(-power);
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        sleep(runTime);
        
        stopMotor();
    }
    public void driveRight(double distance, double power) {
        driveLeft(distance, -power);
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
        else return;

        stopMotor();
        pause();
    }
    public void turnWithFoundation(int angle, double power) {
        int extraAngle = 90;
        if (angle > 0) {
            turn(angle + extraAngle, power);   
        }
        else if (angle < 0) {
            turn(angle - extraAngle, power);
        }
        else return;
    }
}
