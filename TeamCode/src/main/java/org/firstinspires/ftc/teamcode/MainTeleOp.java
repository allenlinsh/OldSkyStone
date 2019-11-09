package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor gripMotor;
    private DcMotor armMotor;
    private Servo leftServo;
    private Servo rightServo;

    @Override
    public void runOpMode() {
        imu         = hardwareMap.get(BNO055IMU.class, "imu");
        leftMotor   = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor  = hardwareMap.get(DcMotor.class, "rightMotor");
        gripMotor   = hardwareMap.get(DcMotor.class, "gripMotor");
        armMotor    = hardwareMap.get(DcMotor.class, "armMotor");
        leftServo   = hardwareMap.get(Servo.class, "leftServo");
        rightServo  = hardwareMap.get(Servo.class, "rightServo");

        // set the left motor direction to "forward"
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // set the grip motor direction to "forward"
        gripMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // set the arm motor zero power behavior to "brake"
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set the grip motor zero power behavior to "brake"
        gripMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // set both motors zero power behavior to "brake"
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // set both motors mode to "run without encoder"
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // set the servo position to "hook off"
        leftServo.setPosition(0);
        rightServo.setPosition(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for the game to start
        waitForStart();

        double tgtPower         = 0;
        double rtnPower         = 0;
        double gripPower        = 0;
        double armPower         = 0;
        double leftServoState   = 0;
        double rightServoState  = 1;
        // run until the end of the match
        while (opModeIsActive()) {
            // assign controller power values
            tgtPower    = -this.gamepad1.left_stick_y;
            rtnPower    = this.gamepad1.right_stick_x;
            gripPower   = 0.3 * this.gamepad2.left_stick_y;
            armPower    = 0.5 * this.gamepad2.right_stick_y;

            if (this.gamepad2.left_bumper) {
                // hook on
                leftServoState = 1;
                rightServoState = 0;
            }
            else if (this.gamepad2.right_bumper) {
                // hook off
                leftServoState = 0;
                rightServoState = 1;
            }

            leftMotor.setPower(tgtPower + rtnPower);
            rightMotor.setPower(tgtPower - rtnPower);
            gripMotor.setPower(gripPower);
            armMotor.setPower(armPower);
            
            leftServo.setPosition(leftServoState);
            rightServo.setPosition(rightServoState);

            telemetry.addData("Status", "Running");
            telemetry.addData("Power", tgtPower);
            telemetry.update();
        }
    }
}
