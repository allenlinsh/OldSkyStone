package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZXY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import java.util.ArrayList;
import java.util.List;

public class VuforiaAutonomousBlue extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor gripMotor;
    private DcMotor armMotor;
    private Servo leftServo;
    private Servo rightServo;
    private VuforiaLocalizer vuforia;
    private OpenGLMatrix lastLocation;
    private WebcamName webcamName;
    private ColorSensor colorSensor;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    private static final boolean PHONE_IS_PORTRAIT                      = false  ;
    private static final String VUFORIA_KEY                             = "AZ2DQXn/////AAABmV2NdKltaEv7nZA9fnEAYpONbuK/sGbJG7tGyKNwNcaEPXyRq7V3WKOcmTwGwpTyl5Sm/2tJR6t5VFwarUda2dnW20yakyCThxpQcM4xXu5xnY3/HVPcTCEloelyqgf0jSbw94/N7b2n7jdkdA/CYYvJOQo7/cQ3cnoa/3aZ1LpJgeYy8SHLDeLe2nwpARjaHokhhG835GzpFlTXa1IhHjo0Lsvm2qTM8WqgLIKYYep1urYPAPYYUsT+WXUSLCbw0TkQcIVLP6FdvQL6FtCeRoA29fpTdq5L4RFsdqac2fELdXY8rjZpJDx4g/8KN6aw1iG4ZocJBzgzhELtCgQbqJppGGk7z/CRTvcXL1dhIunZ";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float stoneZ           = 2.00f * mmPerInch;
    private static final float bridgeZ          = 6.42f * mmPerInch;
    private static final float bridgeY          = 23 * mmPerInch;
    private static final float bridgeX          = 5.18f * mmPerInch;
    private static final float bridgeRotY       = 59;                                 // Units are degrees
    private static final float bridgeRotZ       = 180;
    private static final float halfField        = 72 * mmPerInch;
    private static final float quadField        = 36 * mmPerInch;
    private boolean targetVisible               = false;
    private float webcamXRotate                  = 0;
    private float webcamYRotate                  = 0;
    private float webcamZRotate                  = 0;

    int ticksPerRev         = 1440;
    double timePerUnit      = 1075;
    double timePerDegreeCW  = 7.95;
    double timePerDegreeCCW = 8;
    double power = 0.5;

    @Override
    public void runOpMode() {
        imu         = hardwareMap.get(BNO055IMU.class, "imu");
        leftMotor   = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor  = hardwareMap.get(DcMotor.class, "rightMotor");
        gripMotor   = hardwareMap.get(DcMotor.class, "gripMotor");
        armMotor    = hardwareMap.get(DcMotor.class, "armMotor");
        leftServo   = hardwareMap.get(Servo.class, "leftServo");
        rightServo  = hardwareMap.get(Servo.class, "rightServo");
        webcamName  = hardwareMap.get(WebcamName.class, "Logitech C310");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // initialize vuforia localizer
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // load the data sets for trackable objects
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // localization on the field
        // If you are standing in the Red Alliance Station looking towards the center of the field,
        //      - The X axis runs from your left to the right. (positive from the center to the right)
        //      - The Y axis runs from the Red Alliance Station towards the other side of the field
        //        where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
        //      - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)

        // set the position of the Stone Target
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0, -90)));

        // set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 0, bridgeRotY, bridgeRotZ)));
        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 0, -bridgeRotY, 0)));
        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 0, bridgeRotY, 0)));

        // set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0, 180)));
        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0, 180)));
        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0 , 90)));
        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0, 90)));
        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0, 0)));
        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0, 0)));
        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0 , -90)));
        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZXY, DEGREES, 90, 0, -90)));

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;    // distance in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;    // distance above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;                   // distance left of the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, webcamYRotate, webcamZRotate, webcamXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // enable skystone identification
        targetsSkyStone.activate();
        while (!isStopRequested()) {
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());

                    if(trackable.getName().equals("Stone Target")){
                        telemetry.addLine("Stone Target Is Visible");
                    }

                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            String positionSkystone = "";
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                double xPosition = translation.get(0);
                if(xPosition < -10){
                    positionSkystone = "left";
                }else{
                    positionSkystone = "center";
                }

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                positionSkystone = "right";
                telemetry.addData("Visible Target", "none");
            }
            telemetry.addData("Skystone Position", positionSkystone);
            telemetry.update();
        }

        // disable skystone identification;
        targetsSkyStone.deactivate();

        // set motor direction
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // set motor zero power behavior
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // set motor mode
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // initialize the hook
        hookOff();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for the game to start
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        // **************************************************
        // autonomous path 
        //
        // stop
        //
        // **************************************************

        // stop
        drive(0);

    }
    public void drive (double power) {
        // default: forward
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void driveFor (double unit) {
        int time = (int)(timePerUnit * unit);
        sleep(time);
    }
    public void turn (double power) {
        // default: cw
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }
    public void turnForCW (double degree) {
        int time = (int)(timePerDegreeCW * degree);
        sleep(time);
    }
    public void turnForCCW (double degree) {
        int time = (int)(timePerDegreeCCW * degree);
        sleep(time);
    }
    public void hookOn () {
        leftServo.setPosition(1);
        rightServo.setPosition(0);
    }
    public void hookOff () {
        leftServo.setPosition(0);
        rightServo.setPosition(1);
    }
    /*
    public int drivePos (double distance) {
        double circumference = Math.PI * 4.0; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(distance / inPerRev);

        return targetPos;
    }
    public double getCurrentAngle(){
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double currentAngle = angles.thirdAngle;

        return currentAngle;
    }
    public void turnPos (double angle) {
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialAngle = angles.firstAngle;
        double robotAngle = angles.firstAngle;
        double previousAngle = angles.firstAngle;
        double minMotorPower = 0.3 ;
        double powerScaleFactor;

        // calculate target angle
        double targetAngle = initialAngle + angle;
        double diffAngle = Math.abs(targetAngle - robotAngle);
    }
    */
}