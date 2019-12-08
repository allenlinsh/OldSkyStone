package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZXY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class PreciseAutonomous extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor gripMotor;
    private DcMotor armMotor;
    private Servo leftServo;
    private Servo rightServo;
    private WebcamName webcamName;
    private VuforiaLocalizer vuforia;
    private OpenGLMatrix lastLocation;
    private ColorSensor colorSensor;
    private Orientation lastAngles;
    
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    private static final boolean PHONE_IS_PORTRAIT                      = true  ;
    private static final String VUFORIA_KEY                             = "AZ2DQXn/////AAABmV2NdKltaEv7nZA9fnEAYpONbuK/sGbJG7tGyKNwNcaEPXyRq7V3WKOcmTwGwpTyl5Sm/2tJR6t5VFwarUda2dnW20yakyCThxpQcM4xXu5xnY3/HVPcTCEloelyqgf0jSbw94/N7b2n7jdkdA/CYYvJOQo7/cQ3cnoa/3aZ1LpJgeYy8SHLDeLe2nwpARjaHokhhG835GzpFlTXa1IhHjo0Lsvm2qTM8WqgLIKYYep1urYPAPYYUsT+WXUSLCbw0TkQcIVLP6FdvQL6FtCeRoA29fpTdq5L4RFsdqac2fELdXY8rjZpJDx4g/8KN6aw1iG4ZocJBzgzhELtCgQbqJppGGk7z/CRTvcXL1dhIunZ";
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float stoneZ           = 2.00f * mmPerInch;
    private static final float bridgeZ          = 6.42f * mmPerInch;
    private static final float bridgeY          = 23 * mmPerInch;
    private static final float bridgeX          = 5.18f * mmPerInch;
    private static final float bridgeRotY       = 59;                       // Units are degrees
    private static final float bridgeRotZ       = 180;
    private static final float halfField        = 72 * mmPerInch;
    private static final float quadField        = 36 * mmPerInch;
    private boolean targetVisible               = false;
    private float webcamXRotate                 = 0;
    private float webcamYRotate                 = 0;
    private float webcamZRotate                 = 0;

    int ticksPerRev         = 1440;
    double timePerUnit      = 1075;
    double timePerDegreeCW  = 7.95;
    double timePerDegreeCCW = 8;
    int threshold           = 10;
    String alliance         = "blue";
    double globalAngle      = 0;

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
        webcamName              = hardwareMap.get(WebcamName.class, "Logitech C310");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        colorSensor             = hardwareMap.get(ColorSensor.class, "colorSensor");
    
        // set motor direction
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // initialize the hook
        hookOff();
        
        // initialize vuforia localizer
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        
        // initialize imu
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        
        /*  
        *   ----------------------------------------------------------
        *   start of vuforia setup 
        *   ----------------------------------------------------------
        */

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

        final float CAMERA_FORWARD_DISPLACEMENT  = 6.0f * mmPerInch;    // distance in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 3.5f * mmPerInch;    // distance above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;                   // distance left of the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, webcamYRotate, webcamZRotate, webcamXRotate));
        if (opModeIsActive()) {
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);   
            }
        }
        
        /* 
        *   ----------------------------------------------------------
        *   end of vuforia setup 
        *   ----------------------------------------------------------
        */

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for the game to start
        waitForStart();

        // enable skystone identification
        targetsSkyStone.activate();
        
        // progra`m start here
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
        String posSkystone = "left";
        if (alliance == "red") {
            //posSkystone = lookForSkystone(allTrackables);
            if (posSkystone == "left") {
            }   
            else if (posSkystone == "center") {
            }
            else if (posSkystone == "right") {
            } 
        }
        else if (alliance == "blue") {
            //posSkystone = lookForSkystone(allTrackables);
            if (posSkystone == "left") {
                //gripRelease();
                driveForward(24, 1);
            }   
            else if (posSkystone == "center") {

            }
            else if (posSkystone == "right") {

            } 
        }      
        // disable skystone identification;
        targetsSkyStone.deactivate();
    }
    public void pause() {
        sleep(100);
    }
    public void hookOn() {
        leftServo.setPosition(1);
        rightServo.setPosition(1);
    }
    public void hookOff() {
        leftServo.setPosition(0.35);
        rightServo.setPosition(0.61);
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
    public boolean isAtTargetThreshold(int targetPos, int currPos, int threshold) {
        int error = targetPos - currPos;
        if (Math.abs(error) < threshold) {
            return true;
        }
        else {
            return false;
        }
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
        sleep(1600);
        armMotor.setPower(0);
        pause();
    }
    public void driveForward(double distance, double power) {
        double circumference = Math.PI * 4; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(distance / inPerRev);
        
        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // set motor target position
        leftBackMotor.setTargetPosition(targetPos);
        rightBackMotor.setTargetPosition(targetPos);
        leftFrontMotor.setTargetPosition(targetPos);
        rightFrontMotor.setTargetPosition(targetPos);
        
        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // set motor speed
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        
        // get current position
        int currLeftBackPos = leftBackMotor.getCurrentPosition();
        int currRightBackPos = rightBackMotor.getCurrentPosition();
        int currLeftFrontPos = leftFrontMotor.getCurrentPosition();
        int currRightFrontPos = rightFrontMotor.getCurrentPosition();
        
        boolean leftBackIsInPos = isAtTargetThreshold(targetPos, currLeftBackPos, threshold);
        boolean rightBackIsInPos = isAtTargetThreshold(targetPos, currRightBackPos, threshold);
        boolean leftFrontIsInPos = isAtTargetThreshold(targetPos, currLeftFrontPos, threshold);
        boolean rightFrontIsInPos = isAtTargetThreshold(targetPos, currRightFrontPos, threshold);
        
        if (leftBackIsInPos && rightBackIsInPos && leftFrontIsInPos && rightFrontIsInPos) {
            stopMotor();
        }
    }
    public void driveBackward(double distance, double power) {
        driveForward(-distance, -power);
    }
    public void driveLeft(double distance, double power) {
        double circumference = Math.PI * 4; // pi * diameter
        double inPerRev = circumference / ticksPerRev;
        int targetPos = (int)(2.5 * distance / inPerRev);
        
        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // set motor target position
        leftBackMotor.setTargetPosition(targetPos);
        rightBackMotor.setTargetPosition(-targetPos);
        leftFrontMotor.setTargetPosition(-targetPos);
        rightFrontMotor.setTargetPosition(targetPos);
        
        // set motor mode
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // set motor speed
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        
        // get current position
        int currLeftBackPos = leftBackMotor.getCurrentPosition();
        int currRightBackPos = rightBackMotor.getCurrentPosition();
        int currLeftFrontPos = leftFrontMotor.getCurrentPosition();
        int currRightFrontPos = rightFrontMotor.getCurrentPosition();
        
        boolean leftBackIsInPos = isAtTargetThreshold(targetPos, currLeftBackPos, threshold);
        boolean rightBackIsInPos = isAtTargetThreshold(-targetPos, currRightBackPos, threshold);
        boolean leftFrontIsInPos = isAtTargetThreshold(-targetPos, currLeftFrontPos, threshold);
        boolean rightFrontIsInPos = isAtTargetThreshold(targetPos, currRightFrontPos, threshold);
        
        if (leftBackIsInPos && rightBackIsInPos && leftFrontIsInPos && rightFrontIsInPos) {
            stopMotor();
            pause();
        }
    }
    public void driveRight(double distance, double power) {
        driveLeft(-distance, -power);
    }
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

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
    public void turn(int angle, double power) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (angle < 0)
        {   // turn clockwise.
            leftPower = power;
            rightPower = -power;
        }
        else if (angle > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftBackMotor.setPower(leftPower);
        leftFrontMotor.setPower(leftPower);
        rightBackMotor.setPower(rightPower);
        rightFrontMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (angle < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > angle) {}
        }
            else    // left turn.
            while (opModeIsActive() && getAngle() < angle) {}

        stopMotor();
        pause();

        // reset angle tracking on new heading.
        resetAngle();
    }
    public String lookForSkystone(List<VuforiaTrackable> allTrackables) {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        if (opModeIsActive()) {
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
        }

        // Provide feedback as to where the robot is located (if we know).
        String positionSkystone = "";
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Target Position", translation.get(2) / mmPerInch);

            double xPosition = translation.get(0);
            if(xPosition < -10){
                positionSkystone = "left";
            }
            else{
                positionSkystone = "center";
            }

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Target Rotation", rotation.thirdAngle);
        }
        else {
            positionSkystone = "right";
            telemetry.addData("Visible Target", "none");
        }
        telemetry.addData("Skystone Position", positionSkystone);

        return positionSkystone;
    }
}