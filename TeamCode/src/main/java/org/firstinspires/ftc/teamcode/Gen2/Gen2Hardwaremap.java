package org.firstinspires.ftc.teamcode.Gen2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


//@Config //We need this for Dashboard to change variables
public class Gen2Hardwaremap {

    //FtcDashboard dashboard = FtcDashboard.getInstance();
    //drive motors

    public ElapsedTime currentTime = new ElapsedTime();

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    double[] timeArray = new double[20];

    boolean[] timerInitted = new boolean[20];
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    public DcMotor intake = null;
    public DcMotor R_shooter = null;
    public DcMotor L_shooter = null;
    public DcMotor transfer = null;

    public boolean waiting = false;
    public double shooterP = 0.00255;

    public double turretP = 0.01;

    public Servo L_swingythingy = null;
    public Servo R_swingythingy = null;

    public CRServo L_turret = null;
    public CRServo R_turret = null;

    public Servo L_PTO = null;
    public Servo R_PTO = null;

    public CRServo L_feeder = null;
    public CRServo R_feeder = null;

    public Servo hood = null;

    public boolean shooterOn = false;
    public int previousShooterEncoder = 0;
    private int prevShooterRPM = 0;

    public int error;

    public int angleError;
    ElapsedTime timerS = new ElapsedTime();
    private static final double MIN_SAMPLE_TIME = 0.01;

    public final double L_swingy_Thingy_Open = 0;
    public final double R_swingy_Thingy_Open = 1;

    public final double L_swingy_Thingy_Close = 0.15;
    public final double R_swingy_Thingy_Close = 0.85;

    public int targetRPM = 0;

    public double hood_pose = 1;

    private double turretDegreeRatio = 0.003703703704;
    public DcMotor turretEncoder = null;
    public double cameraDifferenceAngle = 0 ;
    public double turretEncoderCounts = 0;
    public double turretAngle = 0;
    private double turretCenterPosition = 147.3;


    public Gen2Hardwaremap(HardwareMap ahwMap) {

        //drive motors
        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");

        intake = ahwMap.dcMotor.get("intake");
        transfer = ahwMap.dcMotor.get("transfer");

        R_shooter = ahwMap.dcMotor.get("R_shooter");
        L_shooter = ahwMap.dcMotor.get("L_shooter");

        L_feeder = ahwMap.crservo.get("L_feeder");
        R_feeder = ahwMap.crservo.get("R_feeder");

        L_swingythingy = ahwMap.servo.get("L_swingythingy");
        R_swingythingy = ahwMap.servo.get("R_swingythingy");

        hood = ahwMap.servo.get("hood");

        turretEncoder = intake;

        //drive motors and odometry encoders
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.REVERSE);

        L_shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRF.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);

        intake.setPower(0);
        R_shooter.setPower(0);
        L_shooter.setPower(0);
        transfer.setPower(0);
    }

    public void mecanumDrive(double forward, double strafe, double heading, double speed) {

        motorRF.setPower((((forward - strafe) * 1) - (heading * 1)) * speed);
        motorRB.setPower((((forward + strafe) * 1) - (heading * 1)) * speed);
        motorLB.setPower((((forward - strafe) * 1) + (heading * 1)) * speed);
        motorLF.setPower((((forward + strafe) * 1) + (heading * 1)) * speed);
    }

    public void turret(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        AprilTagDetection detection = currentDetections.get(0);
        turretEncoderCounts = turretEncoder.getCurrentPosition();
        turretAngle = ((turretEncoderCounts/360)*6.25); //6.25 is gear ratio

        if (detection.metadata != null) {
            cameraDifferenceAngle = detection.ftcPose.bearing;
        }
        else{
            cameraDifferenceAngle = 1;
        }

        if(turretAngle < -90 || turretAngle > 90){
            angleError = 0;
        }
        else{
            angleError = (int)(cameraDifferenceAngle - turretAngle);
        }
        double turretPower = angleError * turretP;

        R_turret.setPower(turretPower);
        L_turret.setPower(turretPower);

    }

    public void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }

    public boolean boolTimer (double time){
        return currentTime.milliseconds() > time;
    }

    public double timerInit(int t){

        double ti = currentTime.milliseconds() + t;
        //timerInitted = true;

        return ti;

    }

    public void intake (){

        targetRPM = 0;

        intake.setPower(1);

        L_feeder.setPower(-1);
        R_feeder.setPower(1);

        transfer.setPower(-1);

        L_swingythingy.setPosition(L_swingy_Thingy_Open);
        R_swingythingy.setPosition(R_swingy_Thingy_Open);

    }

    public void outTake (){

        targetRPM = 0;

        intake.setPower(-1);

        L_feeder.setPower(1);
        R_feeder.setPower(-1);

        transfer.setPower(1);

        L_swingythingy.setPosition(L_swingy_Thingy_Open);
        R_swingythingy.setPosition(R_swingy_Thingy_Open);

    }

    public void stopIntake(){

        targetRPM = 0;

        intake.setPower(0);

        L_feeder.setPower(0);
        R_feeder.setPower(0);

        transfer.setPower(0);

        L_swingythingy.setPosition(L_swingy_Thingy_Open);
        R_swingythingy.setPosition(R_swingy_Thingy_Open);
    }

    public void shoot (){

        if(!timerInitted[0]) {//very very first thing to happen
            timeArray[0] = currentTime.milliseconds();
            timerInitted[0] = true;
        }

        if (currentTime.milliseconds() > timeArray[0] + 1750) {//Last thing to happen

            L_feeder.setPower(0);
            R_feeder.setPower(0);

            transfer.setPower(0);

            timerInitted[0] = false;
        }

        else if (currentTime.milliseconds() > timeArray[0] + 500) {

            L_feeder.setPower(1);
            R_feeder.setPower(-1);

            transfer.setPower(1);
        }

        else {//Second thing to happen

            transfer.setPower(0);
            intake.setPower(-.75);

            L_swingythingy.setPosition(L_swingy_Thingy_Close);
            R_swingythingy.setPosition(R_swingy_Thingy_Close);

            hood.setPosition(hood_pose);

            targetRPM = 3500;
        }

        shooterOn = true;
    }

    public void autoShoot(){

        if(!timerInitted[7]) {//very very first thing to happen
            timeArray[7] = currentTime.milliseconds();
            timerInitted[7] = true;
        }

        if (currentTime.milliseconds() > timeArray[7] + 2000) {//Last thing to happen

            timerInitted[7] = false;
        }

        else if (currentTime.milliseconds() > timeArray[7] + 500) {

            L_feeder.setPower(1);
            R_feeder.setPower(-1);

            transfer.setPower(1);
        }

        else {//Second thing to happen

            transfer.setPower(0);
            intake.setPower(-1);

            L_swingythingy.setPosition(L_swingy_Thingy_Close);
            R_swingythingy.setPosition(R_swingy_Thingy_Close);

            hood.setPosition(hood_pose);

            targetRPM = 3500;
        }

        shooterOn = true;
    }

    public void hoodUp(){

        targetRPM = 3500;//Should be 4500
        hood_pose = .5;

        if(!timerInitted[1]) {//very very first thing
            timeArray[1] = currentTime.milliseconds();
            timerInitted[1] = true;
        }

        while(hood.getPosition() > 0.5){

            hood.setPosition(hood.getPosition() - 0.01);

            if (currentTime.milliseconds() > timeArray[1] + 200) {//last thing to happen

                timerInitted[1] = false;
            }
        }
    }

    public void hoodMid(){

        if(!timerInitted[2]) {//very very first thing
            timeArray[2] = currentTime.milliseconds();
            timerInitted[2] = true;
        }

        while(hood.getPosition() > 0.85){

            hood.setPosition(hood.getPosition() - 0.01);

            if (currentTime.milliseconds() > timeArray[2] + 200) {//last thing to happen

                timerInitted[2] = false;
            }
        }

        hood_pose = .85;
        targetRPM = 3500;//Should be 4000
    }

    public void hoodDown(){

        hood.setPosition(1);

        hood_pose = 1;
        targetRPM = 3500;
    }

    public int shooterRPM() {

        if (shooterOn) {

            // If our min sample time has not passed yet, return the previous shooter RPM
            if (timerS.seconds() < MIN_SAMPLE_TIME) {
                return prevShooterRPM;
            }
            else {
                int countsPerSecond = (int) ((R_shooter.getCurrentPosition() - previousShooterEncoder) / timerS.seconds());
                previousShooterEncoder = R_shooter.getCurrentPosition();
                timerS.reset();

                prevShooterRPM = countsPerSecond * 60 / 28;

                return prevShooterRPM;
            }
        }

        prevShooterRPM = 0;
        return 0;

    }

    public double setting_ShooterRPM(){

        error = targetRPM - shooterRPM();
        double power = error * shooterP;

        return power;
    }

}