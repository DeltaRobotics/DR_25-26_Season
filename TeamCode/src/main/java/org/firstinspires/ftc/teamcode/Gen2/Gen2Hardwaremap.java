package org.firstinspires.ftc.teamcode.Gen2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Random.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import java.util.List;


//@Config //We need this for Dashboard to change variables
public class Gen2Hardwaremap {

    //FtcDashboard dashboard = FtcDashboard.getInstance();
    //drive motors

    public ElapsedTime currentTime = new ElapsedTime();

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private final static int LED_PERIOD = 10;
    public AprilTagProcessor aprilTag;
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
    public double shooterP = 0.001;
    public double turretP = 0.025;

    public Servo L_swingythingy = null;
    public Servo R_swingythingy = null;

    public CRServo L_turret = null;
    public CRServo R_turret = null;

    public Servo L_PTO = null;
    public Servo R_PTO = null;

    public CRServo L_feeder = null;
    public CRServo R_feeder = null;

    public Servo hood = null;

    public int previousShooterEncoder = 0;
    private int prevShooterRPM = 0;

    public int error;

    public double angleError;
    ElapsedTime timerS = new ElapsedTime();

    //private final static int LED_PERIOD = 10;
    private static final double MIN_SAMPLE_TIME = 0.01;

    public final double L_swingy_Thingy_Open = 0;
    public final double R_swingy_Thingy_Open = 1;

    public final double L_swingy_Thingy_Close = 0.15;
    public final double R_swingy_Thingy_Close = 0.85;

    public final double L_PTO_DOWN = 0.5;
    public final double R_PTO_DOWN = 0.5;

    public final double L_PTO_UP = 0.2;
    public final double R_PTO_UP = 0.6;

    public double targetRPM = 0;

    public double hood_pos = 1;

    private double turretDegreeRatio = 0.003703703704;
    public DcMotor turretEncoder = null;
    public double turretEncoderCounts = 0;
    public double turretAngle = 0;
    public double newTurretP = 0;
    private double turretCenterPosition = 0; //147.3
    public boolean blue = false;
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    //SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;
    public Limelight3A limelight;
    PIDController PIDShooter;

    PIDController PIDTurret;



    public Gen2Hardwaremap(HardwareMap ahwMap) {

        //displayKind = DisplayKind.AUTO;

        blinkinLedDriver = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");
        //pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        //blinkinLedDriver.setPattern(pattern);

        //display = telemetry.addData("Display Kind: ", displayKind.toString());
        //patternName = telemetry.addData("Pattern: ", pattern.toString());

        //ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        //gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        limelight = ahwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();

        //PIDShooter = new PIDController(0.0015,0,0,0, MIN_SAMPLE_TIME * 2,0.1,1);
        PIDShooter = new PIDController(0.003,0,0.00001,0, MIN_SAMPLE_TIME * 2,0.1,1);

        PIDTurret = new PIDController(0.02,0,0.0000008,0, MIN_SAMPLE_TIME * 2,-1,1);

        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");

        intake = ahwMap.dcMotor.get("intake");
        transfer = ahwMap.dcMotor.get("transfer");

        L_turret = ahwMap.crservo.get("L_turret");
        R_turret = ahwMap.crservo.get("R_turret");

        L_shooter = ahwMap.dcMotor.get("L_shooter");
        R_shooter = ahwMap.dcMotor.get("R_shooter");

        L_feeder = ahwMap.crservo.get("L_feeder");
        R_feeder = ahwMap.crservo.get("R_feeder");

        L_PTO = ahwMap.servo.get("L_PTO");
        R_PTO = ahwMap.servo.get("R_PTO");

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

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        //R_shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        //L_shooter.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void turret(Telemetry telemetry){

        LLResult result = limelight.getLatestResult();
        double angle = result.getTx();
        double targetOffsetAngle_Vertical = angle;

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 12.56021; //10.262226

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 16.0;

        // distance from the target to the floor
        double goalHeightInches = 29.5;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        double range = distance - 7;

        int id;

        if(blue){
            id = 20;
        }
        else{
            id = 24;
        }

        if (range > 120){
            targetRPM = 5000;
            hood_pos = 0.6;
        }
        else{
            targetRPM = 2826 + (11.3 * range) + 0.0236 * (range * range);
            hood_pos = 1.05 + (0.000152 * range) - 0.000087 * (range * range) + 0.00000045 * (range * range * range);
        }

        hood.setPosition(hood_pos);

        //Auto-aiming

        turretEncoderCounts = turretEncoder.getCurrentPosition();

        angleError = result.getTx();
        double turretPower = angleError * turretP; // was -angleError
        //turretAngle = (((turretEncoderCounts / 8192) * 360)* 6.25); //I think this math is wrong
        //turretAngle = ((turretEncoderCounts / 360) * 6.25); //6.25 is gear ratio

        //if (result.getTx() == 0){
        //   turretAngle = turretCenterPosition;
        //}
        //else{
        //    turretAngle = ((turretEncoderCounts / 360) * 6.25); //6.25 is gear ratio
//
        //}


        //safety check so the turret doesn't go past 90
        if((turretEncoderCounts < -2048 && turretPower < 0) || (turretEncoderCounts > 2048 && turretPower > 0)){
            turretPower = 0;
        }

        R_turret.setPower(turretPower);
        //L_turret.setPower(turretPower);

    }

    public void initAprilTag(HardwareMap ahwMap) {
/*
         //Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

         //Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(ahwMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(aprilTag)
                    //.enableLiveView(true)
                    .build();
        }
        else{
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(aprilTag)
                    //.enableLiveView(true)
                    .build();

        }



 */

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

        //targetRPM = 300;

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

    public void autoShoot(){

        if(!timerInitted[0]) {//very very first thing to happen
            timeArray[0] = currentTime.milliseconds();
            timerInitted[0] = true;
        }

        if (currentTime.milliseconds() > timeArray[0] + 500) {//Last thing to happen

            L_feeder.setPower(0);
            R_feeder.setPower(0);

            transfer.setPower(0);

            timerInitted[0] = false;
        }

        else if (currentTime.milliseconds() > timeArray[0] + 250) {

            L_feeder.setPower(1);
            R_feeder.setPower(-1);

            transfer.setPower(1);
        }

        else {//Second thing to happen

            transfer.setPower(0);
            intake.setPower(-.75);

            L_swingythingy.setPosition(L_swingy_Thingy_Close);
            R_swingythingy.setPosition(R_swingy_Thingy_Close);

            hood.setPosition(hood_pos);

            R_shooter.setPower(setting_ShooterRPM());
            L_shooter.setPower(setting_ShooterRPM());
        }
    }

    public void teleOpShoot(){

        transfer.setPower(0);
        intake.setPower(-.75);

        L_swingythingy.setPosition(L_swingy_Thingy_Close);
        R_swingythingy.setPosition(R_swingy_Thingy_Close);

        hood.setPosition(hood_pos);

        R_shooter.setPower(setting_ShooterRPM());
        L_shooter.setPower(setting_ShooterRPM());


    }

    public void hoodUp(){
        hood.setPosition(hood_pos);
    }

    public void hoodMid(){
        hood.setPosition(hood_pos);
    }

    public void hoodDown(){
        hood.setPosition(hood_pos);
    }

    public int shooterRPM(){

        if (R_shooter.getPower() != 0) {

            // If our min sample time has not passed yet, return the previous shooter RPM
            if (timerS.seconds() < MIN_SAMPLE_TIME) {
                return prevShooterRPM;
            }
            else {
                int countsPerSecond = (int) ((Math.abs(R_shooter.getCurrentPosition()) - previousShooterEncoder) / timerS.seconds());
                previousShooterEncoder = Math.abs(R_shooter.getCurrentPosition());
                timerS.reset();

                prevShooterRPM = countsPerSecond * 60 / 28;

                return prevShooterRPM;
            }
        }

        prevShooterRPM = 0;
        return 0;

    }

    public double setting_ShooterRPM(){

        PIDShooter.setSetPoint(targetRPM);

        return PIDShooter.Update(shooterRPM());
    }

    public double setting_ShooterRPM(int con){

        PIDShooter.setSetPoint(targetRPM);

        return PIDShooter.Update(shooterRPM());
    }

    public void display_state_shooting(){
        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
        blinkinLedDriver.setPattern(pattern);
    }
    public void display_state_intaking(){
        pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
        blinkinLedDriver.setPattern(pattern);
    }
    public void display_state_outputting(){
        pattern = RevBlinkinLedDriver.BlinkinPattern.GRAY;
        blinkinLedDriver.setPattern(pattern);
    }
    public void display_state_liftActivated(){
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER;
        blinkinLedDriver.setPattern(pattern);
    }
    public void display_state_Lifting(){
        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
        blinkinLedDriver.setPattern(pattern);
    }
    public void display_state_hopperFull(){
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkinLedDriver.setPattern(pattern);
    }
    public void display_state_BlueIdle(){
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        blinkinLedDriver.setPattern(pattern);
    }
    public void display_state_RedIdle(){
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver.setPattern(pattern);
    }

    public void set_turretP(){
        turretP = newTurretP;
    }


}