package org.firstinspires.ftc.teamcode.Gen2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Random.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import java.util.List;


//@Config //We need this for Dashboard to change variables
public class Gen2Hardwaremap {

    public ElapsedTime currentTime = new ElapsedTime();

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

    public Servo L_swingythingy = null;
    public Servo R_swingythingy = null;

    public CRServo L_turret = null;
    public CRServo R_turret = null;

    public Servo L_PTO = null;
    public Servo R_PTO = null;

    public CRServo L_feeder = null;
    public CRServo R_feeder = null;

    public Servo hood = null;

    public Servo rail_cap = null;

    public int previousShooterEncoder = 0;
    private int prevShooterRPM = 0;
    public double angleError;
    ElapsedTime timerS = new ElapsedTime();
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

    private double turretCenterPosition = 0; //147.3
    public boolean blue = false;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public Limelight3A limelight;

    public GoBildaPinpointDriver pinpoint;

    private DistanceSensor sensorDistance;

    PIDController PIDShooter;

    PIDController PIDTurret;



    public Gen2Hardwaremap(HardwareMap ahwMap) {

        pinpoint = ahwMap.get(GoBildaPinpointDriver.class, "pinpoint");

        blinkinLedDriver = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");

        limelight = ahwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();


        // you can use this as a regular DistanceSensor.
        sensorDistance = ahwMap.get(DistanceSensor.class, "sensor_distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        //PIDShooter = new PIDController(0.0015,0,0,0, MIN_SAMPLE_TIME * 2,0.1,1);
        PIDShooter = new PIDController(0.0015,0,0.00001,0, MIN_SAMPLE_TIME * 2,0.1,1);
        //P was 0.003

        PIDTurret = new PIDController(0.045,0,0.0,0, MIN_SAMPLE_TIME * 2,-1,1);

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

        int id;
        double angle = 1;
        // Default to 0.
        angleError = 0;
        

        if(blue){
            id = 20;
        }
        else{
            id = 24;
        }


        if (result.getFiducialResults().isEmpty()) {
            if(blue){
                //angleError = pinpoint.getHeading(AngleUnit.DEGREES) + 40;
            }
            else{
                //angleError = pinpoint.getHeading(AngleUnit.DEGREES) + 130;
            }
            angleError = 0;
        }
        else {
            for(LLResultTypes.FiducialResult fidRes : result.getFiducialResults()) {
                 //Once we have found the correct ID, use its target Y degrees.
                if (fidRes.getFiducialId() == id) {
                    angleError = fidRes.getTargetXDegrees();
                    angle = fidRes.getTargetYDegrees();
                    break;
                } else {
                    angle = 1;
                    break;
                }

            }
        }

        
        //double angle = result.getTy();
        double targetOffsetAngle_Vertical = result.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = -1; //10.262226

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 15.75;

        // distance from the target to the floor
        double goalHeightInches = 29.5;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        double range = distance - 6;

        if (range > 110 && range < 200){
            targetRPM = 5000;
            hood_pos = 0.6;
        }
        else if(range > 200){
            targetRPM = 3500;
            hood_pos = 1;
        }
        else{
            targetRPM = 2826 + (11.3 * range) + ((0.0236 * (range * range)));
            hood_pos = 1.1 - (0.00324 * range) - (0.0000279 * (range * range)) + (0.000000138 * (range * range * range));
        }

        hood.setPosition(hood_pos);

        telemetry.addData("distance", distance);
        telemetry.addData("range", range);
        telemetry.addData("heading",pinpoint.getHeading(AngleUnit.DEGREES));

        //Auto-aiming

        turretEncoderCounts = turretEncoder.getCurrentPosition();

        double turretPower = PIDTurret.Update(angleError); // was -angleError

        //safety check so the turret doesn't go past 90
        if((turretEncoderCounts < -12800 && turretPower < 0) || (turretEncoderCounts > 12800 && turretPower > 0)){
            turretPower = 0;
        }

        R_turret.setPower(turretPower);
        L_turret.setPower(turretPower);

    }

    public void intake (){

        intake.setPower(1);

        L_feeder.setPower(-1);
        R_feeder.setPower(1);

        transfer.setPower(-1);

        L_swingythingy.setPosition(L_swingy_Thingy_Open);
        R_swingythingy.setPosition(R_swingy_Thingy_Open);

        telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

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

        L_swingythingy.setPosition(L_swingy_Thingy_Close);
        R_swingythingy.setPosition(R_swingy_Thingy_Close);

        intake.setPower(-.75);

        //hood.setPosition(hood_pos);

        R_shooter.setPower(setting_ShooterRPM());
        L_shooter.setPower(setting_ShooterRPM());

        transfer.setPower(1);

        L_feeder.setPower(1);
        R_feeder.setPower(-1);
    }

    public void teleOpShoot(){

        L_swingythingy.setPosition(L_swingy_Thingy_Close);
        R_swingythingy.setPosition(R_swingy_Thingy_Close);

        intake.setPower(-.75);

        //hood.setPosition(hood_pos);

        R_shooter.setPower(setting_ShooterRPM());
        L_shooter.setPower(setting_ShooterRPM());

        transfer.setPower(1);

        L_feeder.setPower(1);
        R_feeder.setPower(-1);
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
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
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

}