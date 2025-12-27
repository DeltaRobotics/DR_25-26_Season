package org.firstinspires.ftc.teamcode.Custom;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config //We need this for Dashboard to change variables
public class Gen2Hardwaremap {

    //FtcDashboard dashboard = FtcDashboard.getInstance();
    //drive motors

    public boolean timerInitted = false;

    public ElapsedTime currentTime = new ElapsedTime();

    double[] timeArray = new double[20];
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    public DcMotor intake = null;
    public DcMotor shooter = null;

    public Servo L_swingythingy = null;
    public Servo R_swingythingy = null;

    public CRServo L_turret = null;
    public CRServo R_turret = null;

    public Servo L_PTO = null;
    public Servo R_PTO = null;

    public CRServo L_feeder = null;
    public CRServo R_feeder = null;

    public CRServo L_transfer = null;
    public CRServo R_transfer = null;

    public Servo hood = null;

    public boolean shooterOn = false;
    public int preSEncoder = 0;
    private int prevShooterRPM = 0;
    ElapsedTime timerS = new ElapsedTime();
    private static final double MIN_SAMPLE_TIME = 0.01;

    public final double L_swingy_Thingy_Open = 0;
    public final double R_swingy_Thingy_Open = 1;

    public final double L_swingy_Thingy_Close = 0.15;
    public final double R_swingy_Thingy_Close = 0.85;


    public Gen2Hardwaremap(HardwareMap ahwMap) {

        //drive motors
        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");

        intake = ahwMap.dcMotor.get("intake");
        shooter = ahwMap.dcMotor.get("shooter");

        L_feeder = ahwMap.crservo.get("L_feeder");
        R_feeder = ahwMap.crservo.get("R_feeder");

        L_transfer = ahwMap.crservo.get("L_transfer");
        R_transfer = ahwMap.crservo.get("R_transfer");

        L_swingythingy = ahwMap.servo.get("L_swingythingy");
        R_swingythingy = ahwMap.servo.get("R_swingythingy");

        hood = ahwMap.servo.get("hood");

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
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRF.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);

        intake.setPower(0);
        shooter.setPower(0);
    }

    public void mecanumDrive(double forward, double strafe, double heading, double speed) {

        motorRF.setPower((((forward - strafe) * 1) - (heading * 1)) * speed);
        motorRB.setPower((((forward + strafe) * 1) - (heading * 1)) * speed);
        motorLB.setPower((((forward - strafe) * 1) + (heading * 1)) * speed);
        motorLF.setPower((((forward + strafe) * 1) + (heading * 1)) * speed);
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
        shooter.setPower(0);
        intake.setPower(1);

        L_feeder.setPower(-1);
        R_feeder.setPower(1);

        L_transfer.setPower(1);
        R_transfer.setPower(-1);

        L_swingythingy.setPosition(L_swingy_Thingy_Open);
        R_swingythingy.setPosition(R_swingy_Thingy_Open);

    }

    public void outTake (){

        shooter.setPower(0);
        intake.setPower(-1);

        L_feeder.setPower(1);
        R_feeder.setPower(-1);

        L_transfer.setPower(-1);
        R_transfer.setPower(1);

        L_swingythingy.setPosition(L_swingy_Thingy_Open);
        R_swingythingy.setPosition(R_swingy_Thingy_Open);

    }

    public void stopIntake(){

        shooter.setPower(0);
        intake.setPower(0);

        L_feeder.setPower(0);
        R_feeder.setPower(0);

        L_transfer.setPower(0);
        R_transfer.setPower(0);

        L_swingythingy.setPosition(L_swingy_Thingy_Open);
        R_swingythingy.setPosition(R_swingy_Thingy_Open);
    }

    public void shoot (){

        if(!timerInitted) {
            timeArray[0] = currentTime.milliseconds();
            timerInitted = true;
        }

        if (currentTime.milliseconds() > timeArray[0] + 1000) {

            L_feeder.setPower(1);
            R_feeder.setPower(-1);
            L_transfer.setPower(-1);
            R_transfer.setPower(1);

            timerInitted = false;
        }
        else {//first thing to happen

            intake.setPower(-1);
            L_swingythingy.setPosition(L_swingy_Thingy_Close);
            R_swingythingy.setPosition(R_swingy_Thingy_Close);
            shooter.setPower(1);
        }

        shooterOn = true;

    }

    public int shooterRPM() {

        if (shooterOn) {

            // If our min sample time has not passed yet, return the previous shooter RPM
            if (timerS.seconds() < MIN_SAMPLE_TIME) {
                return prevShooterRPM;
            } else {
                int countsPerSecond = (int) ((shooter.getCurrentPosition() - preSEncoder) / timerS.seconds());
                preSEncoder = shooter.getCurrentPosition();
                timerS.reset();

                prevShooterRPM = countsPerSecond * 60 / 28;

                return prevShooterRPM;
            }
        }

        prevShooterRPM = 0;
        return 0;

    }

    public void hoodUp(){

        hood.setPosition(.3);
    }

    public void hoodDown(){

        hood.setPosition(1);
    }

}