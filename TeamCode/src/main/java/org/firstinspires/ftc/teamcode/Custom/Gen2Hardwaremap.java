package org.firstinspires.ftc.teamcode.Custom;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@Config //We need this for Dashboard to change variables
public class Gen2Hardwaremap {

    //FtcDashboard dashboard = FtcDashboard.getInstance();
    //drive motors
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    public DcMotor intake = null;

    public Servo L_swingythingy = null;
    public Servo R_swingythingy = null;

    public CRServo L_feeder = null;
    public CRServo R_feeder = null;

    public CRServo L_transfer = null;
    public CRServo R_transfer = null;




    public Gen2Hardwaremap(HardwareMap ahwMap) {

        //drive motors
        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");
        intake = ahwMap.dcMotor.get("intake");

        L_feeder = ahwMap.crservo.get("L_feeder");
        R_feeder = ahwMap.crservo.get("R_feeder");

        L_transfer = ahwMap.crservo.get("L_transfer");
        R_transfer = ahwMap.crservo.get("R_transfer");

        L_swingythingy = ahwMap.servo.get("L_swingythingy");
        R_swingythingy = ahwMap.servo.get("R_swingythingy");

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

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRF.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);

        intake.setPower(0);
    }

    public void mecanumDrive(double forward, double strafe, double heading, double speed) {

        motorRF.setPower((((forward - strafe) * 1) - (heading * 1)) * speed);
        motorRB.setPower((((forward + strafe) * 1) - (heading * 1)) * speed);
        motorLB.setPower((((forward - strafe) * 1) + (heading * 1)) * speed);
        motorLF.setPower((((forward + strafe) * 1) + (heading * 1)) * speed);
    }

    public void intake (){
        intake.setPower(1);

        L_feeder.setPower(-1);
        R_feeder.setPower(1);

        L_transfer.setPower(1);
        R_transfer.setPower(-1);

        L_swingythingy.setPosition(0);
        R_swingythingy.setPosition(1);

    }

    public void outTake (){
        intake.setPower(-1);

        L_feeder.setPower(1);
        R_feeder.setPower(-1);

        L_transfer.setPower(-1);
        R_transfer.setPower(1);

        L_swingythingy.setPosition(0);
        R_swingythingy.setPosition(1);

    }
    public void shoot (){

        intake.setPower(-1);

        L_feeder.setPower(1);
        R_feeder.setPower(-1);

        L_transfer.setPower(-1);
        R_transfer.setPower(1);

        //L_swingythingy.setPosition();
        //R_swingythingy.setPosition();

    }

}