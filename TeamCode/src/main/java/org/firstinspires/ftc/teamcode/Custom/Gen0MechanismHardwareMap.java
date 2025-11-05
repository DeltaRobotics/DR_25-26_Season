package org.firstinspires.ftc.teamcode.Custom;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Gen0MechanismHardwareMap {

    public DcMotor intake = null;
    public DcMotor shooter = null;
    public Servo kicker = null;

    public final double INTAKE_ON_POWER = 0.75;
    public final double SHOOTER_IDLE_POWER = 0.45;
    public final double SHOOTER_FULL_POWER = 0.80;

    public Gen0MechanismHardwareMap(HardwareMap ahwMap) {

        intake  = ahwMap.dcMotor.get("intake");
        shooter = ahwMap.dcMotor.get("shooter");

        kicker = ahwMap.servo.get("kicker");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setPower(0);
        intake.setPower(0);

    }
}
