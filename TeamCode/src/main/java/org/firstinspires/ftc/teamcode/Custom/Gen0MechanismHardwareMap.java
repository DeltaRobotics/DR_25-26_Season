package org.firstinspires.ftc.teamcode.Custom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Gen0MechanismHardwareMap {

    public DcMotor intake = null;
    public DcMotor shooter = null;
    public Servo kicker = null;

    public final double INTAKE_ON_POWER = 0.75;
    public final double SHOOTER_IDLE_POWER = 0.45;
    public final double SHOOTER_FULL_POWER = 0.80;
    public int preSEncoder = 0;
    double timeS = 0;
    public boolean timerInittedS = false;

    public boolean onOff = false;

    ElapsedTime timerS = new ElapsedTime();

    public Gen0MechanismHardwareMap(HardwareMap ahwMap) {

        intake  = ahwMap.dcMotor.get("intake");
        shooter = ahwMap.dcMotor.get("shooter");

        kicker = ahwMap.servo.get("kicker");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setPower(0);
        intake.setPower(0);

    }

    public int shooterRPM(){
       if(onOff) {
           int countsPerSecond = (int)((shooter.getCurrentPosition() - preSEncoder) / timerS.seconds());
           preSEncoder = shooter.getCurrentPosition();
           timerS.reset();
           return countsPerSecond * 60 / 28;
       }
      return 0;

    }

    public void shooterON(){
        timerS.reset();
        preSEncoder = shooter.getCurrentPosition();
        shooter.setPower(0.65);
        onOff = true;

    }

    public void shooterOFF(){
        shooter.setPower(0);
        onOff = false;

    }







}
