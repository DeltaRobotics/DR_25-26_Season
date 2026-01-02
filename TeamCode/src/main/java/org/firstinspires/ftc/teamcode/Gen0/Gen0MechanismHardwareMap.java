package org.firstinspires.ftc.teamcode.Gen0;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Gen0MechanismHardwareMap {

    public DcMotor intake = null;
    public DcMotor shooter = null;
    public Servo kicker = null;

    // Constants
    private static final double DEFAULT_INTAKE_POWER = 1;
    private static final double DEFAULT_SHOOTER_POWER = 0.65;

    // In seconds.

    private static final double MIN_SAMPLE_TIME = 0.01;

    // Instance variables.

    public int preSEncoder = 0;
    private int prevShooterRPM = 0;
    double timeS = 0;
    public boolean timerInittedS = false;

    public boolean shooterOn = false;

    ElapsedTime timerS = new ElapsedTime();

    public Gen0MechanismHardwareMap(HardwareMap ahwMap) {

        intake = ahwMap.dcMotor.get("intake");
        shooter = ahwMap.dcMotor.get("shooter");

        kicker = ahwMap.servo.get("kicker");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setPower(0);
        intake.setPower(0);

        kickerDOWN();

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

    public void shooterON() {
        shooterON(DEFAULT_SHOOTER_POWER);

    }

    public void shooterON(double power) {
        timerS.reset();
        preSEncoder = shooter.getCurrentPosition();
        shooter.setPower(power);
        shooterOn = true;

    }

    public void shooterOFF() {
        shooter.setPower(0);
        shooterOn = false;

    }


    public void intakeON() {

        intake.setPower(DEFAULT_INTAKE_POWER);
    }

    public void intakeOFF() {

        intake.setPower(0);
    }

    public void intakeREVERSE() {

        intake.setPower(intake.getPower() * -1);

    }

    public void kickerUP() {

        if (shooterOn) {
            kicker.setPosition(.6);
        }

    }

    public void kickerDOWN() {

        kicker.setPosition(0);

    }


}
