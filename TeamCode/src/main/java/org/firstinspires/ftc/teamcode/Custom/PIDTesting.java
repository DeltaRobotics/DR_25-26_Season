package org.firstinspires.ftc.teamcode.Custom;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="PIDTesting")
@Disabled

public class PIDTesting extends LinearOpMode
{

    private double shooterPower = 0;

    private int previousEncoderCounts = 0;

    private boolean firstRun = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        int encoderCount = 0;

        int countsPerSecond = 0;

        ElapsedTime elapsedTime = new ElapsedTime();

        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

        PIDController PIDShooter = new PIDController(1,0,0,0);

        waitForStart();

        while (opModeIsActive())
        {
            encoderCount = shooter.getCurrentPosition();

            if(firstRun){

                firstRun = false;
            }
            else{

                countsPerSecond = (int)((encoderCount - previousEncoderCounts) / elapsedTime.seconds());
                shooterPower = PIDShooter.Update(countsPerSecond);

            }

            previousEncoderCounts = encoderCount;

            shooter.setPower(shooterPower);

            elapsedTime.reset();

        }
    }
}