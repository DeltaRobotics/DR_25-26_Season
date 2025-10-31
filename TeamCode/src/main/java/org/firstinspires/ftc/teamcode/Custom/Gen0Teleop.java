package org.firstinspires.ftc.teamcode.Custom;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Gen0Teleop")
//@Disabled

public class Gen0Teleop extends LinearOpMode
{
    public boolean buttonY = true;
    public boolean buttonX = true;
    public boolean buttonA = true;
    public boolean buttonB = true;
    public boolean buttonLB = true;
    public boolean buttonLT = true;
    public boolean buttonRB = true;
    public boolean buttonRT = true;
    public boolean buttonDR = true;
    public boolean buttonDL = true;
    public boolean buttonDU = true;
    public boolean buttonDD = true;

    public boolean button2Y = true;
    public boolean button2X = true;
    public boolean button2A = true;
    public boolean button2B = true;
    public boolean button2LB = true;
    public boolean button2LT = true;
    public boolean button2RB = true;
    public boolean button2RT = true;
    public boolean button2DR = true;
    public boolean button2DL = true;
    public boolean button2DU = true;
    public boolean button2DD = true;


    public Servo kicker = null;
    public DcMotor intake = null;
    public DcMotor shooter = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Gen0Hardwaremap robot = new Gen0Hardwaremap(hardwareMap);

        kicker = hardwareMap.servo.get("kicker");

        intake = hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStarted() && !isStopRequested()) {

            //kicker.setPosition(0);

            //intake.setTargetPosition(0);
            //intake.setPower(.2);
            //intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //shooter.setTargetPosition(0);
            //shooter.setPower(.2);
            //shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        while (opModeIsActive())
        {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, .25);

            if(gamepad1.dpad_up && buttonDU){

                kicker.setPosition( +.1);

                buttonDU = false;
            }

            if(!gamepad1.dpad_up && !buttonDU){

                buttonDU = true;
            }
            if(gamepad1.dpad_down && buttonDD){

                kicker.setPosition( -.1);

                buttonDD = false;

            }
            if(!gamepad1.dpad_down && !buttonDD){

                buttonDD = true;
            }


        }
    }
}
