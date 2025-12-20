package org.firstinspires.ftc.teamcode.Custom;

//two face teleop
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Gen2Teleop")
//@Disabled

public class Gen2Teleop extends LinearOpMode {
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


    double time = 0;
    public boolean timerInitted = false;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Gen2Hardwaremap robot = new Gen2Hardwaremap(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, .8);

            if(gamepad1.left_bumper && buttonLB){

                robot.intake();

                buttonLB = false;

            }

            if(!gamepad1.left_bumper && !buttonLB){

                buttonLB = true;

            }

            if(gamepad1.left_trigger > 0.5 && buttonLT ){

                robot.outTake();

                buttonLT = false;

            }

            if(gamepad1.left_trigger < 0.5 && !buttonLT){

                buttonLT = true;

            }

            if(gamepad1.x && buttonX){

                robot.L_swingythingy.setPosition( + 0.01);
                robot.R_swingythingy.setPosition( - 0.01);

                buttonX = false;

            }

            if(!gamepad1.x && !buttonX){

                buttonX = true;

            }

            if(gamepad1.y && buttonY){

                robot.L_swingythingy.setPosition( - 0.01);
                robot.R_swingythingy.setPosition( + 0.01);

                buttonY = false;

            }

            if(!gamepad1.y && !buttonY){

                buttonY = true;

            }
        }
    }
}