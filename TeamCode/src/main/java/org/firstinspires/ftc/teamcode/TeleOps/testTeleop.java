package org.firstinspires.ftc.teamcode.TeleOps;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Hardware_Maps.BatbotHardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;


@TeleOp(name="testTeleop")
//@Disabled

public class testTeleop extends LinearOpMode
{
    private Follower follower;
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    public Servo LHE = null;
    public Servo RHE = null;
    public Servo wrist = null;
    public Servo claw = null;

    public boolean buttonLT = true;
    public boolean buttonRT = true;
    public boolean buttonLB = true;
    public boolean buttonRB = true;
    public boolean buttonA = true;
    public boolean buttonB = true;
    public boolean buttonX = true;
    public boolean buttonY = true;
    public boolean buttonDU = true;
    public boolean buttonDD = true;
    public boolean buttonDR = true;
    public boolean buttonDL = true;

    public double speed = .1;
    private final Pose startPose = new Pose(0,0,0);



    @Override
    public void runOpMode() throws InterruptedException {
        BatbotHardwareMap robot = new BatbotHardwareMap(hardwareMap);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();

        LHE = hardwareMap.servo.get("LHE");
        RHE = hardwareMap.servo.get("RHE");

        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        //small numbers are out
        LHE.setPosition(.905);
        RHE.setPosition(.905);

        //lower numbers go up
        wrist.setPosition(.65);

        //bigger numbers are open
        claw.setPosition(.6);

        waitForStart();

        follower.startTeleopDrive();



        while (opModeIsActive()) {

            follower.setTeleOpDrive(-gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x, true);
            follower.update();

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();

            /////////////////////////////////////////////////////////////////////////////////

            for (int i = 0; i < blocks.length; i++){

                if(blocks[i].id == 1) {
                    telemetry.addData("positionX", String.valueOf(blocks[i].x));
                    telemetry.addData("positionY", String.valueOf(blocks[i].y));


                }

            }
            telemetry.update();



            //pushing the slides out
            if(gamepad1.right_trigger > .5 && buttonRT){

                LHE.setPosition(robot.SLIDES_OUT);
                RHE.setPosition(robot.SLIDES_OUT);

                buttonRT = false;
            }

            if(gamepad1.right_trigger < .5 && !buttonRT){

                buttonRT = true;
            }

            // bringing the slides inside
             if(gamepad1.left_trigger > .5 && buttonLT){
                LHE.setPosition(robot.SLIDES_INSIDE);
                RHE.setPosition(robot.SLIDES_INSIDE);

                buttonLT = false;
             }
             if(gamepad1.left_trigger < .5 && !buttonLT){

                buttonLT = true;
            }

             //wrist up position
             if(gamepad1.y && buttonY){

                 wrist.setPosition(robot.WRIST_UP);

                 buttonY = false;

             }

            if(!gamepad1.y && !buttonY){

                buttonY = true;

            }

            //wrist down position
            if(gamepad1.a && buttonA){

                wrist.setPosition(robot.WRIST_DOWN);

                buttonA = false;

            }

            if(!gamepad1.a && !buttonA){

                buttonA = true;

            }

            //claw closed position
            if(gamepad1.left_bumper && buttonLB){

                claw.setPosition(robot.CLAW_CLOSE);

                buttonLB = false;

            }

            if(!gamepad1.left_bumper && !buttonLB){

                buttonLB = true;

            }

            //claw open position
            if(gamepad1.right_bumper && buttonRB){

                claw.setPosition(robot.CLAW_OPEN);

                buttonRB = false;

            }

            if(!gamepad1.right_bumper && !buttonRB){

                buttonRB = true;

            }


        }
    }
}
