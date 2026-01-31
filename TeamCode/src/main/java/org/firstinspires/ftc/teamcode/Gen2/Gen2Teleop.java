package org.firstinspires.ftc.teamcode.Gen2;

//two face teleop
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


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

    public boolean lifting = false;

    public AprilTagDetection Detection = null;



    @Override
    public void runOpMode() throws InterruptedException {
        Gen2Hardwaremap robot = new Gen2Hardwaremap(hardwareMap);

        robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
        robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);

        robot.transfer.setPower(0);

        robot.L_PTO.setPosition(robot.L_PTO_UP);
        robot.R_PTO.setPosition(robot.R_PTO_UP);

        robot.R_feeder.setPower(0);
        robot.L_feeder.setPower(0);

        robot.initAprilTag(hardwareMap);

        robot.hoodDown();

        waitForStart();

        while (opModeIsActive()) {

            if(!lifting) {
                robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 1);
            }

            robot.turret();

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

            if(gamepad1.b && buttonB){

                robot.stopIntake();

                buttonB = false;
            }

            if(!gamepad1.b && !buttonB){

                buttonB = true;

            }

            if(gamepad1.right_bumper){

                robot.shoot();

                buttonRB = false;

            }

            if(!gamepad1.right_bumper && !buttonRB){

                buttonRB = true;

            }

            if(robot.timerInitted[0]){

                robot.shoot();
            }

            robot.R_shooter.setPower(robot.setting_ShooterRPM(1));
            robot.L_shooter.setPower(robot.setting_ShooterRPM(1));


            /**
             *
             *
             *                  DRIVER 2
             *
             *
             */


            //shooting for driver 2
            if(gamepad2.right_trigger > 0.5 && button2RT){

                robot.shoot();
                button2RT = false;
            }

            if(gamepad2.right_trigger < 0.5 && !button2RT){

                button2RT = true;
            }

            //Lifting
            if(gamepad2.start && gamepad2.dpad_up && button2DU){

                lifting = true;

                if(!robot.timerInitted[15]) {//very very first thing to happen
                    robot.timeArray[15] = robot.currentTime.milliseconds();
                    robot.timerInitted[15] = true;
                }

                if (robot.currentTime.milliseconds() > robot.timeArray[15] + 350) {//Last thing to happen

                    button2DU = false;
                }

                else if (robot.currentTime.milliseconds() > robot.timeArray[15] + 250) {

                    robot.motorRF.setPower(-0.25);
                    robot.motorLF.setPower(-0.25);
                    robot.motorRB.setPower(-0.25);
                    robot.motorLB.setPower(-0.25);

                }

                else {//Second thing to happen

                    robot.L_PTO.setPosition(robot.L_PTO_DOWN);
                    robot.R_PTO.setPosition(robot.R_PTO_DOWN);
                }
            }

            if(lifting){
                robot.mecanumDrive(-gamepad2.right_stick_y, 0, 0, 1);
            }

            if(!gamepad2.start && !gamepad2.dpad_up && !button2DU){

                button2DU = true;
            }

            //calling to track the blue AprilTag
            if(gamepad2.x && button2X){
                robot.blue = true;

                button2X = false;
            }

            if(!gamepad2.x && !button2X){

                button2X = true;
            }

            //calling to track the red AprilTag
            if(gamepad2.b && button2B){
                robot.blue = false;

                button2B = false;
            }

            if(!gamepad2.b && !button2B){

                button2B = true;
            }



            //if(robot.aprilTag.getDetections().size() > 0){

            //    Detection = robot.aprilTag.getDetections().get(0);

            //    if(Detection != null){
            //        telemetry.addData("bearing", Detection.ftcPose.bearing);
            //    }
            //}


            telemetry.addData("L_PTO Pos ", robot.L_PTO.getPosition());
            telemetry.addData("R_PTO Pos ", robot.R_PTO.getPosition());

            telemetry.addData("shooter Position right ", robot.R_shooter.getCurrentPosition());
            telemetry.addData("shooterRPM ", robot.shooterRPM());
            telemetry.addData("power ", robot.setting_ShooterRPM(1));
            telemetry.addData("real shooter power ", robot.R_shooter.getPower());

            telemetry.addData("error ", robot.error);

            telemetry.addData("hood ", robot.hood.getPosition());
            telemetry.update();

        }
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.limelight.stop();
    }
}