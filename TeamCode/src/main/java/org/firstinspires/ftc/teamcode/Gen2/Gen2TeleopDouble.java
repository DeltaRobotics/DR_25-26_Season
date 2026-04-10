package org.firstinspires.ftc.teamcode.Gen2;

//two face teleop
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name = "Gen2TeleopDouble")
//@Disabled

public class Gen2TeleopDouble extends LinearOpMode {
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

    public boolean turretBool = true;

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

        robot.R_turret.setPower(0);
        robot.L_turret.setPower(0);

        robot.hood.setPosition(1);

        robot.targetRPM = 1800;

        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 1);

            if(turretBool){
                robot.turret(telemetry); //Disable for range tuning (taking points)
            }
            else{
                robot.targetRPM = 0;
            }

            robot.R_shooter.setPower(robot.setting_ShooterRPM());
            robot.L_shooter.setPower(robot.setting_ShooterRPM());

            //shooting
            if(gamepad2.right_bumper && button2RB){

                while(gamepad2.right_bumper && button2RB){

                    robot.teleOpShoot();
                    robot.display_state_shooting();
                    robot.transfer.setPower(1);

                    robot.L_feeder.setPower(1);
                    robot.R_feeder.setPower(-1);

                    robot.turret(telemetry);
                    telemetry.update();
                }
                robot.transfer.setPower(0);

                robot.L_feeder.setPower(0);
                robot.R_feeder.setPower(0);

                button2RB = false;
            }

            if(!gamepad2.right_bumper && !button2RB){
                button2RB = true;
            }

            if(gamepad2.right_trigger > 0.5 && button2RT){

                robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
                robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);

                robot.transfer.setPower(-1);
                robot.L_feeder.setPower(-1);

                robot.R_feeder.setPower(1);
                robot.intake.setPower(-0.75);
            }

            if(gamepad2.right_trigger < 0.5 && !button2RT){
                button2RT = true;
            }


            //intaking
            if(gamepad1.left_bumper && buttonLB){

                robot.intake();
                buttonLB = false;
            }

            if(gamepad1.left_bumper && !buttonLB){

                buttonLB = true;
            }


            //Outtaking
            if(gamepad1.left_trigger > 0.5 && buttonLT ){

                robot.outTake();
                //turretBool = false;
                robot.display_state_outputting();

                buttonLT = false;
            }

            if(gamepad1.left_trigger < 0.5 && !buttonLT){

                buttonLT = true;

            }

            /**
             **************************************************
             *                  DRIVER 2
             **************************************************
             */

            //Lifting

            if(gamepad2.a && buttonA){

                robot.farShoot = false;

                buttonA = false;
            }

            if(!gamepad2.a && !buttonA){
                buttonA = true;
            }


            //hood moving down
            if(gamepad2.y && buttonY){

                robot.farShoot = true;

                buttonY = false;

            }

            if(!gamepad2.y && !buttonY){
                buttonY = true;
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

            LLResult result = robot.limelight.getLatestResult();
            Pose3D botpose = result.getBotpose();

            /**

             telemetry.addData("tx", result.getTx());
             telemetry.addData("txnc", result.getTxNC());
             telemetry.addData("ty", result.getTy());
             telemetry.addData("tync", result.getTyNC());

             telemetry.addData("Botpose", botpose.toString());

             // Access classifier results
             List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
             for (LLResultTypes.ClassifierResult cr : classifierResults) {
             telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
             }

             // Access fiducial results
             List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
             for (LLResultTypes.FiducialResult fr : fiducialResults) {
             telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
             }

             // Access color results
             List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
             for (LLResultTypes.ColorResult cr : colorResults) {
             telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
             }
             */

            /**

             telemetry.addData("heading", robot.pinpoint.getHeading(AngleUnit.DEGREES));

             telemetry.addData("L_PTO Pos ", robot.L_PTO.getPosition());
             telemetry.addData("R_PTO Pos ", robot.R_PTO.getPosition());

             telemetry.addData("error ", robot.PIDShooter.getPreviousError());

             telemetry.addData("shooter Position right ", robot.R_shooter.getCurrentPosition());

             telemetry.addData("hood ", robot.hood.getPosition());

             telemetry.addData("hood pos ", robot.hood_pos);

             telemetry.addData("Right Turret Power", robot.R_turret.getPower());

             //telemetry.addData("ty", robot.limelight.getLatestResult().getFiducialResults().);

             //telemetry.addData("ty", robot.limelight.getLatestResult().getFiducialResults().isEmpty() ? "No Target" : robot.limelight.getLatestResult().getFiducialResults().get(0).getTargetYDegrees());

             telemetry.addData("turret Encoder", robot.turretEncoderCounts);

             */

            //telemetry.addData("turret P", String.valueOf(robot.PIDTurret.getP()));

            //telemetry.addData("turret D", String.valueOf(robot.PIDTurret.getD()));

            //telemetry.addData("turret I", String.valueOf(robot.PIDTurret.getI()));

            //telemetry.addData("shooterRPM ", robot.shooterRPM());
            //telemetry.addData("targetRPM ", robot.targetRPM);

            //telemetry.addData("power ", robot.setting_ShooterRPM());
            //telemetry.addData("real shooter power ", robot.R_shooter.getPower());

            //telemetry.addData("Shooter P", robot.PIDShooter.getP());

            //telemetry.update();

        }
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pinpoint.resetPosAndIMU();
        //robot.limelight.stop();
    }
}