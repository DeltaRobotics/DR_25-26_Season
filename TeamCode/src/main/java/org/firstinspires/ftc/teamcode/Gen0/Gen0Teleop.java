package org.firstinspires.ftc.teamcode.Gen0;

//two face teleop
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Gen0Teleop")
@Disabled

public class Gen0Teleop extends LinearOpMode {
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
        Gen0Hardwaremap robot = new Gen0Hardwaremap(hardwareMap);
        Gen0MechanismHardwareMap mechanism = new Gen0MechanismHardwareMap(hardwareMap);

        waitForStart();



        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 1);

            if (gamepad2.dpad_down && buttonDD) {

                mechanism.shooterOFF();

                buttonDD = false;
            }

            if (!gamepad2.dpad_down && !buttonDD) {

                buttonDD = true;
            }

            if (gamepad1.right_bumper && buttonRB || timerInitted) {

                if (mechanism.kicker.getPosition() == 0) {

                    mechanism.kickerUP();
                    mechanism.intakeOFF();
                    time = timer.milliseconds() + 250;
                    timerInitted=true;

                }
                if (time < timer.milliseconds() && timerInitted){
                    mechanism.kickerDOWN();
                    time = timer.milliseconds() + 400;
                    mechanism.intakeON();
                    timerInitted=false;
                }


                buttonRB = false;
            }
            if (!gamepad1.right_bumper && !buttonRB) {

                buttonRB = true;
            }

            if (gamepad1.b && buttonB) {

                buttonB = false;

                mechanism.intakeREVERSE();
            }
            else if (!gamepad1.b) {
                buttonB = true;
            }

            if (gamepad1.x && buttonX) {

                buttonX = false;

                if (mechanism.intake.getPower() == 0) {
                    mechanism.intakeON();
                }
                else {
                    mechanism.intakeOFF();
                }
            }
            else if (!gamepad1.x) {
                buttonX = true;
            }

            if (gamepad1.b && buttonB) {

                buttonB = false;

                mechanism.intakeREVERSE();
            }
            else if (!gamepad1.b) {
                buttonB = true;
            }

            //add shooter shutdown button later

            if (gamepad1.left_bumper && buttonLB) {

                buttonLB = false;

                if (!mechanism.shooterOn) {
                    mechanism.shooterON();
                }
                else {
                    mechanism.shooterOFF();
                }
            }
            else if (!gamepad1.left_bumper) {
                buttonLB = true;
            }

            telemetry.addData("shooterRPM", mechanism.shooterRPM());
            telemetry.addData("kickerPos", mechanism.kicker.getPosition());
            telemetry.update();


        }
    }
}