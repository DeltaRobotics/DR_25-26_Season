package org.firstinspires.ftc.teamcode.Custom;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Gen0Teleop")
//@Disabled

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
    private double intakePower = 0;

    private final double INTAKE_ON_POWER = 0.75;
    private final double SHOOTER_IDLE_POWER = 0.45;
    private final double SHOOTER_FULL_POWER = 0.80;


    public Servo kicker = null;
    public DcMotor intake = null;
    public DcMotor shooter = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Gen0Hardwaremap robot = new Gen0Hardwaremap(hardwareMap);

        kicker = hardwareMap.servo.get("kicker");


        while (!isStarted() && !isStopRequested()) {

            kicker.setPosition(0);

            //intake.setTargetPosition(0);
            //intake.setPower(.2);
            //intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //shooter.setTargetPosition(0);
            //shooter.setPower(.2);
            //shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, .75);

            if (gamepad1.dpad_up && buttonDU) {

                kicker.setPosition(kicker.getPosition() + 0.1);

                buttonDU = false;
            }

            if (!gamepad1.dpad_up && !buttonDU) {

                buttonDU = true;
            }

            if (gamepad1.right_bumper && buttonRB) {

                buttonRB = false;

                robot.intake.setPower(robot.intake.getPower() * -1);

            } else if (!gamepad1.b) {
                buttonB = true;
            }

            if (gamepad1.x && buttonX) {

                buttonX = false;

                if (robot.intake.getPower() == 0) {
                    robot.intake.setPower(INTAKE_ON_POWER);
                } else {
                    robot.intake.setPower(0);
                }
            } else if (!gamepad1.x) {
                buttonX = true;
            }

            if (gamepad1.b && buttonB) {

                buttonB = false;

                robot.intake.setPower(robot.intake.getPower() * -1);

            } else if (!gamepad1.b) {
                buttonB = true;
            }

            //add shooter shutdown button later

            if (gamepad1.left_bumper && buttonLB) {

                buttonLB = false;

                if (robot.shooter.getPower() == SHOOTER_IDLE_POWER) {
                    robot.shooter.setPower(SHOOTER_FULL_POWER);
                } else {
                    robot.shooter.setPower(SHOOTER_IDLE_POWER);
                }
            } else if (!gamepad1.left_bumper) {
                buttonLB = true;
            }

        }
    }
}