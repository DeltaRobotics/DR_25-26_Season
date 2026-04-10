package org.firstinspires.ftc.teamcode.projects;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="strafer")
public class strafer extends LinearOpMode {
    DcMotorEx rf;
    DcMotorEx rb;
    DcMotorEx lf;
    DcMotorEx lb;

    public void mecanumDrive(double forward, double strafe, double heading, double speed) {
        rf.setPower((((forward - strafe) * 1) - (heading * 1)) * speed);
        rb.setPower((((forward + strafe) * 1) - (heading * 1)) * speed);
        lb.setPower((((forward - strafe) * 1) + (heading * 1)) * speed);
        lf.setPower((((forward + strafe) * 1) + (heading * 1)) * speed);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rf =hardwareMap.get(DcMotorEx.class, "motorRF");
        rb = hardwareMap.get(DcMotorEx.class, "motorRB");
        lf = hardwareMap.get(DcMotorEx.class, "motorLF");
        lb = hardwareMap.get(DcMotorEx.class, "motorLB");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 1);
        }
    }
}
