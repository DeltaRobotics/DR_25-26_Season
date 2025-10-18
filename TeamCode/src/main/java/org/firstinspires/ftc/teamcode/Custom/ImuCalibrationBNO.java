package org.firstinspires.ftc.teamcode.Custom;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="ImuCalibrationBNO")
//@Disabled

public class ImuCalibrationBNO extends LinearOpMode
{

    private BNO055IMU imu = null;

    private BNO055IMU.Parameters parameters = null;

    @Override
    public void runOpMode() throws InterruptedException
    {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        parameters = new BNO055IMU.Parameters();

        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;

        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive())
        {

            telemetry.addData("rawCalibrationStatus = ", imu.getCalibrationStatus());
            telemetry.addData("accelerometerCalibrationStatus", imu.isAccelerometerCalibrated());
            telemetry.addData("gyroCalibrationStatus", imu.isGyroCalibrated());
            telemetry.update();

        }
    }
}
