package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double P;

    private double I;

    private double D;

    private double setPoint;

    private ElapsedTime elapsedTime;

    private double totalError;

    private double previousError;

    private boolean firstRun = true;

    public PIDController (double P, double I, double D, double setPoint){

        this.P = P;
        this.I = I;
        this.D = D;
        this.setPoint = setPoint;

        elapsedTime = new ElapsedTime();
        totalError = 0;
        previousError = 0;

    }
    public double Update (double sensorReading){

        double deltaTime = elapsedTime.seconds();

        double error = setPoint - sensorReading;

        totalError += error * deltaTime;

        double controlValue = 0;

        if(firstRun){

            controlValue = P * error + I * totalError;

            firstRun = false;
        }
        else{

            controlValue = P * error + I * totalError + D * ((error - previousError) / deltaTime);

        }

        previousError = error;

        return controlValue;

    }
}
