package org.firstinspires.ftc.teamcode.Random;

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

    private double minSampleTime;

    public PIDController (double P, double I, double D, double setPoint, double minSampleTime){

        this.P = P;
        this.I = I;
        this.D = D;
        this.setPoint = setPoint;
        this.minSampleTime = minSampleTime;

        elapsedTime = new ElapsedTime();
        totalError = 0;
        previousError = 0;

    }
    public double Update (double sensorReading){

        double error = setPoint - sensorReading;

        double controlValue = 0;

        if(firstRun){

            firstRun = false;
        }
        else{
            double deltaTime = elapsedTime.seconds();

            totalError += error * deltaTime;

            controlValue = P * error + I * totalError + D * ((error - previousError) / deltaTime);

        }

        elapsedTime.reset();
        previousError = error;

        return controlValue;

    }

    // Getters and setters
    public double getP()
    {
        return P;
    }

    public double getI()
    {
        return I;
    }

    public double getD()
    {
        return D;
    }

    public double getSetPoint()
    {
        return setPoint;
    }

    public double getTotalError()
    {
        return totalError;
    }

    public void setP(double P)
    {
        this.P = P;
    }

    public void setI(double I)
    {
        this.I = I;
    }

    public void setD(double D)
    {
        this.D = D;
    }

    public void setSetPoint(double setPoint)
    {
        this.setPoint = setPoint;
    }
}
