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

    private double min = 0;

    private double max = 0;

    private double previousControlValue = 0;

    public PIDController (double P, double I, double D, double setPoint, double minSampleTime, double min, double max){

        this.P = P;
        this.I = I;
        this.D = D;
        this.setPoint = setPoint;
        this.minSampleTime = minSampleTime;
        this.min = min;
        this.max = max;

        elapsedTime = new ElapsedTime();
        totalError = 0;
        previousError = 0;

    }
    public double Update (double sensorReading){

        if(elapsedTime.milliseconds() < minSampleTime){
            return previousControlValue;
        }
        else {
            double error = setPoint - sensorReading;

            double controlValue = 0;

            if (firstRun) {

                firstRun = false;
            } else {
                double deltaTime = elapsedTime.seconds() / 60;

                totalError += error * deltaTime;

                controlValue = (P * error) + (I * totalError) + (D * ((error - previousError) / deltaTime));

            }

            elapsedTime.reset();
            previousError = error;

            if (controlValue < min) {
                previousControlValue = min;
                return min;

            }

            if (controlValue > max) {
                previousControlValue = max;
                return max;

            }

            previousControlValue = controlValue;
            return controlValue;
        }
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

    public double getPreviousError(){return previousError;}

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
