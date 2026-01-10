package org.firstinspires.ftc.teamcode.Gen2;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {

    private boolean[] timerInit;

    private double[] timer;

    private ElapsedTime currentTime = new ElapsedTime();

    public Timer(){

        timer = new double[20];
        timerInit = new boolean[20];
    }

    public void initTimer(int timerIndex){

        timer[timerIndex] = currentTime.milliseconds();
        timerInit[timerIndex] = true;
    }

}
