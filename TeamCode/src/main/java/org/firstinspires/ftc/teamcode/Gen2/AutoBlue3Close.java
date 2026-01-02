package org.firstinspires.ftc.teamcode.Gen2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gen0.Gen0MechanismHardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */
@Disabled
@Autonomous(name = "AutoBlue3Close")
public class AutoBlue3Close extends OpMode {

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer,  opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(50, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(48, 79, Math.toRadians(135));
    private final Pose movingBack = new Pose(39, 66, Math.toRadians(135));


    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */



    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, movingBackPath;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, Shooting));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading());

        movingBackPath = new Path(new BezierLine(Shooting, movingBack));
        movingBackPath.setConstantHeadingInterpolation(Shooting.getHeading());

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!follower.isBusy()) {
                    /* Score Preload */
                    mechanism.shooterON(0.6);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePreload, true);
                    setPathState(2);
                }

                break;
            case 2:

                if(!follower.isBusy()) {
                    /* Score Preload */
                    mechanism.intakeON();

                    for (int i=0; i<6; ++i){
                        mechanism.intakeOFF();
                        mechanism.kickerUP();
                        blockingSleep(500);
                        mechanism.kickerDOWN();
                        blockingSleep(100);
                        mechanism.intakeON();
                        blockingSleep(1000);
                    }
                    mechanism.shooterOFF();
                    mechanism.intakeOFF();

                    setPathState(3);
                }

                break;
            case 3:

                if(!follower.isBusy()) {

                    follower.followPath(movingBackPath, true);

                    setPathState(-1);
                }
                break;

        }
    }

    public void blockingSleep(int milliseconds){
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        int targetTime = (int)timer.milliseconds() + milliseconds;

        while(timer.milliseconds() < targetTime){

        }
    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new Gen2Hardwaremap(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}