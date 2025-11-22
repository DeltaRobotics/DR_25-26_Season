package org.firstinspires.ftc.teamcode.Custom;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Gen0AutoRed9")
public class Gen0AutoRed9 extends OpMode {

    Gen0MechanismHardwareMap mechanism = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(78, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(80, 79, Math.toRadians(45));
    private final Pose firstLineup = new Pose(89, 72, Math.toRadians(0) );
    private final Pose firstPickup = new Pose(115, 72, Math.toRadians(0) );
    private final Pose secondLineup = new Pose(110, 52, Math.toRadians(180));
    private final Pose secondPickup = new Pose(133, 52, Math.toRadians(180));
    private final Pose secondPickupBack = new Pose(20, 52, Math.toRadians(180));
    private final Pose movingBack = new Pose(90, 66, Math.toRadians(45) );


    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, firstLineupPath, firstPickupPath, shootFirstPickupPath,secondLineupPath, secondPickupPath, secondPickupBackPath, shootSecondLineupPath, movingBackPath ;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, Shooting));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading());

        firstLineupPath = new Path (new BezierLine(Shooting,firstLineup));
        firstLineupPath.setConstantHeadingInterpolation(firstLineup.getHeading());

        firstPickupPath = new Path (new BezierLine(firstLineup,firstPickup));
        firstPickupPath.setConstantHeadingInterpolation(firstPickup.getHeading());

        shootFirstPickupPath = new Path (new BezierLine(firstPickup,Shooting));
        shootFirstPickupPath.setConstantHeadingInterpolation(Shooting.getHeading());

        secondLineupPath = new Path (new BezierLine(Shooting,secondLineup));
        secondLineupPath.setConstantHeadingInterpolation(secondLineup.getHeading());

        secondPickupPath = new Path (new BezierLine(secondLineup,secondPickup));
        secondPickupPath.setConstantHeadingInterpolation(secondPickup.getHeading());

        secondPickupBackPath = new Path (new BezierLine(secondPickup,secondPickupBack));
        secondPickupBackPath.setConstantHeadingInterpolation(secondPickupBack.getHeading());

        shootSecondLineupPath = new Path (new BezierLine(secondPickup,Shooting));
        shootSecondLineupPath.setConstantHeadingInterpolation(Shooting.getHeading());

        movingBackPath = new Path (new BezierLine(Shooting,movingBack));
        movingBackPath.setConstantHeadingInterpolation(movingBack.getHeading());


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

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Shoot */
                    mechanism.intakeON();

                    for (int i=0; i<3; ++i){
                        mechanism.intakeOFF();
                        mechanism.kickerUP();
                        blockingSleep(500);
                        mechanism.kickerDOWN();
                        blockingSleep(100);
                        mechanism.intakeON();
                        blockingSleep(1000);
                    }
                    mechanism.intakeOFF();
                    setPathState(3);
                }
                break;
            case 3:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.followPath(firstLineupPath, true);
                    setPathState(4);
                }
                break;
            case 4:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    mechanism.intakeON();
                    follower.setMaxPower(0.6);

                    follower.followPath(firstPickupPath, true);
                    setPathState(5);
                }
                break;
            case 5:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    mechanism.intakeON();
                    follower.setMaxPower(1);
                    follower.followPath(shootFirstPickupPath, true);
                    setPathState(6);
                }
                break;
            case 6:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Shoot 2nd Time*/
                    mechanism.intakeON();

                    for (int i=0; i<3; ++i){
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
                    setPathState(7);
                }
                break;
            case 7:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    mechanism.intakeON();
                    follower.followPath(secondLineupPath, true);
                    setPathState(8);
                }
                break;
            case 8:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    mechanism.intakeON();
                    follower.setMaxPower(0.6);
                    follower.followPath(secondPickupPath, true);
                    setPathState(9);
                }
                break;
            case 9:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    mechanism.intakeON();
                    follower.setMaxPower(1);
                    follower.followPath(secondPickupBackPath, true);
                    setPathState(10);
                }
                break;
            case 10:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    mechanism.intakeON();
                    follower.followPath(shootSecondLineupPath, true);
                    setPathState(11);
                }
                break;
            case 11:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Shoot 2nd Time*/
                    mechanism.intakeON();

                    for (int i=0; i<3; ++i){
                        mechanism.intakeOFF();
                        mechanism.kickerUP();
                        blockingSleep(500);
                        mechanism.kickerDOWN();
                        blockingSleep(100);
                        mechanism.intakeON();
                        blockingSleep(1000);
                    }
                    mechanism.intakeOFF();
                    setPathState(12);
                }
                break;
            case 12:

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

        mechanism = new Gen0MechanismHardwareMap(hardwareMap);

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