package org.firstinspires.ftc.teamcode.Gen2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Disabled
@Autonomous(name = "AutoRed12Far")
public class AutoRed12Far extends OpMode {

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(80, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(83, 10, Math.toRadians(69));
    private final Pose firstLineup = new Pose(103, 35, Math.toRadians(0));
    private final Pose firstPickup =  new Pose(133, 35, Math.toRadians(0));
    private final Pose secondLineup = new Pose(103, 58, Math.toRadians(0));
    private final Pose secondPickup = new Pose(133, 58, Math.toRadians(0));
    //private final Pose thirdLineup = new Pose(103, 6, Math.toRadians(0));
    private final Pose thirdPickup = new Pose(132, 10, Math.toRadians(0));
    private final Pose thirdPickupMove = new Pose(130, 14, Math.toRadians(0));
    private final Pose movingOffLine = new Pose(100, 10, Math.toRadians(90));
    private Path scorePreload, firstLineupPath, firstPickupPath, shootFirstPickupPath,secondLineupPath, secondPickupPath, secondPickupBackPath, shootSecondLineupPath, thirdPickupPath,thirdPickupMovePath, shootThirdPickupPath, movingOffLinePath ;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, Shooting));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading()); //startPose.getHeading(), Shooting.getHeading()

        firstLineupPath = new Path(new BezierLine(Shooting, firstLineup));
        firstLineupPath.setLinearHeadingInterpolation(Shooting.getHeading(), firstLineup.getHeading()); //firstLineup.getHeading()

        firstPickupPath = new Path(new BezierLine(firstLineup, firstPickup));
        firstPickupPath.setLinearHeadingInterpolation(firstLineup.getHeading(), firstPickup.getHeading()); //firstPickup.getHeading()

        shootFirstPickupPath = new Path(new BezierLine(firstPickup, Shooting));
        shootFirstPickupPath.setLinearHeadingInterpolation(firstPickup.getHeading(), Shooting.getHeading()); //Shooting.getHeading()

        secondLineupPath = new Path(new BezierLine(Shooting, secondLineup));
        secondLineupPath.setLinearHeadingInterpolation(Shooting.getHeading(), secondLineup.getHeading());//secondLineup.getHeading()

        secondPickupPath = new Path(new BezierLine(secondLineup, secondPickup));
        secondPickupPath.setLinearHeadingInterpolation(secondLineup.getHeading(), secondPickup.getHeading());//secondPickup.getHeading()

        shootSecondLineupPath = new Path(new BezierLine(secondPickup, Shooting));
        shootSecondLineupPath.setLinearHeadingInterpolation(secondPickup.getHeading(), Shooting.getHeading());//Shooting.getHeading()

        thirdPickupPath = new Path (new BezierLine(Shooting,thirdPickup));
        thirdPickupPath.setConstantHeadingInterpolation( thirdPickup.getHeading());//Shooting.getHeading(),

        thirdPickupMovePath = new Path (new BezierLine(thirdPickup,thirdPickupMove));
        thirdPickupMovePath.setLinearHeadingInterpolation( thirdPickup.getHeading(), thirdPickupMove.getHeading());//Shooting.getHeading(),

        shootThirdPickupPath = new Path (new BezierLine(thirdPickup,Shooting));
        shootThirdPickupPath.setLinearHeadingInterpolation(thirdPickup.getHeading(), Shooting.getHeading());

        movingOffLinePath = new Path(new BezierLine(Shooting, movingOffLine));
        movingOffLinePath.setLinearHeadingInterpolation(Shooting.getHeading(), movingOffLine.getHeading());//Shooting.getHeading()


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!follower.isBusy()) {
                    robot.intake.setPower(1);
                    robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
                    robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);


                    follower.followPath(scorePreload, true);
                    setPathState(1);
                }

                break;

            case 1:

                if(!follower.isBusy()) {

                    if(!robot.timerInitted[4]) {//very very first thing to happen
                        robot.timeArray[4] = robot.currentTime.milliseconds();
                        robot.timerInitted[4] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[4] + 1800) {//Last thing to happen
                        setPathState(2);
                        robot.timerInitted[4] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;
            case 2:

                if(!follower.isBusy()) {

                    robot.intake();
                    follower.followPath(firstLineupPath, true);
                    robot.display_state_RedIdle();
                    setPathState(3);
                }
                break;
            case 3:

                if(!follower.isBusy()) {

                    follower.followPath(firstPickupPath, true);
                    setPathState(4);
                }
                break;
            case 4:

                if(!follower.isBusy()) {

                    follower.followPath(shootFirstPickupPath, true);
                    setPathState(5);
                }
                break;
            case 5:

                if(!follower.isBusy()) {

                    if(!robot.timerInitted[5]) {//very very first thing to happen
                        robot.timeArray[5] = robot.currentTime.milliseconds();
                        robot.timerInitted[5] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[5] + 1800) {//Last thing to happen
                        setPathState(6);
                        robot.timerInitted[5] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;
            case 6:

                if(!follower.isBusy()) {
                    robot.intake();
                    follower.followPath(secondLineupPath, true);
                    robot.display_state_RedIdle();
                    setPathState(7);
                }
                break;
            case 7:

                if(!follower.isBusy()) {
                    follower.followPath(secondPickupPath, true);
                    setPathState(9);
                }
                break;
            case 9:

                if(!follower.isBusy()) {
                    follower.followPath(shootSecondLineupPath, true);
                    setPathState(10);
                }
                break;
            case 10:

                if(!follower.isBusy()) {

                    if(!robot.timerInitted[6]) {//very very first thing to happen
                        robot.timeArray[6] = robot.currentTime.milliseconds();
                        robot.timerInitted[6] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[6] + 1800) {//Last thing to happen
                        setPathState(12);
                        robot.timerInitted[6] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;

            case 12:

                if(!follower.isBusy()) {
                    robot.intake();
                    follower.followPath(thirdPickupPath, false);
                    setPathState(13);
                }
                break;
            case 13:

                if(!follower.isBusy()) {
                    robot.intake();
                    follower.followPath(thirdPickupMovePath, false);
                    setPathState(14);
                }
                break;
            case 14:

                if(!follower.isBusy()) {

                    follower.followPath(shootThirdPickupPath, true);
                    setPathState(15);
                }
                break;
            case 15:

                if(!follower.isBusy()) {

                    if(!robot.timerInitted[5]) {//very very first thing to happen
                        robot.timeArray[5] = robot.currentTime.milliseconds();
                        robot.timerInitted[5] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[5] + 1800) {//Last thing to happen
                        setPathState(16);
                        robot.timerInitted[5] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;
            case 16:

                if(!follower.isBusy()) {
                    robot.stopIntake();
                    follower.followPath(movingOffLinePath, true);
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        //robot.turret(telemetry);

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("tx", robot.limelight.getLatestResult().getFiducialResults().isEmpty() ? "No Target" : robot.limelight.getLatestResult().getFiducialResults().get(0).getTargetXDegrees());
        telemetry.update();;
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new Gen2Hardwaremap(hardwareMap);

        robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
        robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);

        robot.transfer.setPower(0);

        robot.L_PTO.setPosition(robot.L_PTO_UP);
        robot.R_PTO.setPosition(robot.R_PTO_UP);

        robot.R_feeder.setPower(0);
        robot.L_feeder.setPower(0);

        robot.R_turret.setPower(0);
        robot.L_turret.setPower(0);

        robot.targetRPM = 1000;

        robot.autoFarShoot();

        robot.blue = false;

        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        //robot.turret(telemetry);
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}