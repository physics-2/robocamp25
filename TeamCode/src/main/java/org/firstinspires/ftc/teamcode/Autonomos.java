package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot_parts.Intake;
import org.firstinspires.ftc.teamcode.Robot_parts.OutTake;
import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;


@Config
@Autonomous(name = "six sample autonomous", group = "Autos")
public class Autonomos extends OpMode {

    private Follower follower;
    private OutTake outTake;
    private Intake intake;
    private final int CYCLE_AMOUNT = 5;
    private final double AT_PATH_END = 0.945;
    private final double AT_PATH_MIDDLE = 0.45;
    private final double AT_PATH_START = 0.1;
    private  boolean goingToScore  = true;//half period cycle,if true - moves to scoring position,if false - moves to intake position

    private int loopCount = 0;
    private int pathState = 0;
    private Timer pathTimer;

    private final Pose startPose = new Pose(0, 60, Math.toRadians(0));

    private final Pose scorePositionFirst = new Pose(46, 72, Math.toRadians(0));
    private final Pose scoreFirstControlPoint = new Pose(21, 72, Math.toRadians(0));


    private final Pose grab1Spes = new Pose(75, 25, Math.toRadians(0));
    private final Pose scoreFirstControlPoint2 = new Pose(75, 36, Math.toRadians(0));
    private final Pose grab1SpesControlPoint = new Pose(6, 25, Math.toRadians(0));

    private final Pose put1Spes = new Pose(25, 25, Math.toRadians(0));

    private final Pose grab2Spes = new Pose(75, 14, Math.toRadians(0));
    private final Pose grab2SpesControlPoint = new Pose(80, 24, Math.toRadians(0));

    private final Pose put2Spes = new Pose(25, 14, Math.toRadians(0));

    private final Pose intakeTransition = new Pose(-0.1, 38, Math.toRadians(0));
    private final Pose intakeTransitionControlPoint = new Pose(33, 42, Math.toRadians(0));
    private final Pose intakeTransitionControlPoint2 = new Pose(29, 37, Math.toRadians(0));

    private final Pose scoreLoop = new Pose(46, 72, Math.toRadians(0));
    private final Pose LoopControlPoint = new Pose(55, 32, Math.toRadians(0));
    private final Pose LoopControlPoint2 = new Pose(0, 72, Math.toRadians(0));

    private final Pose intakeLoop = new Pose(-0.1, 38, Math.toRadians(0));




    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);

        outTake = new OutTake(hardwareMap);
        intake = new Intake(hardwareMap);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        buildPaths();
        setPathState(0);
        loopCount = 0;
        outTake.grabClaw();
        outTake.MoveScorePos();
        outTake.MoveLiftScorePos();
        intake.setLinkageExtension(0);

    }

    PathChain scoreFirst,ToGrab1,Put1,ToGrab2,Put2,intakeTransit,toScoreLoop,toIntakeLoop;

    public void buildPaths() {
        scoreFirst = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(scoreFirstControlPoint),
                        new Point(scorePositionFirst)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(200,() -> outTake.MoveLiftScorePos())
                .addParametricCallback(AT_PATH_END,() -> outTake.releaseClaw())
                .build();


        ToGrab1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePositionFirst),
                        new Point(grab1SpesControlPoint),
                        new Point(scoreFirstControlPoint2),
                        new Point(grab1Spes)
                ))
                .addParametricCallback(AT_PATH_MIDDLE,() -> outTake.MoveIntakePos())
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        Put1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(grab1Spes),
                        new Point(put1Spes)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        ToGrab2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(put1Spes),
                        new Point(grab2SpesControlPoint),
                        new Point(grab2Spes)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        Put2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(grab2Spes),
                        new Point(put2Spes)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(AT_PATH_MIDDLE,() -> outTake.MoveIntakePos())
                .build();


        intakeTransit = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(put2Spes),
                        new Point(intakeTransitionControlPoint),
                        new Point(intakeTransitionControlPoint2),
                        new Point(intakeTransition)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(AT_PATH_END,() -> outTake.grabClaw())
                .build();


        toScoreLoop = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intakeLoop),
                        new Point(LoopControlPoint),
                        new Point(LoopControlPoint2),
                        new Point(scoreLoop)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(100  ,() -> outTake.MoveScorePos())
                .build();

        toIntakeLoop = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scoreLoop),
                        new Point(LoopControlPoint2),
                        new Point(LoopControlPoint),
                        new Point(intakeLoop)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(200,() -> outTake.MoveIntakePos())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(scoreFirst, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(ToGrab1, false);
                    setPathState(2);

                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Put1, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(ToGrab2, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Put2, false);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);//to increase accuracy
                    follower.followPath(intakeTransit, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (loopCount < CYCLE_AMOUNT) {
                        if (goingToScore) {
                            outTake.grabClaw();
                            follower.followPath(toScoreLoop, true);
                            if(pathTimer.getElapsedTime() > 200)goingToScore = false;
                        }
                        else
                        {
                            outTake.releaseClaw();
                            follower.followPath(toIntakeLoop, true);
                           if(pathTimer.getElapsedTime() > 200){
                               goingToScore = true;
                               loopCount++;
                           }
                        }
                    } else {
                        setPathState(7);
                    }
                }
                break;
            case 7:

                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        outTake.updateLiftPID();
        intake.updateLinkagePID();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Loop Count", loopCount);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }
}