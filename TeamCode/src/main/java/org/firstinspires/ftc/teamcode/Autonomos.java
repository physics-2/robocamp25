package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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
@Autonomous(name = "PedroPathingTest", group = "Tests")
public class Autonomos extends OpMode {

    private Follower follower;
    private OutTake outTake;
    private Intake intake;

    private int loopCount = 0;
    private int pathState = 0;
    private Timer actionTimer;
    private Timer pathTimer;

    private final Pose startPose = new Pose(12, 61, Math.toRadians(0));

    private final Pose score1Pos = new Pose(37, 72, Math.toRadians(0));
    private final Pose score1ControlPoint = new Pose(12, 73, Math.toRadians(0));

    private final Pose throw1 = new Pose(35, 24, Math.toRadians(0));
    private final Pose throw1ControlPoint = new Pose(12, 46, Math.toRadians(0));

    private final Pose throw2 = new Pose(35, 13, Math.toRadians(0));
    private final Pose throw3 = new Pose(29, 9, Math.toRadians(-18));

    private final Pose intakePos = new Pose(11, 35.5, Math.toRadians(0));
    private final Pose intakeControlPoint = new Pose(16, 13, Math.toRadians(0));
    private final Pose scorePos = new Pose(37, 72.5, Math.toRadians(0));
    private final Pose scoreControlPoint1 = new Pose(34, 32, Math.toRadians(0));
    private final Pose scoreControlPoint2 = new Pose(8, 75, Math.toRadians(0));

    private final Pose parkPos = new Pose(10, 8, Math.toRadians(0));

    private PathChain throwPath1,throwPath2,throwPath3,intakeTransition;
    private PathChain scorePath, intakePath,park;

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);

        outTake = new OutTake(hardwareMap);
        intake = new Intake(hardwareMap);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        buildPaths();
        setPathState(0);
        loopCount = 0;

        outTake.MoveIntakePos();
    }

    public void buildPaths() {
        Path score1 = new Path(new BezierCurve(
                new Point(startPose),
                new Point(score1ControlPoint),
                new Point(score1Pos)
        ));
        score1.setConstantHeadingInterpolation(Math.toRadians(0));

        Path toThrow1 = new Path(new BezierCurve(
                new Point(score1Pos),
                new Point(throw1ControlPoint),
                new Point(throw1)
        ));
        toThrow1.setConstantHeadingInterpolation(Math.toRadians(0));

        Path toThrow2 = new Path(new BezierLine(
                new Point(throw1),
                new Point(throw2)
        ));
        toThrow2.setConstantHeadingInterpolation(Math.toRadians(0));

        Path toThrow3 = new Path(new BezierLine(
                new Point(throw2),
                new Point(throw3)
        ));
        toThrow3.setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-18));

        Path toIntake1 = new Path(new BezierCurve(
                new Point(throw3),
                new Point(intakeControlPoint),
                new Point(intakePos)
        ));
        toIntake1.setLinearHeadingInterpolation(Math.toRadians(-18),Math.toRadians(0));

        Path scoreLoop = new Path(new BezierCurve(
                new Point(intakePos),
                new Point(scoreControlPoint1),
                new Point(scoreControlPoint2),
                new Point(scorePos)
        ));
        scoreLoop.setConstantHeadingInterpolation(Math.toRadians(0));

        Path intakeLoop = new Path(new BezierCurve(
                new Point(scorePos),
                new Point(scoreControlPoint2),
                new Point(scoreControlPoint1),
                new Point(intakePos)
        ));
        intakeLoop.setConstantHeadingInterpolation(Math.toRadians(0));

        throwPath1 = follower.pathBuilder()
                .addPath(score1)
                .addPath(toThrow1)
                .addTemporalCallback(1200, () -> outTake.MoveIntakePos())
                .build();

        throwPath2 = follower.pathBuilder()
                .addPath(toThrow2)
                .build();

        throwPath3 = follower.pathBuilder()
                .addPath(toThrow3)
                .build();

        intakeTransition = follower.pathBuilder()
                .addPath(toIntake1)
                .build();

        scorePath = follower.pathBuilder()
                .addPath(scoreLoop)
                .addTemporalCallback(700, () -> outTake.MoveScorePos())
                .build();

        intakePath = follower.pathBuilder()
                .addPath(intakeLoop)
                .addParametricCallback(0.3, () -> outTake.MoveIntakePos())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePos), new Point(parkPos)))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(throwPath1, true);
                    setPathState(1);
                }

                break;
            case 1:
                intake.doGrab();
                if (actionTimer.getElapsedTime() >= 500) {

                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(throwPath2, true);
                    setPathState(3);
                }
                break;

            case 3:
                intake.doGrab();
                if (actionTimer.getElapsedTime() >= 500) {

                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(throwPath3, true);
                    setPathState(5);
                }
                break;


            case 5:
                intake.do3SampleGrab();
                if (actionTimer.getElapsedTime() >= 500 ) {
                    intake.do3SampleGrab();
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(intakeTransition, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    if (loopCount < 5) {
                        follower.followPath(scorePath, true);
                        setPathState(8);
                    } else {
                        setPathState(9);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(intakePath, true);
                    setPathState(7);
                    loopCount++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(10);
                }
                break;

            case 10:
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