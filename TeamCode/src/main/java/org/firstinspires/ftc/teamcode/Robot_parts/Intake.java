package org.firstinspires.ftc.teamcode.Robot_parts;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    private final Servo intakeClaw;
    private final Servo rightIntakePitch;
    private final Servo leftIntakePitch;
    private final Servo intakeYaw;
    private final Servo rightIntakeDiff;
    private final Servo leftIntakeDiff;
    private final DcMotorEx linkageIntake;

    private final OutTake outTake;

    public Timer waitTimer;

    public int ThrowServoTime = 30;
    public int ThrowLinkageTime = 75;


    private final PIDCoefficients linkageCoef = new PIDCoefficients(0.03, 0, 0.00005);
    private double linkageI = 0;
    private final double iMaxLinkage = 0;
    private double errorOldLinkage = 0;
    private int linkageTarget = 0;


    private static final double maxSpeed = 0.8;
    private static final double CLAW_OPEN = 0.35;
    private static final double CLAW_CLOSE = 0.59;
    private static final double PITCH_THROW = 0.2;
    private static final double PITCH_GRAB = 0.79;
    private static final int LINKAGE_ZERO = 40;
    private static final int LINKAGE_GET3SAMPLE = 500;
    private static final double YAW_GET3SAMPLE = 0.15;
    private static final double YAW_GRAB = 0.5;
    private static final double YAW_THROW = 0.45;
    private static final double DIFF_RIGHT_GRAB = 0.35;
    private static final double DIFF_LEFT_GRAB = 0.55;
    private static final double DIFF_RIGHT_THROW = 0.9;
    private static final double DIFF_LEFT_THROW = 0.8;



    public Intake(HardwareMap hardwareMap) {

        intakeClaw = hardwareMap.get(Servo.class, "claw_servo");
        intakeYaw = hardwareMap.get(Servo.class, "horizontal_servo");
        leftIntakePitch = hardwareMap.get(Servo.class, "left_vertical_servo");
        rightIntakePitch = hardwareMap.get(Servo.class, "right_vertical_servo");
        leftIntakeDiff = hardwareMap.get(Servo.class, "left_diff_servo");
        rightIntakeDiff = hardwareMap.get(Servo.class, "right_diff_servo");

        leftIntakePitch.setDirection(Servo.Direction.FORWARD);
        rightIntakePitch.setDirection(Servo.Direction.FORWARD);

        leftIntakeDiff.setDirection(Servo.Direction.REVERSE);
        rightIntakeDiff.setDirection(Servo.Direction.FORWARD);


        linkageIntake = hardwareMap.get(DcMotorEx.class, "extend_motor");

        linkageIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        linkageIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkageIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkageIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        openClaw();
        setIntakeYaw(YAW_THROW);
        setIntakePitch(PITCH_THROW);
        setIntakeDiff(DIFF_LEFT_THROW,DIFF_RIGHT_THROW);

        outTake = new OutTake(hardwareMap);
        waitTimer = new Timer();

    }

    public void setThrowTime(int ThrowServoTimePar,int LinkageThrowTimePar){
        ThrowServoTime = ThrowServoTimePar;
        ThrowLinkageTime = LinkageThrowTimePar;
    }

    public void setLinkageExtension(int Target) {
        linkageTarget = Target;
        linkageI = 0;
    }

    public void updateLinkagePID() {
        int error = linkageTarget - linkageIntake.getCurrentPosition();
        double prop = error * linkageCoef.p;
        linkageI += error * linkageCoef.i;
        if (Math.abs(linkageI) > iMaxLinkage) linkageI = iMaxLinkage * Math.signum(linkageI);
        double d = (error - errorOldLinkage) * linkageCoef.d;
        double control = prop + linkageI + d;
        if (Math.abs(control) > 1) control = 1 * Math.signum(control);
        linkageIntake.setPower(control * maxSpeed);
        errorOldLinkage = error;
    }

    public void setIntakePitch(double Position) {
        leftIntakePitch.setPosition(Position);
        rightIntakePitch.setPosition(Position);
    }

    public void setIntakeYaw(double Position) {
        intakeYaw.setPosition(Position);
    }

    public void setIntakeDiff(double leftPosition, double rightPosition) {
        leftIntakeDiff.setPosition(leftPosition);
        rightIntakeDiff.setPosition(rightPosition);
    }

    public void doGrab() {
        openClaw();
        setLinkageExtension(LINKAGE_ZERO);
        setIntakeYaw(YAW_GRAB);
        setIntakePitch(PITCH_GRAB);
        setIntakeDiff(DIFF_LEFT_GRAB, DIFF_RIGHT_GRAB);
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() <= 900 ) updateLinkagePID();
        closeClaw();
        setLinkageExtension(LINKAGE_ZERO);
        setIntakeYaw(YAW_THROW);
        setIntakePitch(PITCH_THROW);
        setIntakeDiff(DIFF_LEFT_GRAB, DIFF_RIGHT_GRAB);

    }

    public void doPut() {
        setLinkageExtension(LINKAGE_ZERO);
        setIntakeYaw(YAW_GRAB);
        setIntakePitch(PITCH_GRAB);
        setIntakeDiff(DIFF_LEFT_GRAB, DIFF_RIGHT_GRAB);
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() <= 900 ) updateLinkagePID();
        openClaw();
        setLinkageExtension(LINKAGE_ZERO);
        setIntakeYaw(YAW_THROW);
        setIntakePitch(PITCH_THROW);
        setIntakeDiff(DIFF_LEFT_GRAB, DIFF_RIGHT_GRAB);
    }

    public void do3SampleGrab(){
        setLinkageExtension(LINKAGE_GET3SAMPLE);
        setIntakeYaw(YAW_GET3SAMPLE);
        setIntakePitch(PITCH_GRAB);
        setIntakeDiff(DIFF_LEFT_GRAB, DIFF_RIGHT_GRAB);
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() <= 900 ) updateLinkagePID();
        closeClaw();
        updateLinkagePID();
    }

    private void closeClaw(){
        intakeClaw.setPosition(CLAW_CLOSE);
    }

    public void setClawPos(double pos){
        intakeClaw.setPosition(pos);
    }

    private void openClaw(){
        intakeClaw.setPosition(CLAW_OPEN);
    }
}