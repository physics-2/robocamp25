package org.firstinspires.ftc.teamcode.Robot_parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OutTake {
    private final Servo leftOutTakeServo;
    private final Servo rightOutTakeServo;
    private final Servo outTakeGrabServo;
    private final Servo outTakeRotationServo;
    private final Servo outTakeExtensionServo;
    private final DcMotorEx liftLeftExtensionMotor;
    private final DcMotorEx liftRightExtensionMotor;

    private final PIDCoefficients liftCoef = new PIDCoefficients(0.01,0.00001,0.00003);
    private double integralLift = 0;
    private final double iMaxLift = 0.3 ;
    private double errorOldLift = 0;
    private int liftTarget = 0;



    private static final double maxSpeed = 1;
    private static final double GRAB_REALESE_POS = 0.6;
    private static final double GRAB_HOLD_POS = 1;
    private static final double ROTATION_SCORE = 0.92;
    private static final double ROTATION_INTAKE = 0.81;
    private static final int LIFT_SCORE = 210;
    private static final int LIFT_INTAKE = 30;
    private static final double OUTTAKE_ARM_INTAKE_POS = 0.16;
    private static final double OUTTAKE_ARM_SCORE_POS = 0.735;
    private static final double EXTENSION_INTAKE_POS = 0.325;
    private static final double EXTENSION_SCORE_POS = 0.6;
    public OutTake(HardwareMap hardwareMap) {

        leftOutTakeServo = hardwareMap.get(Servo.class, "left_scor_perkid");
        rightOutTakeServo = hardwareMap.get(Servo.class, "right_scor_perkid");
        outTakeGrabServo = hardwareMap.get(Servo.class, "scoring_claw");
        outTakeRotationServo  = hardwareMap.get(Servo.class, "scoring_diff");
        outTakeExtensionServo  = hardwareMap.get(Servo.class, "Linkage");

        leftOutTakeServo.setDirection(Servo.Direction.FORWARD);
        rightOutTakeServo.setDirection(Servo.Direction.REVERSE);

        outTakeExtensionServo.setDirection(Servo.Direction.FORWARD);

        liftLeftExtensionMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        liftRightExtensionMotor = hardwareMap.get(DcMotorEx.class, "rightLift");


        liftLeftExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeftExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeftExtensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftLeftExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftRightExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRightExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRightExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRightExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftExtension(int Target){
        liftTarget = Target;
        integralLift = 0;
    }

    public void updateLiftPID(){

            int error = liftTarget - (liftLeftExtensionMotor.getCurrentPosition() + liftRightExtensionMotor.getCurrentPosition()) / 2;
            double prop = error * liftCoef.p;
            integralLift += error * liftCoef.i;
            if (Math.abs(integralLift) > iMaxLift)
                integralLift = iMaxLift * Math.signum(integralLift);
            double d = (error - errorOldLift) * liftCoef.d;
            double control = prop + integralLift + d;
            if (Math.abs(control) > 1) control = 1 * Math.signum(control);
            liftLeftExtensionMotor.setPower(control * maxSpeed);
            liftRightExtensionMotor.setPower(control * maxSpeed);
            errorOldLift = error;

    }

    public void setOutTakeArmPositon(double Position) {
        leftOutTakeServo.setPosition(Position);
        rightOutTakeServo.setPosition(Position);
    }

    public void setOutTakeArmExtension(double Position) {
        outTakeExtensionServo.setPosition(Position);
    }

    public void setOutTakeRotation(double Position) {
        outTakeRotationServo.setPosition(Position);
    }

    public void MoveIntakePos(){
        setLiftExtension(LIFT_INTAKE);
        setOutTakeArmExtension(EXTENSION_INTAKE_POS);
        setOutTakeArmPositon(OUTTAKE_ARM_INTAKE_POS);
        setOutTakeRotation(ROTATION_INTAKE);
    }

    public void MoveLiftScorePos(){
        setLiftExtension(LIFT_SCORE);
    }

    public void MoveScorePos(){
        setLiftExtension(LIFT_SCORE);
        setOutTakeArmPositon(OUTTAKE_ARM_SCORE_POS);
        setOutTakeArmExtension(EXTENSION_SCORE_POS);
        setOutTakeRotation(ROTATION_SCORE);
    }

    public void grabClaw() {
        outTakeGrabServo.setPosition(GRAB_HOLD_POS);
    }
    public void setGrabPos(double pos) {
        outTakeGrabServo.setPosition(pos);
    }

    public void releaseClaw() {
        outTakeGrabServo.setPosition(GRAB_REALESE_POS);
    }


}