package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;



@Config
@Autonomous(name = "StandartOpMode")
public class OpMode extends LinearOpMode {
    DcMotorEx LeftMotor = null;
    private double errorOld = 0;
    private double i = 0;
    private  final double  iMax = 0.5;
    public static final double kP = 0.01;

    public static final double kI = 0.000001;
    public static final double kD = 0.3;

    public void Move(int encoder) {
            int error = encoder - LeftMotor.getCurrentPosition();
            double p = error * kP;
            i += error * kI;
            if(Math.abs(i)>iMax) i = iMax * Math.signum(i);
            double d = (error - errorOld) * kD;
            double control = p + i + d;
            if(Math.abs(control)> 1) control = 1 * Math.signum(control);
            LeftMotor.setPower(control);
            errorOld = error;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        LeftMotor = hardwareMap.get(DcMotorEx.class,"LeftMotor");

        LeftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        LeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        System.currentTimeMillis();


        waitForStart();
        while (opModeIsActive()){

            Move(500);

        }




    }
}
