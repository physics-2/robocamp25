package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");

        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class,"rightRear");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class,"leftRear");


        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()){
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            rightRear.setPower(0.5);
            leftRear.setPower(0.5);
        }


    }
}
