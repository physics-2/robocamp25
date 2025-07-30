package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot_parts.Intake;

@Config
@Autonomous
public class IntakePosTest extends LinearOpMode {
    private Intake intake;
    /* un comment this lines later
    public static double intakeYaw = 0.5;
    public static double intakePitch = 0.5;
    public static double intakeRightDiff = 0.26;
    public static double intakeLeftDiff = 0.26;

     */

    public static int ThrowServoTime = 15;
    public static int ThrowLinkageTime = 75;
    //public static double clawPos = 0.5;
    public static boolean update = true;
   // public static boolean isThrowPos = true; 2
    //boolean isThrown = true; 2
    //boolean isGrabbed = false; 2
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        boolean grabflag = true;
        boolean throwflag1 = true;
        intake.setThrowTime(ThrowServoTime, ThrowLinkageTime);
        waitForStart();

        while (opModeIsActive()) {
            if(grabflag){
                intake.doGrab();
            }
            grabflag = false;
            if(throwflag1){
                intake.doPut();
            }
            throwflag1 = false;
            intake.updateLinkagePID();
        }

    }
}
