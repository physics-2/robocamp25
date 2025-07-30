//To do this test you need to make all outtake classes public

package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot_parts.OutTake;


@Config
@Autonomous(name = "Kinematics", group = "Tests")
public class KinematicTest extends LinearOpMode {
    public static int liftExtension = 0;
    public static double outTakeRotation = 0.76;
    public static double outTakeArmRotation = 0.165;
    public static double outTakeExtension = 0.325;
    public static double grabPos = 0.6;
    public static boolean update = true;
    private OutTake outTake;
    @Override
    public void runOpMode() throws InterruptedException {
        outTake = new OutTake(hardwareMap);
        waitForStart();

       while (opModeIsActive()) {
           if (update) {
               outTake.setLiftExtension(liftExtension);
               outTake.setOutTakeRotation(outTakeRotation);
               outTake.setOutTakeArmPositon(outTakeArmRotation);
               outTake.setOutTakeArmExtension(outTakeExtension);
               outTake.setGrabPos(grabPos);
           }
           outTake.updateLiftPID();
       }


    }
}