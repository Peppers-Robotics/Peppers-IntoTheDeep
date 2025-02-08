package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class ArmTest extends LinearOpMode {
    public static double ArmAngle = 0, PivotAngle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeArm();

        waitForStart();

        while (opModeIsActive()){
            Arm.setArmAngle(ArmAngle);
            Arm.setPivotAngle(PivotAngle);

            Arm.update();
        }
    }
}
