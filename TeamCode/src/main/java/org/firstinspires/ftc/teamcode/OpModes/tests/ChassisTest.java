package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Robot;


@TeleOp
@Config
public class ChassisTest extends LinearOpMode {

    public static double x = 0;
    public static double y = 0;
    public static double rot = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeChassis();

        waitForStart();

        while(opModeIsActive())
        {
            Chassis.drive(x,y,rot);
        }
    }
}
