package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(group = "Chassis")
@Config
public class HoldPosition extends LinearOpMode{
    public static SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D();
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeLocalizer(hardwareMap);
        Robot.InitializeChassis();
        waitForStart();

        while (opModeIsActive()){
            Robot.clearCache();

            Chassis.setTargetPosition(pose);
            Chassis.Update();
            Localizer.Update();
        }
    }
}
