package org.firstinspires.ftc.teamcode.TuneModes.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Initialization;

@Config
@TeleOp(group = "tests")
public class TuneChassisPID extends LinearOpMode {
    SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D(0, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeChassis();

        waitForStart();

        while (opModeIsActive()){
            Chassis.SetTargetPosition(new Pose2d(pose.x, pose.y, pose.h));
            Chassis.Update();
        }
    }
}
