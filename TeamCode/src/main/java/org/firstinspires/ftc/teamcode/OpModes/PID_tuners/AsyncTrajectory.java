package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;

@TeleOp(name = "BackAndForth")
@Config
public class AsyncTrajectory extends LinearOpMode {
    public static double DISTANCE = 1500;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeChassis();
        Robot.InitializeLocalizer(hardwareMap);

        Scheduler task = new Scheduler(), runner;
        task
                .lineToLinearHeadingAsync(new SparkFunOTOS.Pose2D(DISTANCE, 0, Math.toRadians(90)))
                .waitForSync()
                .waitSeconds(2)
                .lineToLinearHeadingAsync(new SparkFunOTOS.Pose2D(0, 0, 0))
                .waitForSync()
                .waitSeconds(2)

                ;

        waitForStart();
        runner = task.clone();

        while (opModeIsActive()){
            Robot.clearCache();
            if(runner.done()) runner = task.clone();

            runner.update();
            Localizer.Update();
            Chassis.Update();
        }
    }
}
