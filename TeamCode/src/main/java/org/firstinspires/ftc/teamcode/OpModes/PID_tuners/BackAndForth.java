package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;

@TeleOp(group = "Chassis")
@Config
public class BackAndForth extends LinearOpMode {
//    public static double DISTANCE = 1500;
    public static SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D(0, 0, 0);
    public static double h = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis.Autonomous = true;
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeChassis();
        Robot.InitializeLocalizer(hardwareMap);
        Robot.InitializeExtendo();
        Robot.InitializeDropDown();

        Scheduler run = new Scheduler();
        Scheduler up = new Scheduler();

        run
                .lineToAsync(new SparkFunOTOS.Pose2D(pose.x, pose.y, Math.toRadians(h)))
                .waitForSync()
                .waitSeconds(2)
                .lineToAsync(new SparkFunOTOS.Pose2D(0, 0, 0))
                .waitForSync()
                .waitSeconds(2)
        ;
        Extendo.Extend(0);
        DropDown.setDown(0);
        waitForStart();

        while(opModeIsActive()){
            Robot.clearCache();
            if(up.done()){
                up = run.clone();
            }
            up.update();
            Localizer.Update();
            Chassis.Update();
            Extendo.update();
            Robot.telemetry.addData("run", run);
        }
    }
}
