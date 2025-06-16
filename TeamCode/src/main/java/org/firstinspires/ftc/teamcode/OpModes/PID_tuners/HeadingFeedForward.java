package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Config
@TeleOp
public class HeadingFeedForward extends LinearOpMode {
    public static double angle = 180;
    static public int freq = 30;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        Chassis.ONLY_FF_HEADING = true;

        Scheduler s = new Scheduler(), r;

        s
                .lineToAsync(new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(angle)))
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        Localizer.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(180)));
                        return true;
                    }
                })
                .waitSeconds(2)
                .lineToAsync(new SparkFunOTOS.Pose2D()).waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Localizer.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(0)));
                        return true;
                    }
                })
                .waitSeconds(2);

        r = s.clone();

        waitForStart();

        while (opModeIsActive()){
            Robot.clearCache(true);

            if(r.done()) r = s.clone();

            Robot.telemetry.addData("target velocity", Chassis.hProfile.getVelocity());
            Robot.telemetry.addData("current velocity", Localizer.getVelocity().h);

            Localizer.Update();
            Chassis.Update();
            r.update();
        }
    }
}
