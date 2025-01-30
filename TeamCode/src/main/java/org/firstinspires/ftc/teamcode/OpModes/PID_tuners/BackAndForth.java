package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
    public static double DISTANCE = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeChassis();
        Robot.InitializeLocalizer(hardwareMap);
        Robot.InitializeExtendo();
        Robot.InitializeDropDown();

        Scheduler run = new Scheduler();

        run
                .waitSeconds(1)
                .lineTo(new SparkFunOTOS.Pose2D(DISTANCE, 0, 0))
                .waitSeconds(4)
                .lineTo(new SparkFunOTOS.Pose2D(0, 0, 0))
                .waitSeconds(3)
        ;
        Extendo.Extend(0);
        DropDown.setDown(0);
        waitForStart();

        while(opModeIsActive()){
            Robot.clearCache();
            if(run.done()){
                run = new Scheduler();
                run
                        .waitSeconds(1)
                        .lineTo(new SparkFunOTOS.Pose2D(DISTANCE, 0, 0))
                        .waitSeconds(4)
                        .lineTo(new SparkFunOTOS.Pose2D(0, 0, 0))
                        .waitSeconds(3)
                        ;
            }

            run.update();
            Localizer.Update();
            Chassis.Update();
            Extendo.update();
            Robot.telemetry.addData("run", run);
        }
    }
}
