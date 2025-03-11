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

import java.util.Arrays;

@TeleOp
@Config
public class CurveTest extends LinearOpMode {
    public static SparkFunOTOS.Pose2D A1 = new SparkFunOTOS.Pose2D(600, 0, 0), A2 = new SparkFunOTOS.Pose2D(950, -900, Math.PI / 2);
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        DropDown.setDown(0);
        Extendo.Extend(0);
        Robot.enable();

        Scheduler s = new Scheduler(), r;
        s
                .splineToAsync(Arrays.asList(A1, A2))
                .waitForSync()
                .waitSeconds(2)
                .splineToAsync(Arrays.asList(A1, new SparkFunOTOS.Pose2D(0, 0, 0)))
                .waitForSync()
                .waitSeconds(2);
        r = s.clone();

        waitForStart();

        while(opModeIsActive()){
            Robot.clearCache();
            if(r.done()) r = s.clone();

            r.update();
            Chassis.Update();
            Localizer.Update();
            Extendo.update();
        }
    }
}
