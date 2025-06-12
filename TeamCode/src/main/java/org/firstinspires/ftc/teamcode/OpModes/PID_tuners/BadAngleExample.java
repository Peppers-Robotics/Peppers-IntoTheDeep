package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;

@Config
@TeleOp
public class BadAngleExample extends LinearOpMode {
    public static double A1 = 20, A2 = 292;
    static public  int freq = 40;
    @Override
    public void runOpMode() throws InterruptedException {


        Robot.InitializeFull(hardwareMap);
        Robot.enable();

        Scheduler s = new Scheduler(), r;
        s.lineToAsync(new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(A1))).waitForSync().waitSeconds(2).lineToAsync(new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(A2))).waitForSync().waitSeconds(2);
        r = s.clone();

        waitForStart();

        while (opModeIsActive()){
            Chassis.Heading.setFreq(freq);
            Robot.clearCache();

            if(r.done()) r = s.clone();

            Localizer.Update();
            Chassis.Update();
            r.update();
        }
    }
}
