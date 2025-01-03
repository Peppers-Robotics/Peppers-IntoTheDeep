package org.firstinspires.ftc.teamcode.TuneModes.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.HelperClasses.Pose2D;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;

@TeleOp
@Config
public class SusJos extends LinearOpMode {
    public static double distance = 20;
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeChassis();
        Initialization.initializeExtendo();

        Scheduler s = new Scheduler();
        s.goTo(new Pose2D(distance, 0, 0)).waitSeconds(1).goTo(new Pose2D(0, 0, 0)).waitSeconds(1);

        waitForStart();
        Extendo.Extend(0);

        while (opModeIsActive()){
           Initialization.updateCacheing();

           s.update();
           if(s.done()){
               s.goTo(new Pose2D(distance, 0, 0)).waitSeconds(1).goTo(new Pose2D(0, 0, 0)).waitSeconds(1);
           }
           Chassis.Update();
           Initialization.telemetry.addData("pose", Chassis.GetCurrentPosition().toString());
           Initialization.telemetry.addData("target", Chassis.GetTargetPosition().toString());
           Initialization.telemetry.update();
        }
    }
}
