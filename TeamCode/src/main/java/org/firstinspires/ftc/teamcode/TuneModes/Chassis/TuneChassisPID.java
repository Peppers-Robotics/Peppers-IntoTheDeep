package org.firstinspires.ftc.teamcode.TuneModes.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.HelperClasses.Pose2D;
import org.firstinspires.ftc.teamcode.Initialization;

@Config
@TeleOp(group = "tests")
public class TuneChassisPID extends LinearOpMode {
    public static Pose2D pose = new Pose2D(0, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeChassis();

        waitForStart();

        while (opModeIsActive()){
            Initialization.updateCacheing();
            Chassis.SetTargetPosition(pose);
            Chassis.Update();
            Initialization.telemetry.addData("pose: ", Chassis.GetCurrentPosition().toString());
            Initialization.telemetry.update();
        }
    }
}
