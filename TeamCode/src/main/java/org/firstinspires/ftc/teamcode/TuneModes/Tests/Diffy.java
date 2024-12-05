package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.OutTake.Arm;

@Config
@TeleOp(group = "tests")
public class Diffy extends LinearOpMode {
    public static double ArmAngle = 0, PivotAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeOuttake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()){
            Initialization.updateCacheing();
            Arm.setArmAngle(ArmAngle);
            Arm.setPivotAngle(PivotAngle);

            Arm.update();
            Initialization.telemetry.update();
        }
    }
}
