package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.OutTake.Arm;

@Config
@TeleOp(group = "tests")
public class Diffy extends LinearOpMode {
    public static double ArmAngle = 0, PivotAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeOuttake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()){
            Arm.setArmAngle(ArmAngle);
            Arm.setPivotAngle(PivotAngle);

            Arm.update();
            Initialization.telemetry.update();
        }
    }
}
