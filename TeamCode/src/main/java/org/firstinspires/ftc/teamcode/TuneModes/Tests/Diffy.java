package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.OutTake.Arm;

@Config
@TeleOp(group = "tests")
public class Diffy extends LinearOpMode {
    public static double ArmAngle = 0, PivotAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm.servo1 = new ServoPlus(hardwareMap.get(Servo.class, "cS0"));
        Arm.servo2 = new ServoPlus(hardwareMap.get(Servo.class, "cS1"));
        waitForStart();

        while (opModeIsActive()){
            Arm.setArmAngle(ArmAngle);
            Arm.setPivotAngle(PivotAngle);

            Arm.update();
        }
    }
}
