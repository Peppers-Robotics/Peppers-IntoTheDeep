package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.LazyIMUBHI;

@TeleOp(group = "tests")
public class LazyIMU extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LazyIMUBHI imu = hardwareMap.get(LazyIMUBHI.class, "imu") ;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("heading", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
