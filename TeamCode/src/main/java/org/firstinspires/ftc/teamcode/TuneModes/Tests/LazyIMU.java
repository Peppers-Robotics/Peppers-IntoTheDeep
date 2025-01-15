package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import android.net.wifi.p2p.WifiP2pManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Initialization;

@TeleOp(group = "tests")
public class LazyIMU extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeClimb();
        IMU imu = hardwareMap.get(IMU.class, "imu") ;
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Climb.Raise();
        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            telemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
