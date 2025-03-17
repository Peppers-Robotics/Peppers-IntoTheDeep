package org.firstinspires.ftc.teamcode.OpModes.tests;

import android.text.method.MultiTapKeyListener;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.IMUBNO085;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@TeleOp
public class IMUtest extends LinearOpMode {
    public static boolean resetYaw = false;
    public static float x = 0, y = 180, z = 0;
    public static int c = 0;
    @Override
    public void runOpMode() throws InterruptedException {
//        IMU imu = hardwareMap.get(IMU.class, "extIMU");
        telemetry = new MultipleTelemetry(telemetry, Robot.telemetry);
        Robot.InitializeHubs(hardwareMap);
//        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()){
            if(resetYaw){
//                Robot.imu.initialize(new Orientation(
//                        AxesReference.EXTRINSIC,
//                        AxesOrder.ZXY,
//                        AngleUnit.DEGREES,
//                        2, 3, 4
//                ))
                Orientation o = new Orientation(
                        AxesReference.EXTRINSIC,
                        AxesOrder.ZXY,
                        AngleUnit.DEGREES,
                        z, x, y, 0
                );
                Robot.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(o)));


                Robot.imu.resetYaw();
                resetYaw = false;
            }
            telemetry.addData("yaw", Robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("pitch", Robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            telemetry.addData("roll", Robot.imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
            telemetry.addData("new data", IMUBNO085.controller.getDigitalChannelState(c));
            Robot.clearCache(true);
//            telemetry.update();
        }
    }
}
