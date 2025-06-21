package org.firstinspires.ftc.teamcode.OpModes.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import dalvik.system.DelegateLastClassLoader;

@TeleOp
@Config
public class StorageTest extends LinearOpMode {
    public static boolean climb = false;
    public static double freq = 20;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeStorage(hardwareMap);
        waitForStart();

        while (opModeIsActive()){

            Robot.clearCache();
//            Storage.sensor.getColorSeenBySensor();

            Robot.telemetry.addData("Color", Storage.sensor.getColorSeenBySensor());
//            Robot.telemetry.addData("r, g, b", Storage.sensor.RGB.R + ", " + Storage.sensor.RGB.G + ", " + Storage.sensor.RGB.B);
            Robot.telemetry.addData("r", Storage.sensor.RGB.R);
            Robot.telemetry.addData("g", Storage.sensor.RGB.G);
            Robot.telemetry.addData("b", Storage.sensor.RGB.B);

            Robot.telemetry.addData("sensorDistance", Storage.sensor.getDistance(DistanceUnit.CM));

        }
    }
}
