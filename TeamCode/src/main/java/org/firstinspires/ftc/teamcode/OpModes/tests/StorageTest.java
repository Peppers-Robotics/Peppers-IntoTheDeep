package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class StorageTest extends LinearOpMode {
    public static boolean climb = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeStorage(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            Robot.clearCache();
            Storage.getStorageStatus();

            Robot.telemetry.addData("r, g, b", Storage.sensor.RGB.R + ", " + Storage.sensor.RGB.G + ", " + Storage.sensor.RGB.B);
        }
    }
}
