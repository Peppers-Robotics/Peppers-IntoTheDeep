package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HelperClasses.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.Intake.Storage;

@Config
@TeleOp(group = "tests")
public class StorageTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Storage.sensor = hardwareMap.get(FastColorRangeSensor.class, "Storage");
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Sample color inside", Storage.getStorageStatus().toString());
            telemetry.addData("Storage empty", Storage.isStorageEmpty());

            telemetry.update();
        }
    }
}
