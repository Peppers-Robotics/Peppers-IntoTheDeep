package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class TestColorSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FastColorRangeSensor sensor = hardwareMap.get(FastColorRangeSensor.class, "Storage");
        Robot.InitializeStorage(hardwareMap);
        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("color", Storage.getStorageStatus());
            telemetry.addData("raw color R",sensor.RGB.R);
            telemetry.addData("raw color G",sensor.RGB.G);
            telemetry.addData("raw color B",sensor.RGB.B);
            telemetry.addData("raw color B",sensor.getDistance(DistanceUnit.CM));

            telemetry.update();
        }
    }
}
