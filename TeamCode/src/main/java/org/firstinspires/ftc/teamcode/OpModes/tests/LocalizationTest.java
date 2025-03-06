package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeLocalizer(hardwareMap);
        Robot.telemetry = new MultipleTelemetry(telemetry, Robot.telemetry);
        waitForStart();

        while(opModeIsActive()){
            Robot.clearCache();
            Localizer.Update();
            Robot.telemetry.addData("pinpoint freq", Localizer.pinPoint.getFrequency());
        }
    }
}
