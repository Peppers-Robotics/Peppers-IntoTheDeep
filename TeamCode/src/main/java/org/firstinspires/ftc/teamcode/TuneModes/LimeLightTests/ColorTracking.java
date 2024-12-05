package org.firstinspires.ftc.teamcode.TuneModes.LimeLightTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.LimeLightHelpers.LimeLightColorTracking;
import org.firstinspires.ftc.teamcode.Initialization;

@TeleOp
@Config
public class ColorTracking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeLimeLight();
        LimeLightColorTracking.StartCamera();

        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("Forward", LimeLightColorTracking.moveForwardExtendo());
            telemetry.addData("Rotate", LimeLightColorTracking.rotateDegree());
            telemetry.addData("Tx", LimeLightColorTracking.getTx());
            telemetry.addData("Ty", LimeLightColorTracking.getTy());
            telemetry.addData("Distance", LimeLightColorTracking.getDistance());
            telemetry.update();
        }
    }
}
