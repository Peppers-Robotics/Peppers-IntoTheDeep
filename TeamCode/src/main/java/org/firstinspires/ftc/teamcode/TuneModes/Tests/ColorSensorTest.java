package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;

@Config
@TeleOp(group = "tests")
public class ColorSensorTest extends LinearOpMode {
    public static BroadcomColorSensor.LEDCurrent current = BroadcomColorSensor.LEDCurrent.CURRENT_100mA;
    public static BroadcomColorSensor.LEDPulseModulation modulation = BroadcomColorSensor.LEDPulseModulation.LED_PULSE_100kHz;
    public static boolean update = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FastColorRangeSensor sensor = hardwareMap.get(FastColorRangeSensor.class, "Storage");

        waitForStart();

        while (opModeIsActive()){
            if(update){
                sensor.changeLEDsettings(modulation, current);
                update = false;
            }

            telemetry.addData("Color detected", sensor.getColorSeenBySensor().toString());
            telemetry.addData("red value", sensor.p.R);
            telemetry.addData("green value", sensor.p.G);
            telemetry.addData("blue value", sensor.p.B);
            telemetry.addData("distance value", sensor.getDistance(DistanceUnit.CM));
            telemetry.addData("(r, g, b)", sensor.RGB.toString());
            telemetry.addData("alpha", sensor.p.A);
            telemetry.update();
        }
    }
}
