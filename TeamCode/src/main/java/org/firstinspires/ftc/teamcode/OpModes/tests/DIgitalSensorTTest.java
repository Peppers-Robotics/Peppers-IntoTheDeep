package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.LimitSwitch;


@Config
@TeleOp
public class DIgitalSensorTTest extends LinearOpMode {
    public DigitalChannelController ControlHubDigital;
    public LimitSwitch lm;
    public static int port = 0;
    

    @Override
    public void runOpMode() throws InterruptedException {

        ControlHubDigital = hardwareMap.get(DigitalChannelController.class,"Control Hub");
        waitForStart();

        while(opModeIsActive())
        {
            lm = new LimitSwitch(ControlHubDigital,port);
            lm.setMode(DigitalChannel.Mode.INPUT);

            telemetry.addData("result",lm.getState());
            telemetry.update();
        }
    }


}
