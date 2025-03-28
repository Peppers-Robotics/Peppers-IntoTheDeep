package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if(hub == Hubs.ControlHub){
                Robot.ControlHubServos.setServoPosition(port, angle / 355.f);
            } else if(hub == Hubs.ExpansionHub) {
                Robot.ExpansionHubServos.setServoPosition(port, angle / 355.f);
            } else {
                Robot.ServoHub.setServoPosition(port, angle / 355.f);
            }
        }
    }

    public enum Hubs{
        ControlHub,
        ExpansionHub,
        ServoHub
    }
    public static Hubs hub = Hubs.ControlHub;
    public static int port = 0;
    public static double angle = 0;


}
