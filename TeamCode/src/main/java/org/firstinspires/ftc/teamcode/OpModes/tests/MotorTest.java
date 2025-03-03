package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class MotorTest extends LinearOpMode {
    public static int motor = 0;
    public static double power = 0;
    public static ServoTest.Hubs hub = ServoTest.Hubs.ControlHub;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()){
            if(hub == ServoTest.Hubs.ControlHub){
                Robot.ControlHubMotors.setMotorPower(motor, power);
            } else {
                Robot.ExpansionHubMotors.setMotorPower(motor, power);
            }

            for(int i = 0; i < 4; i++){
                telemetry.addData("c" + i, Robot.ControlHubMotors.getMotorCurrentPosition(i));
                telemetry.addData("e" + i, Robot.ExpansionHubMotors.getMotorCurrentPosition(i));
            }
            Robot.clearCache();

            telemetry.update();
        }
    }
}
