package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(group = "tests")
@Config
public class ServoTesterMode extends LinearOpMode {
    public enum ServoPorts{
        S0,
        S1,
        S2,
        S3,
        S4,
        S5
    }
    public enum Hub{
        CONTROL_HUB,
        EXPANSION_HUB
    }
    public static ServoPorts port = ServoPorts.S0;
    public static Hub hub = Hub.CONTROL_HUB;
    private int getPortBySP(ServoPorts p){
        switch (p){
            case S0: return 0;
            case S1: return 1;
            case S2: return 2;
            case S3: return 3;
            case S4: return 4;
            case S5: return 5;
        }
        return 0;
    }
    public static double maxAngle = 355, angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoController chubController = hardwareMap.get(Servo.class, "cS0").getController();
        ServoController ehubController = hardwareMap.get(Servo.class, "eS0").getController();
//        chubController.pwmEnable();
//        ehubController.pwmEnable();
        waitForStart();

        while (opModeIsActive()){
            if(hub == Hub.CONTROL_HUB){
                chubController.setServoPosition(getPortBySP(port), angle / maxAngle);
            } else {
                ehubController.setServoPosition(getPortBySP(port), angle / maxAngle);
            }
        }
    }
}
