package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class MotorBetterTestMode extends LinearOpMode {
    public static String motor = "cM0";
    public static double p = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()){
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, motor);
            m.setPower(p);
        }
    }
}
