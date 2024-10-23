package org.firstinspires.ftc.teamcode.TuneModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
@Config
@TeleOp
public class Diffy extends LinearOpMode {
    public Servo r, l;

    public static double stage1Angle = 0, stage2Angle = 0;
    private static double initPos1 = 180, initPos2 = 180;
    public static double ratio = 40.0 / 24.f;
    @Override
    public void runOpMode() throws InterruptedException {
        r = hardwareMap.get(Servo.class, "r");
        l = hardwareMap.get(Servo.class, "l");

        r.setPosition(initPos1 / 360 * ratio);
        l.setPosition(initPos2 / 360 * ratio);
        waitForStart();

        while (opModeIsActive()){
            double lAngle = initPos1 + stage1Angle + stage2Angle;
            double rAngle = initPos2 + stage2Angle - stage1Angle;
            r.setPosition(rAngle / 360 * ratio);
            l.setPosition(lAngle / 360 * ratio);
        }
    }
}
