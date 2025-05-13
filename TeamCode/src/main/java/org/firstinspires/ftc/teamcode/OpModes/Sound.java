package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class Sound extends LinearOpMode {
    public static double freq = 440, power = 0.3;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor speaker = new DcMotorImpl(hardwareMap.get(DcMotorController.class, "Control Hub"), 0);
        ElapsedTime t = new ElapsedTime();

        waitForStart();

        t.reset();
        speaker.setPower(power);
        while(opModeIsActive()){
            if(t.seconds() >= 2.f / freq){
                speaker.setPower(speaker.getPower() * -1);
                t.reset();
            }
        }
    }
}
