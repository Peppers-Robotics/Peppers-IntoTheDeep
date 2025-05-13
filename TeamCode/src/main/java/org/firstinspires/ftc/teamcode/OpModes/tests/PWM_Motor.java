package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Config
@TeleOp
public class PWM_Motor extends LinearOpMode {
    public static double qs = 800;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor speaker = new DcMotorImpl(hardwareMap.get(DcMotorController.class, "Control Hub"), 0);
        ElapsedTime t = new ElapsedTime();
        waitForStart();

        while(opModeIsActive()){
            if(1.f / qs <= t.seconds()) {
                speaker.setPower(1 - speaker.getPower());
                t.reset();
            }
        }
    }
}
