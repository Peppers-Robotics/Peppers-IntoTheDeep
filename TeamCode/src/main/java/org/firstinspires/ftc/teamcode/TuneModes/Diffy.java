package org.firstinspires.ftc.teamcode.TuneModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.OutTake.Arm;

@Config
@TeleOp
public class Diffy extends LinearOpMode {
    public static double ArmAngle = 180, PivotAngle = 180;
    @Override
    public void runOpMode() throws InterruptedException {
        Arm.servo1 = hardwareMap.get(ServoPlus.class, "cS0");
        Arm.servo2 = hardwareMap.get(ServoPlus.class, "cS1");
        Arm.servo1.setAngle(180);
        Arm.servo1.setAngle(180);

        waitForStart();

        while (isStarted() && !isStopRequested()){
            Arm.setArmAngle(ArmAngle);
            Arm.setPivotAngle(PivotAngle);

            Arm.update();
        }
    }
}
