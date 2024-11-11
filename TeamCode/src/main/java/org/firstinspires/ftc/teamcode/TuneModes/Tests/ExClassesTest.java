package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;

@TeleOp(group = "tests")
public class ExClassesTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ServoPlus s1 = hardwareMap.get(ServoPlus.class, "s1");
        CachedMotor c1 = new CachedMotor( hardwareMap.get(DcMotorEx.class, "m1"));
        s1.setAngle(0);
        c1.setPower(0);

        waitForStart();

        while (opModeIsActive()){
            s1.setAngle(180);
            c1.setPower(1);
        }
    }
}
