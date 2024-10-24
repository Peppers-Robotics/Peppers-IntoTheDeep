package org.firstinspires.ftc.teamcode.TuneModes;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;

@TeleOp
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
