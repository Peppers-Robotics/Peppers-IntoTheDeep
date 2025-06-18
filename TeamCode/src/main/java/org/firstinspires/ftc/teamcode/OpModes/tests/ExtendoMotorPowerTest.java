package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;

@TeleOp
public class ExtendoMotorPowerTest extends LinearOpMode {
    public static final String fileName = "ExtendoPowerVsPos.cps";

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeExtendo();
        Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        File f = new File("/sdcard/FIRST/" + fileName);
        OutputStream o;
        try {
            o = new FileOutputStream(f);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        waitForStart();
        Extendo.motor.setPower(1);
        while (opModeIsActive() && Extendo.encoder.getCurrentPosition() <= Extendo.getMaxPosition()){
            Robot.clearCache();
            try {
                o.write(Integer.toString(Extendo.encoder.getCurrentPosition()).getBytes());
                o.write(" ".getBytes());
                o.write(Double.toString(Extendo.motor.getCurrent(CurrentUnit.AMPS)).getBytes());
                o.write("\n".getBytes());
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
        try {
            o.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }
}
