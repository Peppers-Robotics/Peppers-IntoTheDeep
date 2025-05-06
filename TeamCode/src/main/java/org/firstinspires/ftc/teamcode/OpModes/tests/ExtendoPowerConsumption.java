package org.firstinspires.ftc.teamcode.OpModes.tests;

import android.annotation.SuppressLint;
import android.content.Context;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;

@Config
@TeleOp
public class ExtendoPowerConsumption extends LinearOpMode {
    public static String FileName = "ExtendoPowerVsPos", FileName2 = "ExtendoPowerVsTime";
    @SuppressLint("SdCardPath")
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeExtendo();
        Extendo.DISABLE = false;
        Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, Robot.telemetry);
        while(opModeInInit()){
            telemetry.addLine("Press △/Y to reset Extendo Encoder");
            telemetry.addLine("Press Start to start the test");
            telemetry.update();
            if(gamepad1.triangle || gamepad1.y){
                Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        StringBuilder data1 = new StringBuilder();
        StringBuilder data2 = new StringBuilder();
        Extendo.Extend(Extendo.getMaxPosition());
        ElapsedTime time = new ElapsedTime();
        while(opModeIsActive() && Extendo.getCurrentPosition() < Extendo.getMaxPosition()){
            Robot.clearCache(false);
            data1.append(Extendo.motor.getCurrent(CurrentUnit.AMPS)).append(" ").append(Extendo.getCurrentPosition()).append("\n");
            data2.append(Extendo.motor.getCurrent(CurrentUnit.AMPS)).append(" ").append(time.seconds()).append("\n");
            telemetry.addData("Extendo pos", Extendo.getCurrentPosition());
            telemetry.update();
            Extendo.update();
        }
        try {
            @SuppressLint("SdCardPath") File file = new File("/sdcard/FIRST", FileName);
            FileOutputStream stream = new FileOutputStream(file);
            byte data[] = data1.toString().getBytes(StandardCharsets.UTF_8);
            stream.write(data);
            stream.close();

            file = new File("/sdcard/FIRST", FileName2);
            stream = new FileOutputStream(file);
            data = data2.toString().getBytes(StandardCharsets.UTF_8);
            stream.write(data);
            telemetry.update();
            stream.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
