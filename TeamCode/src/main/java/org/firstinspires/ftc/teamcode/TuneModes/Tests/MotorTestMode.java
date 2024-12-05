package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(group = "tests")
public class MotorTestMode extends LinearOpMode {
    public static int port = 0;
    public static double power = 0;
    public static ServoTesterMode.Hub hub = ServoTesterMode.Hub.CONTROL_HUB;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorController chController = hardwareMap.getAll(DcMotorControllerEx.class).get(0);
        DcMotorController ehController = hardwareMap.getAll(DcMotorControllerEx.class).get(1);
        for(int i = 0; i < 4; i++){
            chController.setMotorMode(i, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ehController.setMotorMode(i, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chController.setMotorMode(i, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ehController.setMotorMode(i, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();

        while (opModeIsActive()){
            if(port < 0) port = 0;
            if(port > 3) port = 3;
            for(int i = 0; i < 4; i++){
                if(i != port) {
                    chController.setMotorPower(i, 0);
                    ehController.setMotorPower(i, 0);
                }
            }
            switch (hub){
                case CONTROL_HUB:
                    chController.setMotorPower(port, power);
                    break;
                case EXPANSION_HUB:
                    ehController.setMotorPower(port, power);
                    break;
            }
            for(int i = 0; i < 4; i++){
                telemetry.addData("ControlHub port" + Integer.toString(i) + " encoder value", chController.getMotorCurrentPosition(i));
                DcMotorEx m = new DcMotorImplEx(chController, i);
                telemetry.addData("ControlHub port" + i + " power", m.getCurrent(CurrentUnit.MILLIAMPS));
            }

            for(int i = 0; i < 4; i++){
                telemetry.addData("ExpansionHub port" + Integer.toString(i) + " encoder value", ehController.getMotorCurrentPosition(i));
                DcMotorEx m = new DcMotorImplEx(ehController, i);
                telemetry.addData("ExpansionHub port" + i + " power", m.getCurrent(CurrentUnit.MILLIAMPS));
            }
            telemetry.update();
        }
    }
}

