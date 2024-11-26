package org.firstinspires.ftc.teamcode.TuneModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;

@TeleOp(group = "test")
@Config
public class AnalogServoTest extends LinearOpMode {
    public static int analogPort = 0, servoPort = 0;
    public static double position = 0;
    public static boolean isChub = true;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ServoController sc = hardwareMap.getAll(ServoController.class).get(isChub ? 0 : 1);
        AnalogInputController ac = hardwareMap.getAll(AnalogInputController.class).get(isChub ? 0 : 1);
//        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "cA2");

        double lastAngle = 0, currentAngle = 0;
        int rev = 0;

        waitForStart();
        while (opModeIsActive()){
            sc.setServoPosition(servoPort, position);
            currentAngle = ac.getAnalogInputVoltage(analogPort) / 3.3 * 360;
//            currentAngle = encoder.getVoltage() / 3.3f * 360.f;
            if(lastAngle < 90 && currentAngle > 270){
                rev--;
            }
            if(currentAngle < 90 && lastAngle > 270) {
                rev++;
            }
            lastAngle = currentAngle;
            telemetry.addData("Servo pos", currentAngle);
            telemetry.addData("Servo correctedpos", (currentAngle + 360 * rev));
            telemetry.addData("Revolutions", rev);

            telemetry.addData("cA0", ac.getAnalogInputVoltage(0));
            telemetry.addData("cA1", ac.getAnalogInputVoltage(1));
            telemetry.addData("cA2", ac.getAnalogInputVoltage(2));
            telemetry.addData("cA3", ac.getAnalogInputVoltage(3));
            telemetry.update();
        }
    }
}
