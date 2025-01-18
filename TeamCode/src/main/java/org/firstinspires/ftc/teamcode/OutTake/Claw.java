package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;

import java.util.concurrent.TimeUnit;

@SuppressWarnings("unused")
@Config
public class Claw {
    public static ServoPlus clawServo;
    public static Rev2mDistanceSensor clawSensor;
    public static double OpenPosition = 120, ClosePosition = 185, mmClose = 70;
    public static void open(){
        clawServo.setAngle(OpenPosition);
    }
    public static void close(){
        clawServo.setAngle(ClosePosition);
    }
    public static boolean isClosed(){
        return clawServo.isEqualToAngle(ClosePosition);
    }
    public static boolean isOpened(){
        return clawServo.isEqualToAngle(OpenPosition);
    }
    private static long time = 0;
    private static double distance = 20;
    public static boolean HasElementInIt() {
        if (TimeUnit.MILLISECONDS.toSeconds(System.currentTimeMillis() - time) >= 1.f / 20) {
            time = System.currentTimeMillis();
//            distance = clawSensor.getDistance(DistanceUnit.MM);
        }
//        Initialization.telemetry.addData("sensor distance", distance);
        return false;
    }
}
