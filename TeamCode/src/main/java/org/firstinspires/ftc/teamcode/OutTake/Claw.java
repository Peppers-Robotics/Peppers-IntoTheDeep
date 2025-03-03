package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;

import java.util.concurrent.TimeUnit;

@SuppressWarnings("unused")
@Config
public class Claw {
    public static ServoPlus clawServo;
    public static double OpenPosition = 120, ClosePosition = 50, mmClose = 70;
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
}
