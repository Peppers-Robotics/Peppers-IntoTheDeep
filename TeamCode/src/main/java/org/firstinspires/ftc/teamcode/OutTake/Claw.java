package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.Storage;

@Config
public class Claw {
    public static double Closed = 0, Opened = 90;
    public static ServoPlus servo;
    public static FastColorRangeSensor sensor;
    private static boolean closed = false;

    public static void Close(){
        servo.setAngle(Closed);
        closed = true;
    }
    public static void Open(){
        servo.setAngle(Opened);
        closed = false;
    }
    public static boolean isClosed(){
        return closed; // avoid servo.getAngle() because java and double funny
    }
    public static Storage.PiceColor GetSensorStatus(){
        switch (sensor.getColorSeenBySensor()){
            case YELLOW: return Storage.PiceColor.YELLOW;
            case RED: return Storage.PiceColor.RED;
            case BLUE: return Storage.PiceColor.BLUE;
            default: return Storage.PiceColor.NONE;
        }
    }
    public static boolean isEmpty(){
        return GetSensorStatus() == Storage.PiceColor.NONE;
    }
}
