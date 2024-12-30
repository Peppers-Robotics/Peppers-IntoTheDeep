package org.firstinspires.ftc.teamcode.Intake;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;

public class Storage {
    public static FastColorRangeSensor sensor;
    public enum PiceColor {
        BLUE,
        RED,
        YELLOW,
        NONE
    }
    public static PiceColor StorageSampleType(){
        switch (sensor.getColorSeenBySensor()){
            case YELLOW: return PiceColor.YELLOW;
            case RED: return PiceColor.RED;
            case BLUE: return PiceColor.BLUE;
            default: return PiceColor.NONE;
        }
    }
    public static boolean IsStorageEmpty(){
        return StorageSampleType() == PiceColor.NONE;
    }
}
