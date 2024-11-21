package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.HelperClasses.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.Initialization;

@SuppressWarnings("unused")
public class Storage {
    public enum SpecimenType {
        BLUE,
        RED,
        YELLOW,
        NONE
    }
    public static FastColorRangeSensor sensor;

    public static boolean isStorageEmpty(){
        return getStorageStatus() == SpecimenType.NONE;
    }

    public static boolean hasAlliancePice(){
        if(getStorageStatus() == SpecimenType.NONE) return false;
        switch (Initialization.Team){
            case RED:
                return getStorageStatus() != SpecimenType.BLUE;
            case BLUE:
                return getStorageStatus() != SpecimenType.RED;
        }
        return false;
    }

    public static SpecimenType getStorageStatus(){
        switch (sensor.getColorSeenBySensor()){
            case RED:
                return SpecimenType.RED;
            case BLUE:
                return SpecimenType.BLUE;
            case YELLOW:
                return SpecimenType.YELLOW;
            default:
                return SpecimenType.NONE;
        }
    }
}
