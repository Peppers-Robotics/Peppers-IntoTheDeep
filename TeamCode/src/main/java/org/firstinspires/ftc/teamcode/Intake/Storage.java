package org.firstinspires.ftc.teamcode.Intake;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
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
    public static boolean wrongPice(){
        if(getStorageStatus() == SpecimenType.NONE) return false;
        return !hasAlliancePice();
    }

    public static SpecimenType getStorageStatus(){
        if(sensor.getDistance(DistanceUnit.CM) > 5) return SpecimenType.NONE;

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
