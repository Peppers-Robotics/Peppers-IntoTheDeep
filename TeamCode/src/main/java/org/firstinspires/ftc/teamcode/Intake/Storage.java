package org.firstinspires.ftc.teamcode.Intake;

import android.hardware.camera2.params.BlackLevelPattern;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;

@SuppressWarnings("unused")
public class Storage {
    public enum SpecimenType {
        BLUE,
        RED,
        YELLOW,
        NONE
    }
    public enum Team {
        RED,
        BLUE
    }
    public static Team team = Team.RED;
    public static FastColorRangeSensor sensor;

    public static boolean isStorageEmpty(){
        return getStorageStatus() == SpecimenType.NONE;
    }

    public static boolean hasWrongPice(){
        switch (team){
            case RED:
                return getStorageStatus() == SpecimenType.BLUE || isStorageEmpty();
            case BLUE:
                return isStorageEmpty() || getStorageStatus() == SpecimenType.RED;
        }
        return false;
    }
    public static boolean hasTeamPice(){
        if(getStorageStatus() == SpecimenType.YELLOW) return true;
        switch (team){
            case RED:
                return getStorageStatus() == SpecimenType.RED;
            case BLUE:
                return getStorageStatus() == SpecimenType.BLUE;
        }
        return false;
    }

    public static SpecimenType getStorageStatus(){
//        return SpecimenType.YELLOW;
        if(sensor.getDistance(DistanceUnit.CM) >= 2.5) return SpecimenType.NONE;

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
