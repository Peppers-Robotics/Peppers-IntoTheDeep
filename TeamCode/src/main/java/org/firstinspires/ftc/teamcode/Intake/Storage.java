package org.firstinspires.ftc.teamcode.Intake;

import android.hardware.camera2.params.BlackLevelPattern;
import android.widget.GridLayout;

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
    public static FastColorRangeSensor sensor1,sensor2;

    public static boolean isStorageEmpty(){
        return getStorageStatus() == SpecimenType.NONE;
    }

    public static boolean hasWrongPice(){
        SpecimenType t = getStorageStatus();
        switch (team){
            case RED:
                return t == SpecimenType.BLUE || t == SpecimenType.NONE;
            case BLUE:
                return t == SpecimenType.RED || t == SpecimenType.NONE;
        }
        return false;
    }
    public static boolean hasTeamPice(){
        SpecimenType t = getStorageStatus();
        if(t == SpecimenType.YELLOW) return true;
        switch (team){
            case RED:
                return t == SpecimenType.RED;
            case BLUE:
                return t == SpecimenType.BLUE;
        }
        return false;
    }

    public static SpecimenType getStorageStatus(){
//        return SpecimenType.YELLOW;
        //sensor2 cel mai aproape de iesire
        //sensor1 cel din capat
        if(sensor1.getDistance(DistanceUnit.CM) >= 3.6) return SpecimenType.NONE;

        switch (sensor1.getColorSeenBySensor()){
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
