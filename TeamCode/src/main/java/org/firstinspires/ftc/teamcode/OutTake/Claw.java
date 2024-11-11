package org.firstinspires.ftc.teamcode.OutTake;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.Storage;

@SuppressWarnings("unused")
public class Claw {
    public static ServoPlus clawServo;
    public static FastColorRangeSensor clawSensor;
    public static double OpenPosition = 0, ClosePosition = 0;
    public static void open(){
        clawServo.setPosition(OpenPosition);
    }
    public static void close(){
        clawServo.setPosition(ClosePosition);
    }
    public static boolean isClosed(){
        return clawServo.getPosition() != OpenPosition;
    }
    public static boolean HasElementInIt(){ return clawSensor.getDistance(DistanceUnit.MM) <= 20; }
}
