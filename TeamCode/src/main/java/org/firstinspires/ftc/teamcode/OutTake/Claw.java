package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;

@SuppressWarnings("unused")
@Config
public class Claw {
    public static ServoPlus clawServo;
    public static FastColorRangeSensor clawSensor;
    public static double OpenPosition = 120, ClosePosition = 255, mmClose = 37;
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
    public static boolean HasElementInIt(){
        Initialization.telemetry.addData("sensor distance", clawSensor.getDistance(DistanceUnit.MM));
        return clawSensor.getDistance(DistanceUnit.MM) <= mmClose;
    }
}
