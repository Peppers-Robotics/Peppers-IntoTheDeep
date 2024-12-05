package org.firstinspires.ftc.teamcode.OutTake;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;

@SuppressWarnings("unused")
public class Claw {
    public static ServoPlus clawServo;
    public static FastColorRangeSensor clawSensor;
    public static double OpenPosition = 100, ClosePosition = 255;
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
        return clawSensor.getDistance(DistanceUnit.MM) <= 50 &&
                (clawSensor.getColorSeenBySensor() ==  Colors.ColorType.YELLOW ||
                 clawSensor.getColorSeenBySensor() == Colors.ColorType.RED ||
                 clawSensor.getColorSeenBySensor() == Colors.ColorType.BLUE );
    }
}
