package org.firstinspires.ftc.teamcode.OutTake;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Storage;

@SuppressWarnings("unused")
public class Claw {
    public static ServoPlus clawServo;
    public static FastColorRangeSensor clawSensor;
    public static double OpenPosition = 200, ClosePosition = 355;
    public static void open(){
        clawServo.setAngle(OpenPosition);
    }
    public static void close(){
        clawServo.setAngle(ClosePosition);
    }
    public static boolean isClosed(){
        return clawServo.getAngle() == ClosePosition;
    }
    public static boolean HasElementInIt(){
        return clawSensor.getDistance(DistanceUnit.MM) <= 50 &&
                (clawSensor.getColorSeenBySensor() ==  Colors.ColorType.YELLOW ||
                 clawSensor.getColorSeenBySensor() == Colors.ColorType.RED ||
                 clawSensor.getColorSeenBySensor() == Colors.ColorType.BLUE );
    }
}
