package org.firstinspires.ftc.teamcode.HelperClasses;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;

class ColorRangeSensorPacket {
    public int R, G, B;
    public double D;
    public ColorRangeSensorPacket(){
        R = G = B = 0;
        D = 0;
    }
}

public class FastColorRangeSensor extends RevColorSensorV3 {
    public enum Colors{
        RED,
        BLUE,
        GREEN,
        WHITE,
        YELLOW,
        BLACK,
        OTHER
    }
    private long timeDistance = 0, timeRGB = 0;
    private double freq = 50;
    ColorRangeSensorPacket p;
    public FastColorRangeSensor(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        timeDistance = System.currentTimeMillis();
        timeRGB = System.currentTimeMillis();
        setFreqToUpdate(50);
    }
    // in reads / s
    public void setFreqToUpdate(double x){
        freq =  x;
    }
    @Override
    public double getDistance(DistanceUnit unit){
        if(System.currentTimeMillis() - timeDistance > freq){
            p.D = unit.fromUnit(DistanceUnit.INCH, this.inFromOptical(this.rawOptical()));
            timeDistance = System.currentTimeMillis();
        }
        return p.D;
    }

    public static double[] rgbToHSV(int[] rgb){
        double[] ret = new double[3];

        double r = rgb[0] / 255.f, g = rgb[1] / 255.f, b = rgb[2] / 255.f;
        double max = Math.max(Math.max(r, g), b);
        double min = Math.min(Math.min(r, g), b);
        double delta = max - min;

        if(delta == 0) ret[0] = 0;
        else if(max == r) ret[0] = 60 * ((g - b) / delta % 6);
        else if(max == g) ret[0] = 60 * ((b - r) / delta + 2);
        else if(max == b) ret[0] = 60 * ((r - g) / delta + 4);

        if(delta == 0) ret[1] = 0;
        else ret[1] = delta / max;

        ret[2] = max;

        return ret;
    }
    private static boolean isInRage(double x, double lowerBound, double upperBound){
        return x <= upperBound && x >= lowerBound;
    }

    public Colors getColorSeenBySensor(){
        if(System.currentTimeMillis() - timeRGB > freq){
            p.G = this.green();
            p.B = this.red();
            p.R = this.blue();
        }
        double[] hsv = rgbToHSV(new int[]{p.R, p.G, p.B});

        if(isInRage(hsv[0], 50, 65) && isInRage(hsv[1], 75, 100) && isInRage(hsv[2], 50, 100)){
            return Colors.YELLOW;
        }

        if(isInRage(hsv[0], 200, 255) && isInRage(hsv[1], 65, 100) && isInRage(hsv[2], 50, 100)){
            return Colors.BLUE;
        }

        hsv = rgbToHSV(new int[]{p.B, p.G, p.R});

        if(isInRage(hsv[0], 200, 255) && isInRage(hsv[1], 65, 100) && isInRage(hsv[2], 50, 100)){
            return Colors.RED;
        }


        return Colors.OTHER;
    }
}
