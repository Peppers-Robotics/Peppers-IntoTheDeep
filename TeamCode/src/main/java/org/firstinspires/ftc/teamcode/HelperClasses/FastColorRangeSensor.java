package org.firstinspires.ftc.teamcode.HelperClasses;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

class ColorRangeSensorPacket {
    public int R, G, B;
    public double D;
    public ColorRangeSensorPacket(){
        R = G = B = 0;
        D = 0;
    }
}

@I2cDeviceType
@DeviceProperties(
        xmlTag = "FastColorRangeSensor",
        name = "Fast ColorRangeSensor - REV V3"
)
public class FastColorRangeSensor extends RevColorSensorV3 implements HardwareDevice {
    private long timeDistance = 0, timeRGB = 0;
    private double freq = 20;
    private double lowPassFilter = 0.8;
    public ColorRangeSensorPacket p = new ColorRangeSensorPacket();
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
    public void setLowPassFilterCoefficient(double k){
        this.lowPassFilter = k;
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

    public Colors.ColorType getColorSeenBySensor(){
        if(System.currentTimeMillis() - timeRGB > freq){
            p.G = (int) (this.green() * this.lowPassFilter + p.G * (1 - lowPassFilter));
            p.R = (int) (this.red()   * this.lowPassFilter + p.R * (1 - lowPassFilter));
            p.B = (int) (this.blue()  * this.lowPassFilter + p.B * (1 - lowPassFilter));
            timeRGB = System.currentTimeMillis();
        }
        return Colors.getColorFromRGB(new Colors.Color(p.R, p.G, p.B));

    }
}
