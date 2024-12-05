package org.firstinspires.ftc.teamcode.HelperClasses.Devices;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;

import java.awt.font.NumericShaper;


@I2cDeviceType
@DeviceProperties(
        xmlTag = "FastColorRangeSensor",
        name = "Fast ColorRangeSensor - REV V3"
)
public class FastColorRangeSensor extends RevColorSensorV3 implements HardwareDevice {
    public class ColorRangeSensorPacket {
        public double R, G, B, A;
        public double D;
        public ColorRangeSensorPacket(){
            R = G = B = 0;
            D = 0;
        }
        @Override
        public String toString(){
            return Double.toString(R) + ' ' + Double.toString(G) + ' ' + Double.toString(B);
        }
    }
    private long timeDistance = 0, timeRGB = 0;
    private double freq = 20;
    private double lowPassFilter = 0.6;
    public ColorRangeSensorPacket p = new ColorRangeSensorPacket();
    public ColorRangeSensorPacket RGB = new ColorRangeSensorPacket();
    public FastColorRangeSensor(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        timeDistance = System.currentTimeMillis();
        timeRGB = System.currentTimeMillis();
        setFreqToUpdate(50);
    }
    public void changeLEDsettings(LEDPulseModulation l, LEDCurrent c){
        setLEDParameters(l, c);
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

    public Colors.ColorType getColorSeenBySensor(){
        if(System.currentTimeMillis() - timeRGB > freq){
            p.G = (int) (this.green() * this.lowPassFilter + p.G * (1 - lowPassFilter));
            p.R = (int) (this.red()   * this.lowPassFilter + p.R * (1 - lowPassFilter));
            p.B = (int) (this.blue()  * this.lowPassFilter + p.B * (1 - lowPassFilter));
            p.A = Math.max(p.G, Math.max(p.R, p.B));

            RGB.R = Range.clip((p.R / p.A) * 255, 0, 255);
            RGB.G = Range.clip((p.G / p.A) * 255, 0, 255);
            RGB.B = Range.clip((p.B / p.A) * 255, 0, 255);

            timeRGB = System.currentTimeMillis();
        }
        return Colors.getColorFromRGB(new Colors.Color(RGB.R, RGB.G, RGB.B));

    }
}