package org.firstinspires.ftc.teamcode.HelperClasses.Devices;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@I2cDeviceType
@DeviceProperties(
        name = "Lazy BHI260IMU",
        xmlTag = "lazy-bhi260imu"
)
public class LazyIMUBHI extends BHI260IMU implements HardwareDevice {
    private long timeHeading = 0, timeVelocity = 0;
    private double freq = 0.5;
    private double lastAngleRead = 0, lastVeloRead = 0;
    public LazyIMUBHI(I2cDeviceSynchSimple i2cDeviceSynchSimple, boolean deviceClientIsOwned) {
        super(i2cDeviceSynchSimple, deviceClientIsOwned);
        timeHeading = 0;
        timeVelocity = 0;
    }
    public void setFreqRead(double freq){
        this.freq = freq;
    }

    public double getHeading(AngleUnit unit){
        if(TimeUnit.MILLISECONDS.toSeconds(System.currentTimeMillis() - timeHeading) < 1 / freq) return lastAngleRead;
        timeHeading = System.currentTimeMillis();

        lastAngleRead = getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return unit.fromUnit(AngleUnit.RADIANS, lastAngleRead);
    }
    public double getHeadingVelocity(AngleUnit unit){
        if(TimeUnit.MILLISECONDS.toSeconds(System.currentTimeMillis() - timeVelocity) < 1 / freq) return lastVeloRead;
        timeVelocity = System.currentTimeMillis();
        lastVeloRead = getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate;

        return lastVeloRead;
    }
}
