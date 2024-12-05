package org.firstinspires.ftc.teamcode.HelperClasses.Devices;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.bosch.BNO055Util;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@I2cDeviceType
@DeviceProperties(
        name = "Lazy BNO055 IMU",
        xmlTag = "lazy-bno055imu"
)
public class LazyIMUBNO055 extends BNO055IMUNew implements HardwareDevice {
    private long timeHeading = 0, timeVelocity = 0;
    private double freq = 2;
    private double lastAngleRead = 0, lastVeloRead = 0;
    public LazyIMUBNO055(I2cDeviceSynchSimple i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
        timeHeading = 0;
        timeVelocity = 0;
    }

    @Override
    public String getDeviceName() {
        return "Lazy IMU BNO055";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
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
