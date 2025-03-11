package org.firstinspires.ftc.teamcode.HelperClasses.Devices;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.concurrent.TimeUnit;

@I2cDeviceType
@DeviceProperties(
        name = "BNO085 IMU",
        xmlTag = "bno085imu"
)
public class IMUBNO085 extends BNO055IMUNew implements HardwareDevice {
    private double lastAngleRead = 0, lastVeloRead = 0;
    private static int digital = 0;
    public static DigitalChannelController controller;
    public IMUBNO085(I2cDeviceSynchSimple i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    @Override
    public String getDeviceName() {
        return "IMU BNO085";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    private YawPitchRollAngles cache = null;

    public YawPitchRollAngles getOrientation(){
//        if(cache == null) {
        cache = getRobotYawPitchRollAngles();
//            return cache;
//        }
//        if(!controller.getDigitalChannelState(digital)){
//            cache = getRobotYawPitchRollAngles();
//        }
        return cache;
    }

}
