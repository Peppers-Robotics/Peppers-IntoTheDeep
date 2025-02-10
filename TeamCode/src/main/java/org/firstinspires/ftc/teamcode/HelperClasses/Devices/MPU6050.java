package org.firstinspires.ftc.teamcode.HelperClasses.Devices;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "MPU6050 IMU", description = "InvenSense MPU6050 IMU", xmlTag = "MPU6050")
public class MPU6050 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // MPU6050 Registers
    private static final int MPU6050_ADDRESS = 0x68;
    private static final int PWR_MGMT_1 = 0x6B;
    private static final int ACCEL_XOUT_H = 0x3B;
    private static final int ACCEL_YOUT_H = 0x3D;
    private static final int ACCEL_ZOUT_H = 0x3F;
    private static final int GYRO_XOUT_H = 0x43;
    private static final int GYRO_YOUT_H = 0x45;
    private static final int GYRO_ZOUT_H = 0x47;

    public MPU6050(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(MPU6050_ADDRESS));
        try {
            ((LynxI2cDeviceSynch) (this.deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        } catch (Exception ignored){}

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        // Wake up the MPU6050 by writing 0 to the PWR_MGMT_1 register
        deviceClient.write8(PWR_MGMT_1, 0);
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MPU6050 IMU";
    }

    public short getAccelX() {
        return TypeConversion.byteArrayToShort(deviceClient.read(ACCEL_XOUT_H, 2));
    }

    public short getAccelY() {
        return TypeConversion.byteArrayToShort(deviceClient.read(ACCEL_YOUT_H, 2));
    }

    public short getAccelZ() {
        return TypeConversion.byteArrayToShort(deviceClient.read(ACCEL_ZOUT_H, 2));
    }

    public short getGyroX() {
        return TypeConversion.byteArrayToShort(deviceClient.read(GYRO_XOUT_H, 2));
    }

    public short getGyroY() {
        return TypeConversion.byteArrayToShort(deviceClient.read(GYRO_YOUT_H, 2));
    }

    public short getGyroZ() {
        return TypeConversion.byteArrayToShort(deviceClient.read(GYRO_ZOUT_H, 2));
    }

    public double getAccelXInG() {
        return getAccelX() / 16384.0;
    }

    public double getAccelYInG() {
        return getAccelY() / 16384.0;
    }

    public double getAccelZInG() {
        return getAccelZ() / 16384.0;
    }

    public double getGyroXInDegreesPerSecond() {
        return getGyroX() / 131.0;
    }

    public double getGyroYInDegreesPerSecond() {
        return getGyroY() / 131.0;
    }

    public double getGyroZInDegreesPerSecond() {
        return getGyroZ() / 131.0;
    }
}