package org.firstinspires.ftc.teamcode.HelperClasses.Devices;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(
        xmlTag = "MPU9050", name = "Adafruit-MPU9050"
)
public class MPU9050 extends I2cDeviceSynchDeviceWithParameters implements HardwareDevice {

    protected MPU9050(I2cDeviceSynchSimple i2cDeviceSynchSimple, boolean deviceClientIsOwned, @NonNull Object defaultParameters) {
        super(i2cDeviceSynchSimple, deviceClientIsOwned, defaultParameters);
    }

    @Override
    protected boolean internalInitialize(@NonNull Object o) {

        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "MPU9050";
    }
}
