package org.firstinspires.ftc.teamcode.HelperClasses.Devices;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class LimitSwitch extends DigitalChannelImpl implements DigitalChannel, HardwareDevice {
    public LimitSwitch(DigitalChannelController controller, int portNumber){
        super(controller,portNumber);
    }
}
