package org.firstinspires.ftc.teamcode.HelperClasses;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealthImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;


@ServoType(flavor = ServoFlavor.CUSTOM)
@DeviceProperties(name = "ServoPlus", xmlTag = "servoPlus")
public class ServoPlus extends ServoImpl implements Servo, HardwareDevice {
    public ServoPlus(ServoController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }
    public ServoPlus(ServoController controller, int portNumber) {
        super(controller, portNumber);
    }
    public ServoPlus(Servo s){
        super(s.getController(), s.getPortNumber(), s.getDirection());
    }
    private volatile double MaxAngle = 355;

    synchronized public void setMaxAngle(double angle){
        MaxAngle = angle;
    }
    private double thisAngle = 0;
    synchronized public void setAngle(double angle){
        thisAngle = angle;
        double toSetPos = CutOffResolution.GetResolution(angle / MaxAngle, 2);
        setPosition(toSetPos);
    }
    public double getAngle(){
        return thisAngle;
    }
    public boolean isEqualToAngle(double angle){
        return Math.abs(angle - getAngle()) < 0.1;
    }
}
