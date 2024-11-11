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
    private double positionSetted = 69;
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

    synchronized public void setAngle(double angle){
        double toSetPos = angle / MaxAngle;
        setPosition(toSetPos);
    }

    @Override
    synchronized public void setPosition(double pos){
        if(pos != positionSetted){
            positionSetted = pos;
            if(pos < 0) pos = 0;
            if(pos > 1) pos = 1;
            this.controller.setServoPosition(portNumber, pos);
        }
    }
    public double getPosition(){
        return positionSetted;
    }
    public double getAngle(){
        return getPosition() / MaxAngle;
    }
}
