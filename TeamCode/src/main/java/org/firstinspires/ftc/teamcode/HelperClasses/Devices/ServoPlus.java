package org.firstinspires.ftc.teamcode.HelperClasses.Devices;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.CutOffResolution;
import org.firstinspires.ftc.teamcode.Initialization;


@ServoType(flavor = ServoFlavor.CUSTOM)
@DeviceProperties(name = "ServoPlus", xmlTag = "servoPlus")
public class ServoPlus extends ServoImpl implements Servo, HardwareDevice {
    private boolean isCR = false;
    public ServoPlus(ServoController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
        isCR = false;
    }
    public ServoPlus(ServoController controller, int portNumber) {
        super(controller, portNumber);
        isCR = false;
    }
    public ServoPlus(Servo s){
        super(s.getController(), s.getPortNumber(), s.getDirection());
        isCR = false;
    }
    private volatile double MaxAngle = 355;

    synchronized public void setMaxAngle(double angle){
        MaxAngle = angle;
    }
    private double thisAngle = 0;
    synchronized public void setAngle(double angle){
//        if(!Initialization.hubs.get(0).isEngaged()) return;
        thisAngle = angle;
        double toSetPos = CutOffResolution.GetResolution(angle / MaxAngle, 2);
        setPosition(toSetPos);
    }
    public double getAngle(){
        if(encoder == null)
            return thisAngle;
        else return encoder.getVoltage() / encoder.getMaxVoltage() * 360.f;
    }
    public boolean isEqualToAngle(double angle){
        return Math.abs(angle - getAngle()) < 0.1;
    }
    // -------------------- CR Implementation --------------------

    public void setEncoder(AnalogInput ai){
        encoder = ai;
    }

    private AnalogInput encoder = null;
    private double lastAngle = 0;
    private int revolutions = 0;

    public void setToCRControlled(AnalogInput ai){
        isCR = true;
        encoder = ai;
        revolutions = 0;
        lastAngle = getCurrentRawAngle();
    }
    public void setToServoControlled(){
        isCR = false;
    }
    public double getCurrentRawAngle(){
        if(!isCR) return getAngle();
        return 360.f + encoder.getVoltage() / encoder.getMaxVoltage() * 360.f;
    }

    public void update(){
        double currentAngle = getCurrentRawAngle();
        if(currentAngle <= 90 && lastAngle >= 270){
            revolutions++;
        }
        if(currentAngle >= 270 && lastAngle <= 90){
            revolutions --;
        }
        lastAngle = currentAngle;
    }
    public double getCurrentCorrectedAngle(){
        return getCurrentRawAngle() + 360 * revolutions;
    }
    public void setPower(double p){
        if(p < 0) setPosition(0.5 * p);
        else setPosition(0.5 * p + 0.5);
    }
}
