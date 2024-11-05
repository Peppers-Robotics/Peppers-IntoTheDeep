package tests.HardwareEmulators;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class ServoE implements Servo {
    private double pos = 0;
    private Direction dir;
    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {
        dir = direction;
    }

    @Override
    public Direction getDirection() {
        return dir;
    }

    @Override
    public void setPosition(double position) {
        pos = position;
    }

    @Override
    public double getPosition() {
        return pos;
    }

    @Override
    public void scaleRange(double min, double max) {

    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.HiTechnic;
    }

    @Override
    public String getDeviceName() {
        return "Axon";
    }

    @Override
    public String getConnectionInfo() {
        return "Emulator";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
