package org.firstinspires.ftc.teamcode.HelperClasses;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.Objects;


@DeviceProperties(
        xmlTag = "cachedMotor",
        name = "CachedMotor",
        description = "generic motor",
        builtIn = false
)
@MotorType(
        ticksPerRev = 1550,
        gearing = 55,
        maxRPM = 6000,
        achieveableMaxRPMFraction = 1.0,
        orientation = Rotation.CCW
)

public class CachedMotor extends DcMotorImpl implements DcMotor, HardwareDevice {
    private double lastSetPower = 69;
    private static double MAX_VELOCITY;
    public CachedMotor(DcMotorController controller, int portNumber) {

        super(controller, portNumber);
    }
    public CachedMotor(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction){
        super(controller, portNumber, direction);
    }
    public CachedMotor(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction, @NonNull MotorConfigurationType mct){
        super(controller, portNumber, direction, Objects.requireNonNull(mct));

        mct.setAchieveableMaxRPMFraction(1.0);
        this.controller.setMotorType(portNumber, mct.clone());
    }
    public CachedMotor(DcMotor motor){
        super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
    }
    /**
     * MaxVelocity is measured in [outputDiameter]_(SI) / s, we recommend using m/s
     * <p>
     * @param gearRatio is measured as output / input
     * </p>
     * */
    public void setMaxVelocity(double MaxRPM, double gearRatio, double CPR){
        MAX_VELOCITY = MaxRPM * gearRatio * CPR;
    }
    public double getMaxVelocity(){
        return MAX_VELOCITY;
    }

    @Override
    public synchronized void setPower(double power){
        if(power != lastSetPower){
            lastSetPower = power;
            controller.setMotorPower(this.getPortNumber(), power);
        }
    }

    public void setAproxVelocity(double velocity){
        setPower(velocity / MAX_VELOCITY);
    }

}
