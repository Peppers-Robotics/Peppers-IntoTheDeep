package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.LimitSwitch;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@SuppressWarnings("unused")
@Config
public class Extendo {
    public static CachedMotor motor, encoder;
    public static PIDController pidController = new PIDController(0.01, 0, -0.0004);
    public static int MaxExtendoExtension = 840;
    private static double targetPosition = 0;
    public static int offset = 0;
    public static LimitSwitch lm;

    static {
        pidController.setFreq(40);
    }
    public static boolean ReachedTargetPosition(){
        return Math.abs(targetPosition - getCurrentPosition()) <= 5;
    }
    public static boolean ReachedTargetPosition(double t){
        return Math.abs(targetPosition - getCurrentPosition()) <= t;
    }

    public static int getCurrentPosition(){
        //offset = Extendo.getCurrentPosition() + offset;
        return encoder.getCurrentPosition();// - offset;
    }
    public static double getCurrentVelocity(){ return encoder.getVelocity(); }

    public synchronized static void Extend(int position, double afterMs){
        new Thread(() -> {
            double time = System.currentTimeMillis();
            while (System.currentTimeMillis() - time < afterMs);
            Extend(position);
        }).start();
    }
    public static void Extend(int position){
        if(position < 0) position = 0;
        if(position > MaxExtendoExtension) position = MaxExtendoExtension;
        if(position == targetPosition) return;

        targetPosition = position;
        pidController.setTargetPosition(position);
    }
    public static boolean DISABLE = false, was = false;
    private static ElapsedTime time = new ElapsedTime();
    public static double power = 1;
    public static boolean PowerOnToTransfer = false;
    public static int getTargetPosition(){
        return (int) targetPosition;
    }
    public static void update(){
        if(DISABLE){
            return;
        }
//        if (RESET) {
//            if (motor.getCurrent(CurrentUnit.AMPS) > 9 || was) {
//                was = true;
//                motor.setPower(0);
//                if (time.seconds() > 0.2) {
//                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    RESET = false;
//                }
//
//            } else {
//                motor.setPower(-1);
//                time.reset();
//            }
//            return;
//        }

        if(PowerOnToTransfer) {
//            motor.setPower(pidController.calculatePower(getCurrentPosition(), getCurrentVelocity()));
            motor.setPower(-power);
//            motor.setMotorDisable();
        } else {
            motor.setPower(pidController.calculatePower(getCurrentPosition(), getCurrentVelocity()));
            motor.setMotorEnable();
        }
//        Robot.telemetry.addData("Extendo pos", motor.getCurrentPosition());
    }
    public static double extendoMass = 2, extendoF = 0.05, magicNumber = 23.83;
    public static void holdStill(double pitch){
        double p = magicNumber * Math.cos(pitch) + extendoF;
        motor.setPower(p);
    }


    public static boolean isMaxExtended() {
        return getCurrentPosition() >= MaxExtendoExtension;
    }

    public static int getMaxPosition() {
        return MaxExtendoExtension;
    }
}
