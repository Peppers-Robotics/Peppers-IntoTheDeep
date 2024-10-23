package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;

@Config
public class Elevator {
    public static CachedMotor motor;
    private static PIDController controller;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);

    private static double targetPos = 0;

    synchronized public static void setTargetPosition(double pos){
        if(pos == targetPos) return;
        controller.setTargetPosition(pos);
    }

    synchronized public static void update(){
        motor.setPower(controller.calculatePower(motor.getCurrentPosition()));
    }

}
