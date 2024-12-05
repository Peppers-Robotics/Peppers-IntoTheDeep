package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Initialization;

@SuppressWarnings("unused")
@Config
public class Extendo {
    public static CachedMotor motor;
    public static PIDController pidController = new PIDController(0.01, 0, 0.00045);
    public static int MaxExtendoExtension = 1300;
    private static volatile double targetPosition = 0;
    public static boolean ReachedTargetPosition(){
        return Math.abs(targetPosition - getCurrentPosition()) <= 5;
    }
    public static boolean ReachedTargetPosition(double t){
        return Math.abs(targetPosition - getCurrentPosition()) <= t;
    }

    public static double getCurrentPosition(){
        return -motor.getCurrentPosition();
    }

    public synchronized static void Extend(int position){
        pidEnable = true;
        position *= -1;
        if(position == targetPosition) return;

        targetPosition = position;
        pidController.setTargetPosition(position);
    }
    public static boolean pidEnable = false;
    public static ElapsedTime retractTime = new ElapsedTime(), waitToCloseClaw = new ElapsedTime();
    public static void update(){

       if(pidEnable){
           motor.setPower(pidController.calculatePower(motor.getCurrentPosition()));
       }
       Initialization.telemetry.addData("currentPosition", motor.getCurrentPosition());
       Initialization.telemetry.addData("targetPos", targetPosition);
    }


}
