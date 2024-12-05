package org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public PIDCoefficients pidCoefficients;
    private double targetPosition = 0;
    public double error, lastError, maxActuatorOutput, Isum = 0;
    private final ElapsedTime et = new ElapsedTime();
    public int clamp = 1;
    public double freq = 20;

    public PIDController(PIDCoefficients pidcoef){
        pidCoefficients = pidcoef;
        error = 0;
        lastError = 0;
        maxActuatorOutput = 1; // default for FTC motors
    }
    public PIDController(double p, double i, double d){
        this(new PIDCoefficients(p, i, d));
    }
    public void setPidCoefficients(PIDCoefficients coeff){
        pidCoefficients = coeff;
    }
    public void setFreq(double f){freq = f;}
    private ElapsedTime time = new ElapsedTime();
    private double lastReturn = 0;
    public double calculatePower(double currentPosition){
        if(time.seconds() < 1.0/freq) return lastReturn;
        time.reset();
        error = targetPosition - currentPosition;
        double dtime = et.seconds();

        double P = error;
        double D = (error - lastError) / et.seconds();
        Isum += P * dtime * clamp;
        double r = pidCoefficients.p * P + pidCoefficients.i * Isum + pidCoefficients.d * D;

        double ret = r;

//        if(Math.abs(r) > maxActuatorOutput && error * r > 0){ // Integral Clamping for anti-windup
//            ret = maxActuatorOutput;
//        }
//        if(Math.abs(error) <= 80){
//            ret -= Isum * pidCoefficients.i;
//        }


        et.reset();

        lastError = error;
        lastReturn = ret;
        return ret;
    }
    public void setTargetPosition(double pos, boolean resetIsum){
        targetPosition = pos;
        if(resetIsum) Isum = 0;
    }
    public void setTargetPosition(double pos){
        setTargetPosition(pos, true);
    }
    public double getTargetPosition(){
        return targetPosition;
    }
    public void setMaxActuatorOutput(double mao){
        maxActuatorOutput = mao;
    }
}
