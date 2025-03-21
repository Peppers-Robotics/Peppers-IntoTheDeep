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
        return calculatePower(currentPosition, null);
    }
    public double calculatePower(double currentPosition, Double d){
        if(time.seconds() < 1.0/freq) return lastReturn;
        time.reset();
        error = targetPosition - currentPosition;
        double dtime = et.seconds();

        double P = error;
        double D = (error - lastError) / et.seconds();
        if(d != null){
            D = d;
        }
        Isum += error * dtime;
        double r = pidCoefficients.p * P + pidCoefficients.d * D;
//        double r = pidCoefficients.p * P + pidCoefficients.d * D;

        if(Math.abs(r) >= maxActuatorOutput && error * r > 0){
            clamp = 0;
            Isum = 0;
        } else clamp = 1;

        r += pidCoefficients.i * Isum;

        et.reset();

        lastError = error;
        lastReturn = r;
        return r;
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
    public PIDCoefficients getCoeff(){
        return pidCoefficients;
    }
}
