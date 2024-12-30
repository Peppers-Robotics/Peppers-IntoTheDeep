package org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import javax.crypto.Cipher;

public class PPDController {
    public PIDCoefficients coeff;
    private double TargetPositon = 0;
    private double Frequency = 30;
    public PPDController(double p, double d){
        coeff.d = d;
        coeff.p = p;
    }
    public PPDController(PIDCoefficients pidc){
        coeff = pidc;
    }
    public void setTargetPositon(double pos){
        TargetPositon = pos;
    }
    public void setFreqToRun(double freq){
        Frequency = freq;
    }
    private double Isum = 0, lastCalcPower = 0;
    private ElapsedTime time = new ElapsedTime(), freqTime = new ElapsedTime();
    public double update(double ExternalVelocity, double CurrentPositon){
        if(1 / freqTime.seconds() < Frequency) return lastCalcPower;
        freqTime.reset();
        double error = TargetPositon - CurrentPositon;
        Isum += error * time.seconds();
        time.reset();
        lastCalcPower = coeff.p * error - coeff.d * ExternalVelocity + Isum * coeff.i;

        return lastCalcPower;
    }
}
