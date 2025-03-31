package com.ftc.bumblebee.Math;

public class PIDController {
    public double p = 0, i = 0, d = 0;
    private double iSum = 0, lastError = 0;
    public PIDController(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
        setFrequency(freq);
    }
    public void setCoefficients(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }
    public void setFrequency(double f){
        this.freq = f;
        freqTime = System.currentTimeMillis();
    }
    public void setMaxAchivableOutput(double max){
        this.MaxOutput = max;
    }
    private double MaxOutput = 1;
    private long freqTime = 0, dt = -1;
    private double freq = 20;
    private double lastOutput = 0;
    public double output(double e, Double velo){
        if((System.currentTimeMillis() - freqTime) / 1000.f <= 1.0 / freq)
            return lastOutput;
        if(dt == -1) dt = System.currentTimeMillis();
        double P = e;
        double D = (e - lastError) * 1000.f / (System.currentTimeMillis() - dt);
        if(velo != null) D = velo;
        if(!(Math.abs(p * P + d * D) >= MaxOutput && e * (p * P + d * D) > 0)){
            iSum += e * (System.currentTimeMillis() - dt) / 1000.f;
        }
        lastError = e;
        lastOutput = P * p + D * d + iSum * i;
        return lastOutput;
    }
}
