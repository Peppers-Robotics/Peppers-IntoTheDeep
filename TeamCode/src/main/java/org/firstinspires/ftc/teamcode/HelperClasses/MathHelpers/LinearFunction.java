package org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers;

public class LinearFunction {
    public double start, end;

    public LinearFunction(double start, double end){
        this.start = start;
        this.end = end;
    }

    public double getOutput(double t){
        return (end - start) * t + start;
    }
}
