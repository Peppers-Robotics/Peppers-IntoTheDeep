package org.firstinspires.ftc.teamcode.HelperClasses;

public class DifferentialHelper {
    private double angleFirstJoint, angleSecondJoint;
    private double gearRatio = 0;
    public DifferentialHelper(double gearRatio){
        angleFirstJoint = 0;
        angleSecondJoint = 0;
        this.gearRatio = gearRatio;
    }

    public void setAngleToFirstJoint(double angle){
        angleFirstJoint = angle;
    }

    public void setAngleToSecondJoint(double angle){
        angleSecondJoint = angle;
    }

    public double[] getRawAngles(){
        return new double[]{(angleSecondJoint + angleFirstJoint) * gearRatio, (angleSecondJoint - angleFirstJoint) * gearRatio};
    }
}
