package org.firstinspires.ftc.teamcode.HelperClasses.ChassisControl;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotorFeedForward {
    public double Kt = 0.016, Ke = 0.002, R = 2.75, wheelRadius = 12, maxRPM = 435;
    public double Mct = R / (Kt * 1 + maxRPM * Ke);

    public double mass = 2.3;
    public MotorFeedForward(double mass){
        this.mass = mass;
    }
    public double getForceFromAcceleration(double acc){
        return Mct * wheelRadius * acc * mass * 1 / 100.f;
    }
}
