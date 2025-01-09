package org.firstinspires.ftc.teamcode.HelperClasses.ChassisControl;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PredictiveModel {
    public static class Parameters{
        public double velocity, position;
        public Parameters(double pos, double velo){
            velocity = velo;
            position = pos;
        }
    }
    public static double mass = 12; // in kg
    public double frictionConstant = 0.001;
    public double acceleration = 2.55;
    public PredictiveModel(double kf, double acc){
        frictionConstant = kf;
        acceleration = acc;
    }

    public Parameters getPrediction(Parameters initialCondition, double force, double timeframe){
        Parameters derivative = new Parameters(initialCondition.velocity, (force - mass * 9.8 * frictionConstant * initialCondition.velocity) / mass);
        return new Parameters(derivative.position * timeframe + initialCondition.position, initialCondition.velocity + derivative.velocity * timeframe);
    }
    public double getOptimalForceForTargetVelocity(double currentVelo, double targetVelo){
        double deltaV = targetVelo - currentVelo;
        return mass * deltaV + mass * frictionConstant * 9.8 * currentVelo;
    }
}
