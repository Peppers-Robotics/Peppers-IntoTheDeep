package org.firstinspires.ftc.teamcode.HelperClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AsymetricMotionProfile {
    private double maxVelocity, acceleration;
    private ElapsedTime time = new ElapsedTime();
    private double accelerationTime, deccelarationTime, constantTime,
            currentPosition, initialPosition, targetPosition, mvUsed,
            velocity, sig, deceleration;
    private double t0 = 0, t1 = 0, t2 = 0;
    public AsymetricMotionProfile(double maxVelocity, double acceleration, double deceleration){
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.deceleration = deceleration;
    }
    public void startMotion(double initialPos, double targetPos){
        if(initialPos == targetPos) return;
        double dist = Math.abs(targetPos - initialPos);
        sig = Math.signum(targetPos - initialPos);
        initialPosition = initialPos;
        targetPosition = targetPos;
        currentPosition = initialPosition;

        accelerationTime = maxVelocity / acceleration;
        deccelarationTime = maxVelocity / deceleration;

        if(dist <= (accelerationTime + deccelarationTime) * maxVelocity / 2){
            constantTime = 0;
            mvUsed = Math.sqrt(2*dist / (1.0 / acceleration + 1.0 / deceleration));
            accelerationTime = mvUsed / acceleration;
            deccelarationTime = mvUsed / deceleration;

        } else {
            constantTime = dist / maxVelocity - accelerationTime / 2 - deccelarationTime / 2;
            mvUsed = maxVelocity;
        }

        t0 = accelerationTime;
        t1 = t0 + constantTime;
        t2 = t1 + deccelarationTime;


        velocity = 0;
        time.reset();
    }
    private double a(double t){
        if(t <= t0) return acceleration;
        if(t <= t1) return 0;
        if(t <= t2) return -acceleration;
        return 0;
    }
    private double v(double t){
        if(t <= t0) return t * acceleration;
        if(t <= t1) return maxVelocity;
        if(t <= t2) return maxVelocity - acceleration * (t - t1);
        return 0;
    }
    private double p(double t){
        if(t <= t0) return acceleration / 2 * t * t;
        if(t <= t1) return acceleration / 2 * t0 * t0 + mvUsed * (t - t0);
        if(t <= t2) return acceleration / 2 * t0 * t0 + mvUsed * (t - t0) - deceleration / 2 * (t - t1) * (t - t1);
        return 0;
    }
    public void update(){
        if(time.seconds() < t2)
            currentPosition = initialPosition + sig * p(time.seconds());
        else currentPosition = targetPosition;
    }
    public double getVelocity(){
        return v(time.seconds());
    }
    public double getAcceleration(){
        return a(time.seconds());
    }
    public double getPosition(){
        return currentPosition;
    }
    public void setInstant(double position) { currentPosition = position; }
    public boolean motionEnded(){
        return time.seconds() >= t2;
    }
    public double getTimeUntilMotionEnd(){
        return t2 - time.seconds();
    }
}