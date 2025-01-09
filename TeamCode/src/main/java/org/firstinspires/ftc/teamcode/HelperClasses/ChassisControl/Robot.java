package org.firstinspires.ftc.teamcode.HelperClasses.ChassisControl;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;

@Config
public class Robot {
    public static MotorFeedForward leftFront = new MotorFeedForward(2),
                                   leftBack = new MotorFeedForward(2.7),
                                   rightBack = new MotorFeedForward(1.9),
                                   rightFront = new MotorFeedForward(3.6);
    public static AsymmetricMotionProfile driveProfile = new AsymmetricMotionProfile(65, 55, 55),
            strafeProfile = new AsymmetricMotionProfile(65, 55, 55),
            rotationProfile = new AsymmetricMotionProfile(2* 3.14, 2*3.14, 2 * 3.14);
    public static PredictiveModel drive = new PredictiveModel(0.02, 2.55), strafe = new PredictiveModel(0.02, 2.3), rotate = new PredictiveModel(0.02, 3.14);
    private static double lastTime = 0;
    public static Localizer localizer;
    private static double getLoopTimeInSeconds(){
        double ct = System.currentTimeMillis() - lastTime;
        lastTime = System.currentTimeMillis();
        return ct * 1 / 100.f;
    }
    public Robot(){
    }

    public static void update(){

        // calculate force with the formula wheel.getForceFromAcceleration(acceleration) * 100;
//        double leftFrontForce = 10, rightFrontForce = 0, leftBackForce = 2, rightBackForce = 3;

//        double forwardForce = vectorAddition(leftFrontForce, rightFrontForce);
//        double backwardForce = vectorAddition(leftBackForce, rightBackForce);
//        double leftwardForce = vectorAddition(leftBackForce, leftFrontForce);
//        double rightwardForce = vectorAddition(rightBackForce, rightFrontForce);

//        double xForce = forwardForce + backwardForce, yForce = leftwardForce - rightwardForce, headingForce = leftFrontForce + leftBackForce - rightwardForce - rightBackForce;
        double BLp, BRp, FLp, FRp;
        // x FF
        PredictiveModel.Parameters currentState = new PredictiveModel.Parameters(localizer.getPoseEstimate().getX(), localizer.getPoseVelocity().getX());
        double targetVelocity = driveProfile.getVelocity();
        double robotXAcceleration = drive.getOptimalForceForTargetVelocity(currentState.velocity, driveProfile.getVelocity()) / PredictiveModel.mass;
    }

    private static double vectorAddition(double v1, double v2){
        return v1 * v2 / Math.sqrt(v1 * v1 + v2 * v2);
    }

}
