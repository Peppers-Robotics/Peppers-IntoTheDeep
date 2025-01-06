package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.DifferentialHelper;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;

@Config
public class Arm {
    public static double s1Offset = 230, s2Offset = 140;
    private static double armPrevPos = 0, pivotPrevPos = 0;
    public static ServoPlus servo1, servo2;
    public static AsymmetricMotionProfile armProfile, pivotProfile;
    private static final DifferentialHelper diffy;

    public static PIDController s1Controller = new PIDController(0.01, 0, 0), s2Controller = new PIDController(0.01, 0, 0);

    static {
        armProfile = new AsymmetricMotionProfile(4e3, 7e3, 7e4);
        pivotProfile = new AsymmetricMotionProfile(6e3, 6e3, 5e4);
        diffy = new DifferentialHelper(1/2.f);
    }


    public static void setArmAngle(double angle){
        if(armProfile.getPosition() == angle) return;
//        diffy.setAngleToSecondJoint(angle);
        armProfile.startMotion(armPrevPos, angle);
//        armProfile.startMotion(getCurrentArmAngle(true), angle);
        armPrevPos = angle;
    }

    public static void update(){
        if(Extendo.getCurrentPosition() > 600){
            if(Arm.getCurrentArmAngle() > 210) Arm.setArmAngle(200);
        }
        armProfile.update();
        pivotProfile.update();
        diffy.setAngleToFirstJoint(armProfile.getPosition());
        diffy.setAngleToSecondJoint(pivotProfile.getPosition());

        servo1.setAngle(diffy.getRawAngles()[1] + s1Offset);
        servo2.setAngle(diffy.getRawAngles()[0] + s2Offset);

//        s1Controller.setTargetPosition(diffy.getRawAngles()[0] +//        s1Offset);
//        s2Controller.setTargetPosition(diffy.getRawAngles()[1] + s1Offset);

//        servo1.setPower(s1Controller.calculatePower(servo1.getCurrentCorrectedAngle()));
//        servo2.setPower(s1Controller.calculatePower(servo2.getCurrentCorrectedAngle()));

        Initialization.telemetry.addData("Arm angle", armProfile.getPosition());
        Initialization.telemetry.addData("s1_pos", servo1.getPosition());
        Initialization.telemetry.addData("s2_pos", servo2.getPosition());
        Initialization.telemetry.addData("Pivot angle", pivotProfile.getPosition());
        Initialization.telemetry.addData("deg1", diffy.getRawAngles()[1]);
        Initialization.telemetry.addData("deg0", diffy.getRawAngles()[0]);
    }

    public static void setPivotAngle(double angle){
        if(pivotProfile.getPosition() == angle) return;
//        diffy.setAngleToFirstJoint(angle);
        pivotProfile.startMotion(pivotPrevPos, angle);
        pivotPrevPos = angle;
    }

    public static boolean motionCompleted(){
        return pivotProfile.motionEnded() && armProfile.motionEnded();
    }
    public static double getCurrentPivotAngle(){
        return pivotProfile.getPosition();
    }
    public static double getPrecentOfArmMotionCompleted(){
        return armProfile.getPrecentOfMotion();
    }
    public static double getCurrentArmAngle(){
        return armProfile.getPosition();
    }
    public static double getCurrentArmAngle(boolean fromEncoder){
        return (servo1.getAngle() + servo2.getAngle()) / 2;
    }

}
