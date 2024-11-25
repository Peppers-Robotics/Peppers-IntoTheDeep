package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.CutOffResolution;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;
import org.firstinspires.ftc.teamcode.Initialization;

@Config
public class Elevator {
    public static CachedMotor motor;
    public static PIDController controller = new PIDController(0.013, 0, 0.0005);
    public static PIDCoefficients climb = new PIDCoefficients(0.01, 0, 0.0003);
    public static PIDCoefficients normal = new PIDCoefficients(0.013, 0, 0.0005);
    public static AsymmetricMotionProfile motionProfile = new AsymmetricMotionProfile(3000, 5000, 7000);

    static {
        PIDControllerInWork = true;
    }

    private static double targetPos = 0;

    synchronized public static void setTargetPosition(double pos){
        pos *= -1;
        if(pos == targetPos) return;
        motionProfile.startMotion(targetPos, pos);
        targetPos = pos;
        motionProfile.update();
        controller.setTargetPosition(pos);
//        controller.setTargetPosition(motionProfile.getPosition());
    }

    public static double getTargetPosition(){ return targetPos; }
    public static double getCurrentPosition(){ return -motor.getCurrentPosition(); }
    public static boolean PIDControllerInWork;

    public static boolean ReachedTargetPosition(){ return Math.abs(getCurrentPosition() - 2) <= getTargetPosition(); }
//    public static boolean ReachedTargetPosition(){ return motionProfile.motionEnded(); }
    public static final double ratio = 1070/435.f;

    public static void update(){

       if(targetPos <= 0 && getCurrentPosition() <= 10 && !Climb.isPTOEngaged()){
           motor.setMotorDisable();
       } else {
           motor.setMotorEnable();
       }

        motionProfile.update();
        controller.setTargetPosition(targetPos);
        if(Climb.isPTOEngaged()){
            controller.setPidCoefficients(climb);
            controller.setTargetPosition(targetPos, false);
            double pidp = -controller.calculatePower(motor.getCurrentPosition());
            Initialization.telemetry.addData("power", pidp);
            motor.setPower(-pidp * ratio);
            Chassis.BL.setPower(pidp);
            Chassis.BR.setPower(pidp);

        } else {
            controller.setPidCoefficients(normal);
            motor.setPower(
                    controller.calculatePower(motor.getCurrentPosition())
            );
        }
        Initialization.telemetry.addData("Elevator pose from profile", motionProfile.getPosition());
        Initialization.telemetry.addData("Elevator current pose", -motor.getCurrentPosition());
        Initialization.telemetry.addData("Elevator targetPos", targetPos);
        Initialization.telemetry.addData("Is elevator enabled", motor.isMotorEnabled());
    }

}
