package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.CutOffResolution;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;
import org.firstinspires.ftc.teamcode.Initialization;

@Config
public class Elevator {
    public static CachedMotor motor;
    private static PIDController controller;
    public static PIDCoefficients pidCoefficients;
    private static AsymmetricMotionProfile motionProfile;

    static {
        pidCoefficients = new PIDCoefficients(0, 0, 0);
        controller = new PIDController(pidCoefficients);
        motionProfile = new AsymmetricMotionProfile(100, 40, 30);
        PIDControllerInWork = true;
    }

    private static double targetPos = 0;

    synchronized public static void setTargetPosition(double pos){
        if(pos == targetPos) return;
        motionProfile.startMotion(targetPos, pos);
        targetPos = pos;
        motionProfile.update();
        controller.setTargetPosition(motionProfile.getPosition());
    }

    public static double getTargetPosition(){ return targetPos; }
    public static double getCurrentPosition(){ return motor.getCurrentPosition(); }
    public static boolean PIDControllerInWork;

//    public static boolean ReachedTargetPosition(){ return Math.abs(getCurrentPosition() - 2) <= getTargetPosition(); }
    public static boolean ReachedTargetPosition(){ return motionProfile.motionEnded(); }
    private static boolean elevatorReachedStopMotion, NEED_TO_RESET = false;
    private static ElapsedTime time = new ElapsedTime();

    public static void update(){
       /* if(targetPos <= 0 && NEED_TO_RESET) {
            if(elevatorReachedStopMotion){
                if(time.seconds() > 0.1){
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    NEED_TO_RESET = false;
                }
                motor.setPower(0);
            }
            else {
                elevatorReachedStopMotion = motor.getVelocity() <= 5;
                motor.setPower(-1);
                if(elevatorReachedStopMotion) {
                    time.reset();
                    motor.setPower(0);
                }
            }
            return;
        }
*/
        controller.setPidCoefficients(pidCoefficients);
        motionProfile.update();
        controller.setTargetPosition(motionProfile.getPosition(), false);
        motor.setPower(CutOffResolution.GetResolution(controller.calculatePower(motor.getCurrentPosition()), 2));
        Initialization.telemetry.addData("Elevator pose from profile", motionProfile.getPosition());
        Initialization.telemetry.addData("Elevator current pose", motor.getCurrentPosition());
        Initialization.telemetry.addData("Elevator targetPos", motor.getTargetPosition());
    }

}
