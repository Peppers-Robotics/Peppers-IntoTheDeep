package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
public class Elevator {
    public static boolean Disable = true;
    public static CachedMotor motor;
    public static PIDController controller = new PIDController(0.013, 0, -0.0003);
    public static PIDCoefficients climb = new PIDCoefficients(0.04, 0, 0);
    public static PIDCoefficients normal = new PIDCoefficients(0.008, 0, -0.00015);
    public static double kfUp = 0.22, kfDown = 0.04, elevatorMin = 400, elevatorMax = 1080;
    public static AsymmetricMotionProfile motionProfile = new AsymmetricMotionProfile(6000, 7000, 7000);

    static {
        PIDControllerInWork = true;
        controller.setFreq(30);
    }

    private static double targetPos = 0;
    public static boolean PowerOnDownToTakeSample = false;
    public static double power = 1;

    synchronized public static void setTargetPosition(double pos){
        pos *= 1;
        if(pos == targetPos) return;
        motionProfile.startMotion(targetPos, pos);
        targetPos = pos;
        motionProfile.update();
        controller.setTargetPosition(pos, false);
        PowerOnDownToTakeSample = false;
        motor.setMotorEnable();
//        controller.setTargetPosition(motionProfile.getPosition());
    }

    public static double getTargetPosition(){ return targetPos; }
    public static double getCurrentPosition(){ return motor.getCurrentPosition(); }
    public static boolean PIDControllerInWork;

//    public static boolean ReachedTargetPosition(){ return Math.abs(getCurrentPosition() - 2) <= getTargetPosition(); }
    public static boolean ReachedTargetPosition(){ return motionProfile.motionEnded(); }
    public static final double ratio = 1070/435.f;
    public static boolean RESET = true;
    public static ElapsedTime time = new ElapsedTime();
    private static boolean was = false;

    public static void update() {
        if (Disable) {
            motor.setPower(0);
            if(Climb.PTOActivated()) {
                Chassis.FL.setPower(0);
                Chassis.FR.setPower(0);
            }
            return;
        }
        Robot.telemetry.addData("Elevator Current Position", motor.getCurrentPosition());
//        Robot.telemetry.addData("Elevator enabled", motor.isMotorEnabled());
//        Robot.telemetry.addData("TargetPosition", targetPos);

        if (RESET) {
            if (motor.getCurrent(CurrentUnit.AMPS) > 6.5 || was) {
                was = true;
                motor.setPower(0);
                if (time.seconds() > 0.2) {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RESET = false;
                }

            } else {
                motor.setPower(-1);
                time.reset();
            }
            return;
        }

        motionProfile.update();
//        controller.setTargetPosition(motionProfile.getPosition());
        if(Climb.PTOActivated()){
            controller.setPidCoefficients(climb);
            motor.setMotorDisable();
            double pp = -controller.calculatePower(motor.getCurrentPosition());
            Chassis.FL.setPower(pp);
            Chassis.FR.setPower(pp);
            return;
        }
        if(getTargetPosition() <= 0 && getCurrentPosition() < 30 && !PowerOnDownToTakeSample){
            motor.setMotorDisable();
        } else if(getTargetPosition() > 0 || PowerOnDownToTakeSample){
            motor.setMotorEnable();
        }


        if (PowerOnDownToTakeSample) {
            motor.setPower(-power);
            motor.setMotorEnable();
        } else {
            controller.setPidCoefficients(normal);
            if (motor.isMotorEnabled()) {
                double p = controller.calculatePower(motor.getCurrentPosition(), motor.getVelocity());
                if(Elevator.getCurrentPosition() > 400){
                    double kf = (kfUp - kfDown) / (elevatorMax - elevatorMin) * Elevator.getCurrentPosition() + 0;
                    p += kf;
                }
                motor.setPower(p);
            }
        }

    }

}
