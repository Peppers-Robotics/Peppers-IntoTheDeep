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
    public static boolean Disable = false;
    public static CachedMotor motor, motor2, encoder;
    public static PIDController controller = new PIDController(0.013, 0, -0.0003);
    public static PIDCoefficients climb = new PIDCoefficients(0.04, 0, 0);
    public static PIDCoefficients normal = new PIDCoefficients(0.009, 0.001, -0.00015);
    public static double kfUp = 0.1, kfDown = 0.04, elevatorMin = 400, elevatorMax = 1080;
    public static AsymmetricMotionProfile motionProfile = new AsymmetricMotionProfile(10000, 12000, 10000);

    static {
        PIDControllerInWork = true;
        controller.setFreq(30);
    }

    private static double targetPos = 0;
    public static boolean PowerOnDownToTakeSample = false;
    public static double power = 1;
    private static boolean resetTo0 = false;

    synchronized public static void setTargetPosition(double pos){
        pos *= 1;
        if(pos == targetPos) return;
        if(targetPos > 0) resetTo0 = false;
//        motionProfile.startMotion(targetPos, pos);
        targetPos = pos;
        motionProfile.update();
        controller.setTargetPosition(pos, true);
        PowerOnDownToTakeSample = false;
        motor.setMotorEnable();
//        controller.setTargetPosition(motionProfile.getPosition());
    }

    public static double getTargetPosition(){ return targetPos; }
    public static double getCurrentPosition(){ return -encoder.getCurrentPosition(); }
    public static boolean PIDControllerInWork;

//    public static boolean ReachedTargetPosition(){ return Math.abs(getCurrentPosition() - 2) <= getTargetPosition(); }
    public static boolean ReachedTargetPosition(){ return motionProfile.motionEnded(); }
    public static final double ratio = 1070/435.f;
    public static boolean RESET = true;
    public static ElapsedTime time = new ElapsedTime();
    private static boolean was = false, dis = false;

    public static void update() {
        if (Disable) {
                motor.setPower(0);
                motor2.setPower(0);
                if (Climb.PTOActivated()) {
                    Chassis.FL.setPower(0);
                    Chassis.FR.setPower(0);
                    Chassis.BL.setPower(0);
                    Chassis.BR.setPower(0);
                }
            return;
        } else dis = false;
//        Robot.telemetry.addData("Elevator Current Position", encoder.getCurrentPosition());
//        Robot.telemetry.addData("Elevator power consumption", motor.getCurrent(CurrentUnit.AMPS) + motor2.getCurrent(CurrentUnit.AMPS));
//        Robot.telemetry.addData("Elevator enabled", motor.isMotorEnabled());
//        Robot.telemetry.addData("TargetPosition", targetPos);
        if(getTargetPosition() <= 1 && getCurrentPosition() <= 30 && !resetTo0){
            resetTo0 = true;
            RESET = true;
        }
        if (RESET) {
            if (motor.getCurrent(CurrentUnit.AMPS) + motor2.getCurrent(CurrentUnit.AMPS) >= 7 || was) {
                was = true;
                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                if (time.seconds() >= 0.2) {
                    motor.setPower(0);
                    motor2.setPower(0);
                    RESET = false;
//                }

            } else {
                motor.setPower(-1);
                motor2.setPower(-1);
                time.reset();
            }
            return;
        } else was = false;

        motionProfile.update();
//        controller.setTargetPosition(motionProfile.getPosition());
        if(Climb.PTOActivated()){
            controller.setPidCoefficients(climb);
            motor.setMotorDisable();
            motor2.setMotorDisable();
            double pp = controller.calculatePower(getCurrentPosition());
//            Chassis.FL.setPower(pp);
//            Chassis.FR.setPower(pp);
//            Chassis.BL.setPower(-pp);
//            Chassis.BR.setPower(-pp);
            Chassis.drive(-pp, 0, 0);
            return;
        }
        if(getTargetPosition() <= 0 && getCurrentPosition() < 30 && !PowerOnDownToTakeSample){
            motor.setMotorDisable();
            motor2.setMotorDisable();
        } else if(getTargetPosition() > 0 || PowerOnDownToTakeSample){
            motor.setMotorEnable();
            motor2.setMotorEnable();
        }

        motionProfile.update();
        if (PowerOnDownToTakeSample) {
            motor.setPower(-power);
            motor2.setPower(-power);
            motor.setMotorEnable();
            motor2.setMotorEnable();
        } else {
            controller.setPidCoefficients(normal);
            if (motor.isMotorEnabled()) {
//                controller.setTargetPosition(motionProfile.getPosition(), false);
                double p = controller.calculatePower(getCurrentPosition(), -encoder.getVelocity());
                //c2
                if(Elevator.getCurrentPosition() > 400){
                    double kf = (kfUp - kfDown) / (elevatorMax - elevatorMin) * Elevator.getCurrentPosition() + 0;
                    p += kf;
                }
                motor.setPower(p);
                motor2.setPower(p);
            }
        }

    }

}
