package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Initialization;

@Config
public class Elevator {
    public static boolean Disable = false;
    public static CachedMotor motor;
    public static PIDController controller = new PIDController(0.013, 0, -0.0003);
    public static PIDCoefficients climb = new PIDCoefficients(0.02, 0, 0);
    public static PIDCoefficients normal = new PIDCoefficients(0.013, 0.00005, -0.0003);
    public static double kf = 0.05, kff = 1;
    public static double aggresiveP = 0.01, aggressiveI = 0.0002, aggressiveD = 0.0003;
    public static AsymmetricMotionProfile motionProfile = new AsymmetricMotionProfile(6000, 7000, 7000);

    static {
        PIDControllerInWork = true;
        controller.setFreq(20);
    }

    private static double targetPos = 0;
    public static boolean PowerOnDownToTakeSample = false;

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

    public static void update(){
        if(Disable){
            if(Climb.isPTOEngaged()) {
                Chassis.BL.setPower(0);
                Chassis.BR.setPower(0);
            }
            motor.setPower(0);
            return;
        }

        if(RESET){
            if(motor.getCurrent(CurrentUnit.AMPS) > 5.5 || was){
                was = true;
                motor.setPower(0);
                if(time.seconds() > 0.2){
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

        if(controller.getTargetPosition() <= 0 && getCurrentPosition() <= 25 && !PowerOnDownToTakeSample || Climb.isPTOEngaged()){
            motor.setMotorDisable();
        } else {
            motor.setMotorEnable();
        }

        motionProfile.update();
        controller.setTargetPosition(targetPos);
        if(Climb.isPTOEngaged()){
            motor.setMotorDisable();
            controller.setPidCoefficients(climb);
            controller.setTargetPosition(targetPos, false);
            double pidp = -controller.calculatePower(motor.getCurrentPosition());
            Initialization.telemetry.addData("power", pidp);
//            motor.setPower(-pidp * ratio);
            Chassis.BL.setPower(pidp);
            Chassis.BR.setPower(pidp);

        } else {
            if(PowerOnDownToTakeSample){
                motor.setPower(-1);
            } else {
                if(controller.getTargetPosition() != OutTakeStateMachine.ElevatorTakeSpecimen) {
                    controller.setPidCoefficients(normal);
                } else {
                    controller.setPidCoefficients(new PIDCoefficients(aggresiveP, aggressiveI, aggressiveD));
                }
                if (motor.isMotorEnabled()) {
                    motor.setPower(
                            controller.calculatePower(motor.getCurrentPosition(), motor.getVelocity())
                            + kf
                    );
					motor.setPower(motor.getPower() * 12 / Initialization.Voltage);
                } else motor.setPower(0);
            }
        }
        Initialization.telemetry.addData("Elevator pose from profile", motionProfile.getPosition());
        Initialization.telemetry.addData("Elevator power", motor.getPower());
        Initialization.telemetry.addData("Elevator current pose", getCurrentPosition());
        Initialization.telemetry.addData("Elevator targetPos", targetPos);
        Initialization.telemetry.addData("Is elevator enabled", motor.isMotorEnabled());
        Initialization.telemetry.addData("Elevator velocity", Elevator.motor.getVelocity());
    }

}
