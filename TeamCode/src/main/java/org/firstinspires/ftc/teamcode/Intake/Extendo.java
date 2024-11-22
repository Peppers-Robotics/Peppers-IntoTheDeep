package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.CutOffResolution;
import org.firstinspires.ftc.teamcode.HelperClasses.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogicStateMachine;

@SuppressWarnings("unused")
@Config
public class Extendo {

    public enum States {
        RETRACT,
        IDLE,
        HOLD0
    }
    public static States CurrentState;

    public static CachedMotor motor;
    public static ServoPlus dropDownIntakeRight;
    public static ServoPlus dropDownIntakeLeft;
    public static PIDController pidController = new PIDController(0.02, 0, 0.0007);
    public static double MaxExtension = 120;
    private static volatile double targetPosition = 0;
//    public static final double tensionCoeff = 10;
    public static double off1 = 10, off2 = 10, koeff = 0.11;
    public static final double A = 60, B = 90;
    public static AsymmetricMotionProfile DropDownProfile;

    static {
        CurrentState = States.RETRACT;
        DropDownProfile = new AsymmetricMotionProfile(1000, 8000, 8000);
    }

    private static double inverseKinematicsForDropdownLinkage(double lenght){

        if(lenght > 2 * A) lenght = 2 * A;
        if(lenght < 0) lenght = 0;

        double theta = Math.acos(- (B*B - A*A - (B - A + lenght)*(B - A + lenght)) / (2*A * (B - A + lenght)));

        return Math.toDegrees(Math.PI - theta);
    }

    public synchronized static void DropDown(double distanceInMm){
        if(DropDownProfile.motionEnded()) DropDownProfile.startMotion(DropDownProfile.getPosition(), inverseKinematicsForDropdownLinkage(distanceInMm));

        dropDownIntakeRight.setAngle(DropDownProfile.getPosition() + off1 + DropDownProfile.getPosition() * koeff);
        dropDownIntakeLeft.setAngle(DropDownProfile.getPosition() + off2 + DropDownProfile.getPosition() * koeff);

        Initialization.telemetry.addData("dropdown profile", DropDownProfile.getPosition());
    }

    public synchronized static void Extend(int position){
        pidEnable = true;
        position *= -1;
        if(position == targetPosition) return;

        targetPosition = position;
        pidController.setTargetPosition(position);
    }
    public static boolean pidEnable = false;
    public static ElapsedTime retractTime = new ElapsedTime(), waitToCloseClaw = new ElapsedTime();
    public static void update(){

        switch (CurrentState){
            case RETRACT:
                DropDown(0);
                DropDownProfile.update();
                if(retractTime.seconds() >= 0.5 && motor.getVelocity() <= 5){
                    motor.setPower(0);
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    CurrentState = States.HOLD0;
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    waitToCloseClaw.reset();
                } else {
                    if(DropDownProfile.motionEnded())
                        motor.setPower(1);
                }
                break;
            case HOLD0:
                if(!DropDownProfile.motionEnded() || DropDownProfile.getPosition() > 1) waitToCloseClaw.reset();
                if(waitToCloseClaw.seconds() >= 0.3 && waitToCloseClaw.seconds() <= 2){
                    if(Storage.hasAlliancePice()) {
                        Claw.close();
                        OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.IDLE_SCORING);
                    }
                }
                if(-motor.getCurrentPosition() <= 50)
                    motor.setMotorDisable();
                else motor.setMotorEnable();
                motor.setPower(12 / Initialization.Voltage * 0.12);
                retractTime.reset();
                break;
            case IDLE:
                motor.setMotorEnable();
                retractTime.reset();
                waitToCloseClaw.reset();
                break;
        }

       DropDownProfile.update();
       DropDown(DropDownProfile.getTargetPosition());
       Initialization.telemetry.addData("currentPosition", motor.getCurrentPosition());
       Initialization.telemetry.addData("targetPos", targetPosition);
    }


}
