package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HelperClasses.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.CutOffResolution;
import org.firstinspires.ftc.teamcode.HelperClasses.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.OutTake.Claw;

@SuppressWarnings("unused")
@Config
public class Extendo {
    public static CachedMotor motor;
    public static ServoPlus dropDownIntakeRight;
    public static ServoPlus dropDownIntakeLeft;
    public static PIDController pidController = new PIDController(0, 0, 0);
    public static double MaxExtension;
    private static volatile double targetPosition = 0;
//    public static final double tensionCoeff = 10;
    public static double off1 = 31, off2 = 30;
    public static final double A = 60, B = 90;
    public static boolean HOME_RESET_ENCODERS = true;
    public static AsymmetricMotionProfile DropDownProfile;

    static {
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
        dropDownIntakeRight.setAngle(DropDownProfile.getPosition() + off1);
        dropDownIntakeLeft.setAngle(DropDownProfile.getPosition() + off2);
        Initialization.telemetry.addData("dropdown profile", DropDownProfile.getPosition());
    }

    public synchronized static void Extend(int position){
        if(position == targetPosition) return;
        if(position == 0){
            HOME_RESET_ENCODERS = true;
        }

        targetPosition = position;
        pidController.setTargetPosition(position);
    }
    public synchronized static void update(){
       /* if(HOME_RESET_ENCODERS){

            if(motor.getCurrentPosition() < 30) motor.setPower(-0.3);
            else motor.setPower(-1);

            if(motor.getVelocity() < 5){
                if(Claw.HasElementInIt()) Claw.close(); // :) <- eu cand am scris asta
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                HOME_RESET_ENCODERS = false;
            }

            return;
        }*/
        double PIDPower = pidController.calculatePower(motor.getCurrentPosition());

        motor.setPower(CutOffResolution.GetResolution(PIDPower, 2));
        DropDownProfile.update();
        Initialization.telemetry.addData("currentPosition", motor.getCurrentPosition());
        Initialization.telemetry.addData("targetPos", targetPosition);
    }


}
