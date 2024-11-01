package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.CutOffResolution;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;

@SuppressWarnings("unused")
public class Extendo {
    public static CachedMotor motor;
    public static ServoPlus dropDownIntakeRight;
    public static ServoPlus dropDownIntakeLeft;
    public static PIDController pidController = new PIDController(0, 0, 0);
    public static double MaxExtension;
    private static volatile double targetPosition = 0;
    public static final double tensionCoeff = 10;
    public static final double A = 60, B = 90;
    public static boolean HOME_RESET_ENCODERS = true;

    private static double inverseKinematicsForDropdownLinkage(double lenght){

        if(lenght > 2 * A) lenght = 2 * A;
        if(lenght < 0) lenght = 0;

        double theta = Math.acos(- (B*B - A*A - (B - A + lenght)*(B - A + lenght)) / (2*A * (B - A + lenght)));

        return Math.toDegrees(Math.PI - theta);
    }

    public synchronized static void DropDown(double distanceInMm){
        dropDownIntakeRight.setAngle(inverseKinematicsForDropdownLinkage(distanceInMm) - tensionCoeff);
        dropDownIntakeLeft.setAngle(inverseKinematicsForDropdownLinkage(distanceInMm) - tensionCoeff);
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
        if(HOME_RESET_ENCODERS){

            if(motor.getCurrentPosition() < 30) motor.setPower(-0.3);
            else motor.setPower(-1);

            if(motor.getVelocity() < 5){
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                HOME_RESET_ENCODERS = false;
            }

            return;
        }
        double PIDPower = pidController.calculatePower(motor.getCurrentPosition());

        motor.setPower(CutOffResolution.GetResolution(PIDPower, 2));
    }


}
