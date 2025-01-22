package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.GenericController;

public class IntakeLogic extends GenericController {
    public enum States {
        RETRACT,
        IDLE
    }
    public static States state = States.IDLE;
    public static ElapsedTime time;
    public static boolean wasDriverActivated = false;
    private static boolean reset = false;
    public static void update(){
        if(Controls.RetractExtendo) {
            state = States.RETRACT;
            Controls.RetractExtendo = false;
        }
        switch (state){
            case IDLE:
                // :)
                break;
            case RETRACT:
                DropDown.setDown(0);
                ActiveIntake.Reverse(0.6);
                Extendo.motor.setPower(-1);
                if((Extendo.motor.getCurrent(CurrentUnit.AMPS) > 5 && Extendo.motor.getVelocity() < 2) || reset){
                    Extendo.motor.setPower(0);
                    reset = true;
                    if(time.seconds() > 0.2){
                        Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        state = States.IDLE;
                        Controls.Transfer = true;
                    }
                } else time.reset();
                break;
        }
        if(wasDriverActivated && Math.abs(gamepad2.right_trigger) <= 0.02 && Math.abs(gamepad2.left_trigger) <= 0.02){
            ActiveIntake.powerOn();
            wasDriverActivated = false;
        }
        if(Math.abs(gamepad2.right_trigger) > 0.02 && ActiveIntake.isOff()){
            ActiveIntake.powerOn(gamepad2.right_trigger);
            wasDriverActivated = true;
        }
        if(Math.abs(gamepad2.left_trigger) >= 0.02 && ActiveIntake.isOff()){
            ActiveIntake.Reverse(gamepad2.left_trigger);
            wasDriverActivated = true;
        }

    }
}
