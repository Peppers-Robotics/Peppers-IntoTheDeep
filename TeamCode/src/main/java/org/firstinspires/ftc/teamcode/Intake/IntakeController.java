package org.firstinspires.ftc.teamcode.Intake;

import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.GenericController;

public class IntakeController extends GenericController {
    private static int ExtendoPosition, MaxExtension = 1200;
    public static int Resolution;
    static {
        ExtendoPosition = 0;
        Resolution = 20;
    }
    public static void Update(){
//        ExtendoPosition += (int) (-gamepad1.right_stick_y * Resolution);
        if(Math.abs(gamepad1.right_stick_y) >= 0.05){
            Extendo.CurrentState = Extendo.States.IDLE;
        } else if(Extendo.motor.getCurrentPosition() > -30){
            Extendo.CurrentState = Extendo.States.HOLD0;
        }
        if(Extendo.CurrentState == Extendo.States.IDLE){
            Extendo.motor.setPower(gamepad1.right_stick_y);
        }
        if(gamepad2.right_trigger > 0.1) {
            Extendo.DropDown(Extendo.MaxExtension);
            ActiveIntake.powerOn();
        }
        else if(gamepad2.left_trigger > 0.1)
            ActiveIntake.Reverse();
        else {
            Extendo.DropDown(0);
            ActiveIntake.powerOff();
        }
        if(Controls.RetractExtendo){
            if(Extendo.CurrentState == Extendo.States.IDLE)
                Extendo.CurrentState = Extendo.States.RETRACT;

            Controls.RetractExtendo = false;
        }



        gamepad2.update();
        gamepad1.update();
    }
}
