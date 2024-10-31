package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HelperClasses.AutoGamepad;
import org.firstinspires.ftc.teamcode.HelperClasses.GenericController;

public class IntakeController extends GenericController {
    private static int ExtendoPosition, MaxExtension = 1200;
    private static int Resolution;
    static {
        ExtendoPosition = 0;
        Resolution = 0;
    }
    public static void update(){
        ExtendoPosition += (int) (-gamepad1.right_stick_y * Resolution);
        if(ExtendoPosition < 0) ExtendoPosition = 0;
        if(ExtendoPosition > MaxExtension) ExtendoPosition = MaxExtension;

        Extendo.Extend(ExtendoPosition);


        if(gamepad2.right_trigger > 0.2) {
            Extendo.DropDown(Extendo.MaxExtension);
            ActiveIntake.powerOn();
        }
        else if(gamepad2.left_trigger > 0.2)
            ActiveIntake.Reverse();
        else {
            Extendo.DropDown(0);
            ActiveIntake.powerOff();
        }

    }
}
