package org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.AutoGamepad;

public class GenericController {
    public static AutoGamepad gamepad1, gamepad2;
    public static void Initialize(Gamepad D1, Gamepad D2){
        gamepad1 = new AutoGamepad(D1);
        gamepad2 = new AutoGamepad(D2);
    }
}
