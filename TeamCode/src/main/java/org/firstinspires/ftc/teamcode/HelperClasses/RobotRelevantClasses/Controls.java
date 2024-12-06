package org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.AutoGamepad;
import org.firstinspires.ftc.teamcode.Intake.Storage;

@Config
public class Controls {
    public static AutoGamepad gamepad1, gamepad2;

    public static boolean RetractExtendo, ScoreLevel1, ScoreLevel2, GrabSpecimen, Climbing,
            Grab,
            Retract,
                            DunkToScore, IdleWithSample, Throw;

    public static void Initialize(Gamepad gamepadD1, Gamepad gamepadD2){
        gamepad1 = new AutoGamepad(gamepadD1);
        gamepad2 = new AutoGamepad(gamepadD2);
    }

    private static void reset(){
        if(RetractExtendo) RetractExtendo = false;
        if(ScoreLevel1) ScoreLevel1 = false;
        if(ScoreLevel2) ScoreLevel2 = false;
        if(GrabSpecimen) GrabSpecimen = false;
    }
    private static boolean ClimbingHelp = false;

    public static void Update(){
        if(gamepad1.wasPressed.circle)      Throw        = true;
        if(gamepad2.wasPressed.dpad_down)   ScoreLevel1  = true;
        if(gamepad2.wasPressed.dpad_up)     ScoreLevel2  = true;
        if(gamepad2.wasPressed.triangle)    GrabSpecimen = true;
        if(gamepad1.wasPressed.square || gamepad2.wasPressed.square)
                                            Grab         = true;
        if(gamepad2.wasPressed.circle)      Retract      = true;
        if(gamepad2.wasPressed.dpad_left || gamepad2.wasPressed.dpad_right)
                                            DunkToScore  = true;
        if((gamepad2.gamepad.left_bumper && gamepad2.gamepad.right_bumper) && (gamepad1.gamepad.right_bumper && gamepad1.gamepad.left_bumper) && !ClimbingHelp) {
            Climbing = true;
            ClimbingHelp = true;
        } else ClimbingHelp = false;

        if(gamepad1.wasPressed.ps){
            if(Storage.sensor.isLightOn()) Storage.sensor.enableLed(false);
            else Storage.sensor.enableLed(true);
        }

        gamepad1.update();
        gamepad2.update();
    }

}
