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
                            DunkToScore, Transfer, Throw, SlowDown, ImogenDriver, BlockTransfer;

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
    private static boolean ClimbingHelp = false, imogenHelper = false;


    public static void Update(){
        if((gamepad1.gamepad.triangle && gamepad1.gamepad.options) ||
           (gamepad2.gamepad.triangle && gamepad2.gamepad.options)){
            if(!imogenHelper) {
                ImogenDriver = !ImogenDriver;
                if (ImogenDriver) {
                    gamepad1.gamepad.setLedColor((double) 0xf8, (double) 0x86, (double) 0x05, (int) 1e10);
                    gamepad2.gamepad.setLedColor((double) 0xf8, (double) 0x86, (double) 0x05, (int) 1e10);
                } else {
                    gamepad2.gamepad.setLedColor((double) 0xba, (double) 0x00, (double) 0x71, (int) 1e10);
                    gamepad1.gamepad.setLedColor((double) 0xba, (double) 0x00, (double) 0x71, (int) 1e10);
                }
                imogenHelper = true;
            }
            return;
        } else imogenHelper = false;

        SlowDown = gamepad1.gamepad.left_bumper;

        if(ImogenDriver){
            if(gamepad1.wasPressed.dpad_down)   ScoreLevel1  = true;
            if(gamepad1.wasPressed.dpad_up)     ScoreLevel2  = true;
            if(gamepad1.wasPressed.square)      Grab         = true;
            if(gamepad1.wasPressed.circle)      Retract      = true;
            if(gamepad1.wasPressed.triangle)    GrabSpecimen = true;
            if(gamepad1.wasPressed.dpad_right)  Throw        = true;
            if(gamepad1.wasPressed.touchpad){
                Climbing = true;
                ClimbingHelp = true;
            } else ClimbingHelp = false;
            gamepad1.update();
            gamepad2.update();
            return;
        }


        if(gamepad1.wasPressed.dpad_right || gamepad1.wasPressed.dpad_left)    Throw        = true;
        if(gamepad2.wasPressed.dpad_down)   ScoreLevel1  = true;
        if(gamepad2.wasPressed.dpad_up)     ScoreLevel2  = true;
        if(gamepad2.wasPressed.triangle)    GrabSpecimen = true;
        if(gamepad2.wasPressed.square || gamepad1.wasPressed.square)      Grab         = true;
        if(gamepad2.wasPressed.circle)      Retract      = true;
        if(gamepad2.wasPressed.cross)       RetractExtendo = true;
        if(gamepad2.wasPressed.dpad_right)  DunkToScore  = true;
        if((gamepad2.gamepad.left_stick_button || gamepad2.gamepad.right_stick_button) &&
                (gamepad1.gamepad.left_stick_button || gamepad1.gamepad.right_stick_button) && !ClimbingHelp) {
            Climbing = true;
            ClimbingHelp = true;
        } else ClimbingHelp = false;
        if(gamepad1.wasPressed.triangle)    BlockTransfer = true;

        if(gamepad1.wasPressed.ps){
            if(Storage.sensor.isLightOn()) Storage.sensor.enableLed(false);
            else Storage.sensor.enableLed(true);
        }

        gamepad1.update();
        gamepad2.update();
    }

}
