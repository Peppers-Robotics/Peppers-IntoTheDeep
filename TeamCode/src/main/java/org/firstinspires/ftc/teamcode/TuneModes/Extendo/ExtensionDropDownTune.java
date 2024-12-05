package org.firstinspires.ftc.teamcode.TuneModes.Extendo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.DropDown;

@TeleOp(group = "Intake")
@Config
public class ExtensionDropDownTune extends LinearOpMode {
    public static boolean goUp = false, goDown = false, goMiddle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeExtendo();

        waitForStart();

        while (opModeIsActive()){
            if(goUp){
                DropDown.GoUp();
                goUp = false;
            }
            if(goDown){
                DropDown.GoDown();
                goDown = false;
            }
            if(goMiddle){
                DropDown.GoMiddle();
                goMiddle = false;
            }
            DropDown.Update();
            Initialization.telemetry.update();
        }
    }
}
