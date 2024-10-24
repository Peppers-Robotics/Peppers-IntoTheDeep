package org.firstinspires.ftc.teamcode.TuneModes.Extendo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;

@TeleOp(group = "Intake")
@Config
public class ExtensionDropDownTune extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeRobot(hardwareMap);

        Extendo.dropDownIntakeLeft.getController().pwmEnable();
        Extendo.dropDownIntakeRight.getController().pwmEnable();

        waitForStart();

        while (opModeIsActive()){
            Extendo.DropDown(0);
        }
    }
}
