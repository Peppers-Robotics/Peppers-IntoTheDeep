package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class DropDownTest extends LinearOpMode {
    public static double precent = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeExtendo();
        Robot.InitializeDropDown();
        waitForStart();

        while (opModeIsActive()){
            DropDown.setDown(precent);
            Robot.clearCache();
        }
    }
}
