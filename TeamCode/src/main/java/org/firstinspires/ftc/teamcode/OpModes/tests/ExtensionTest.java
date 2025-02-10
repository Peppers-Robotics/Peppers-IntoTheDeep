package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.OutTake.Extension;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class ExtensionTest extends LinearOpMode {
    public static double precent = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeExtension();

        waitForStart();

        while(opModeIsActive()){
            Extension.f = new LinearFunction(Extension.retractPos, Extension.extendoPos);
            Extension.Extend(precent);
        }
    }
}
