package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "IntakeTest")
@Config
public class IntakeTest extends LinearOpMode {
    public static double pow = 0;
    @Override
    public void runOpMode() {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeActiveIntake();
        Robot.enable();

        waitForStart();
        while (opModeIsActive()) {
            Robot.clearCache(true);
            Robot.telemetry.addData("Active intake power consumption", ActiveIntake.motor.getCurrent(CurrentUnit.AMPS));
            ActiveIntake.motor.setPower(pow);

        }
    }
}
