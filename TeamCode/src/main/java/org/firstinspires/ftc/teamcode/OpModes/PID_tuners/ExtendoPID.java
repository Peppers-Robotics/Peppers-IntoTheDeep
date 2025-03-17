package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@TeleOp
public class ExtendoPID extends LinearOpMode {
    public static int position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeDropDown();
        Robot.InitializeExtendo();

        waitForStart();

        while(opModeIsActive()){
            DropDown.setDown(0);
            Robot.clearCache();
            Robot.telemetry.addData("Extendo powerConsumption", Extendo.motor.getCurrent(CurrentUnit.AMPS));
            Extendo.Extend(position);
            Extendo.update();
        }
    }
}
