package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@TeleOp
public class ExtendoPID extends LinearOpMode {
    public static int position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeExtendo();

        waitForStart();

        while(opModeIsActive()){
            Robot.clearCache();

            Extendo.Extend(position);
            Extendo.update();
        }
    }
}
