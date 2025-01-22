package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class StorageTest extends LinearOpMode {
    public static boolean climb = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeClimb();
        Robot.InitializeExtendo();
        waitForStart();

        while (opModeIsActive()){
            Robot.clearCache();

            if(climb)
                Climb.Update();
        }
    }
}
