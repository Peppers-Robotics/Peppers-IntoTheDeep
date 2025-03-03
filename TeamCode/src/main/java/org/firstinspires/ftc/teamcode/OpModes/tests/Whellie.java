package org.firstinspires.ftc.teamcode.OpModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class Whellie extends LinearOpMode {
    public static boolean wh = false;
    public static boolean pto = false;
    public static double pow = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeClimb();
        Robot.InitializeChassis();

        waitForStart();

        while(opModeIsActive()){
            if(wh){
                Climb.ActivateWheelie();
            } else {
                Climb.DeactivateWheelie();
            }
            if(pto){
                Climb.EngagePTO();
            } else {
                Climb.DisengagePTO();
            }
            Chassis.drive(pow, 0, 0);
        }
    }
}
