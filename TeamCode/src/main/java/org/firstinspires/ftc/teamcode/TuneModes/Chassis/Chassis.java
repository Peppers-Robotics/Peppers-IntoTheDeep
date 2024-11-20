package org.firstinspires.ftc.teamcode.TuneModes.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization;

@TeleOp(group = "tests")
public class Chassis extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.InitializeChassis(hardwareMap);
        Initialization.initializeHubCacheing(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double h = -gamepad1.right_trigger + gamepad1.left_trigger;
            org.firstinspires.ftc.teamcode.Chassis.drive(x, y, -h);

            Initialization.updateCacheing();
        }
    }
}
