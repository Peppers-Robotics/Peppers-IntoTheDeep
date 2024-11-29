package org.firstinspires.ftc.teamcode.TuneModes.Localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Localization.ThreeDeadWheelLocalizer;

@TeleOp
public class LateralDistance extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Controls.Initialize(gamepad1, gamepad2);
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeChassis();

        ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            Initialization.updateCacheing();
            Chassis.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    gamepad1.left_trigger - gamepad1.right_trigger);


            Controls.Update();
            localizer.update();
//            telemetry.addData("x", localizer.getCurrentPosition().getX(DistanceUnit.MM));
//            telemetry.addData("y", localizer.getCurrentPosition().getY(DistanceUnit.MM));
//            telemetry.addData("heading", localizer.getCurrentPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
