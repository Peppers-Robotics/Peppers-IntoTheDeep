package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.bumblebee.Localizers.Pose2d;
import com.ftc.bumblebee.PathGenerators.PurePursuit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.PinPoint;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.PinPointLocalizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.Locale;
import java.util.Vector;

@TeleOp(name = "PurePursuitTest")
@Config
public class PurePursuitTest extends LinearOpMode {
    public static double radius = 500;
    @Override
    public void runOpMode() {
        Robot.InitializeFull(hardwareMap);
        Extendo.Extend(0);
        Arm.setArmAngle(0);
        DropDown.setDown(0);

        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        PinPointLocalizer localizer = new PinPointLocalizer(hardwareMap.get(PinPoint.class, "pinpoint"));

        Vector<Pose2d> check = new Vector<>();
        check.add(new Pose2d(0, 0, 0));
        check.add(new Pose2d(2030, 30, Math.PI / 2));
        check.add(new Pose2d(2030, 2050, Math.PI));
        check.add(new Pose2d(-272, 2050, Math.PI));


        PurePursuit trajectory = new PurePursuit(check, radius);

        waitForStart();

        while (opModeIsActive()) {
            Robot.clearCache(true);
            // OpMode loop
            Arm.update();
            Elevator.update();
            Pose2d c = new Pose2d(Localizer.getCurrentPosition().x, Localizer.getCurrentPosition().y, Localizer.getCurrentPosition().h);
            Pose2d next = trajectory.getNextHeadingPoint(c);
            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(next.x, next.y, next.h));
            Chassis.Update();
            localizer.update();
            Robot.telemetry.addData("next point", next.x + ", " + next.y + " ", next.h);
        }
    }
}
