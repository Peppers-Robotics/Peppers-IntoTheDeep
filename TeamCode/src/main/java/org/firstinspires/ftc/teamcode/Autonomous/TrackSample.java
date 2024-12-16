package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HelperClasses.LimeLightHelpers.LimeLightColorTracking;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Autonomous(group = "limelight color")
@Config
public class TrackSample extends LinearOpMode {
    public static PIDController TController = new PIDController(SampleMecanumDriveCancelable.TRANSLATIONAL_PID);
    public static double kp = 0;
    SampleMecanumDriveCancelable drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Initialization.initializeRobot(hardwareMap);
        Initialization.initializeLimeLight();

        waitForStart();

        while (opModeIsActive()){

            double x = LimeLightColorTracking.getTx();
            double xD = 0;
            double y = TController.calculatePower(LimeLightColorTracking.getTy());
            if(Extendo.isMaxExtended()){
                xD = TController.calculatePower(x);
            } else Extendo.Extend((int) (Extendo.getCurrentPosition() + x * kp));

            drive.setWeightedDrivePower(new Pose2d(y, xD, 0));

            drive.update();
            Initialization.updateCacheing();
        }
    }
}
