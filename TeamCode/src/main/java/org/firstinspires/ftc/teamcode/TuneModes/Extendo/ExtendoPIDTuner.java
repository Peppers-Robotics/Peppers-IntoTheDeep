package org.firstinspires.ftc.teamcode.TuneModes.Extendo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Initialization;

@SuppressWarnings("unused")
@TeleOp(group = "IntakeTuning")
@Config
public class ExtendoPIDTuner extends LinearOpMode {
    public static double TargetPosition = 0;
    public static boolean AutoUpdateTargetPosition = false;
    public static double freqToUpdateTargetPosition = 1, step = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeRobot(hardwareMap);
        ElapsedTime time = new ElapsedTime();

        Thread updater = new Thread(() -> {
            while(opModeIsActive()) {
                Initialization.updateCacheing();
                Extendo.update();
            }
        });

        waitForStart();
        time.reset();
        updater.start();

        while (opModeIsActive()){
            Extendo.DropDown(0);
            if(AutoUpdateTargetPosition && time.seconds() > freqToUpdateTargetPosition){
                TargetPosition = step - TargetPosition;
                time.reset();
            }

            Extendo.Extend((int) TargetPosition);

        }
        Initialization.UninitializeRobot();
    }
}
