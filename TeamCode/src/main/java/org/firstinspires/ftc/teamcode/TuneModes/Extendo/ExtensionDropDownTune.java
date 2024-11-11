package org.firstinspires.ftc.teamcode.TuneModes.Extendo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;

import java.nio.file.attribute.DosFileAttributes;

@TeleOp(group = "Intake")
@Config
public class ExtensionDropDownTune extends LinearOpMode {

    public static double Distance = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Extendo.dropDownIntakeLeft = new ServoPlus(hardwareMap.get(Servo.class, "cS2"));
        Extendo.dropDownIntakeRight = new ServoPlus(hardwareMap.get(Servo.class, "cS3"));

        waitForStart();

        while (opModeIsActive()){
            Extendo.DropDown(Distance);
        }
    }
}
