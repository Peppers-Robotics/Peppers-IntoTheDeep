package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeLogic;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class OpModeManager {
    public HardwareMap hardwareMap;
    public Gamepad gamepad1, gamepad2;
    public Telemetry telemetry;

    public boolean isClimbing = false;
    public OpModeManager(HardwareMap hm, Gamepad g1, Gamepad g2, Telemetry t, Storage.Team team){
        hardwareMap = hm;
        gamepad1 = g1;
        gamepad2 = g2;
        telemetry = t;

        Storage.team = team;

        Robot.InitializeFull(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);
        IntakeLogic.Initialize(gamepad1, gamepad2);
        Extendo.pidController.setFreq(40);
        Elevator.controller.setFreq(40);

        Extendo.Extend(0);
        ActiveIntake.Block();
        Claw.open();
    }
    public static double getSquaredSigned(double h){
        return Math.signum(h) * (h * h);
    }
    public static double getPowerSigned(double h, double p){
        double sgn = Math.signum(h);
        h = Math.abs(h);
        for(int i = 1; i < p; i++){
            h *= h;
        }
        return sgn * h;
    }
    public void update(){
        Robot.clearCache();

        if(Controls.Climbing && !isClimbing){
            Climb.run = Climb.climb.clone();
            isClimbing = true;
            Controls.Climbing = false;
        }
        if(isClimbing){
            Climb.Update();
            return;
        }

        Chassis.drive(getPowerSigned(gamepad1.left_stick_x, 3), -getPowerSigned(gamepad1.left_stick_y, 3), getPowerSigned(gamepad1.right_trigger - gamepad1.left_trigger, 3));
        OutTakeLogic.update();
        IntakeLogic.update();
        Extendo.update();
        Elevator.update();
        Arm.update();
        Controls.Update();
    }
}
