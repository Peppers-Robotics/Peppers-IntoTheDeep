package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeLogic;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.Extension;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;

@Config
public class OpModeManager {
    public HardwareMap hardwareMap;
    public Gamepad gamepad1, gamepad2;
    public Telemetry telemetry;
    public static double tSpeed = 1, rot = 0.7;
    public static double min = 0.4;
    public boolean isClimbing = false;
    public static boolean reverse = true;
    public static Thread thread;
    public OpModeManager(HardwareMap hm, Gamepad g1, Gamepad g2, Telemetry t, Storage.Team team){
        hardwareMap = hm;
        gamepad1 = g1;
        gamepad2 = g2;
        telemetry = t;
        IntakeLogic.IgnoreUntilNext = false;

        Storage.team = team;

        Robot.InitializeFull(hardwareMap);
        Robot.disable();
        Controls.Initialize(gamepad1, gamepad2);
        IntakeLogic.Initialize(gamepad1, gamepad2);
        Extendo.pidController.setFreq(40);
        Elevator.controller.setFreq(40);
        Limelight3A c = hardwareMap.get(Limelight3A.class, "camera");
        c.shutdown();

        Extendo.Extend(0);

        Elevator.setTargetPosition(Elevator.getTargetPosition());
        Climb.DeactivateWheelie();
        Climb.DisengagePTO();
        Arm.setArmAngle(Arm.getCurrentArmAngle());

//        Extension.Retract();
        Extension.Extend(Extension.getPrecent());
        Extendo.Extend(Extendo.getCurrentPosition());
        ActiveIntake.Unblock();
        Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Claw.open();
//        Chassis.Heading.setPidCoefficients(new PIDCoefficients(0.5, 0, 0));
//        Arm.setArmAngle(OutTakeLogic.ArmIdle);
        Rotation = 0;
        isClimbing = false;
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
    private static boolean resetPP = false;
    public static double Rotation = 0;
    public static double getPowerConsumption(){
        double ret = Chassis.FR.getCurrent(CurrentUnit.AMPS) + Chassis.FL.getCurrent(CurrentUnit.AMPS) +
                Chassis.BL.getCurrent(CurrentUnit.AMPS) + Chassis.BR.getCurrent(CurrentUnit.AMPS);
        return ret;
    }
    private static Scheduler autoScore;
    long freq = 0;
    private static boolean stop = false;
    public void update(){
        Robot.clearCache(true);

        if(Controls.Climbing && !isClimbing){
            Chassis.drive(0, 0, 0);
            Climb.run = Climb.climb.clone();
            isClimbing = true;
            Climb.DisengagePTO();
            Climb.ActivateWheelie();
            DropDown.setDown(0);
            Elevator.setTargetPosition(0);
            Controls.Climbing = false;
            Robot.telemetry.clearAll();
        }
        if(isClimbing){
            Climb.Update();
            Elevator.update();
            Arm.update();
            return;
        }

        if(Elevator.getCurrentPosition() > 500){
            tSpeed = 0.6;
        }
        else {
            tSpeed = 1;
        }
        double pow = (min - 1) / (Extendo.getMaxPosition()) * Extendo.getCurrentPosition() + 1;

        Chassis.drive(
                (reverse ? -1 : 1) * getPowerSigned(gamepad1.left_stick_x, 3) * tSpeed,
                (reverse ? 1 : -1) * getPowerSigned(gamepad1.left_stick_y, 3) * tSpeed,
                getPowerSigned(gamepad1.right_trigger - gamepad1.left_trigger, 3) * tSpeed * pow * rot
        );
        if(Controls.gamepad2.wasPressed.dpad_left) Claw.close();
        IntakeLogic.update();
        Extendo.update();
        Elevator.update();
        Arm.update();
        Controls.CleanCommands();
        Controls.Update();

        if(gamepad1.options) {
            Storage.getStorageStatus();
            Robot.telemetry.addData("r, g, b", Storage.sensor.RGB.R + ", " + Storage.sensor.RGB.G + ", " + Storage.sensor.RGB.B);
        }

    }
}
