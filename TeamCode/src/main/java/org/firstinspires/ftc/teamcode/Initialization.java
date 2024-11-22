package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

import java.util.List;

public class Initialization {
    public static List<LynxModule> hubs;
    public enum AllianceColor{
        RED,
        BLUE
    }
    public static AllianceColor Team;
    public static Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    public static double Voltage = 12;
    public static void initializeHubCacheing(@NonNull HardwareMap hm){
        hubs = hm.getAll(LynxModule.class);
        for(LynxModule l : hubs){
            l.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        try {
            Voltage = hubs.get(0).getInputVoltage(VoltageUnit.VOLTS);
        } catch (Exception e){
            Voltage = 12;
        }
    }
    public static void initializeExtendo(@NonNull HardwareMap hm){
        Extendo.motor = new CachedMotor(hm.get(DcMotor.class, "cM1"));
//        Extendo.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Extendo.dropDownIntakeLeft = hm.get(ServoPlus.class, "eS4");
        Extendo.dropDownIntakeRight = hm.get(ServoPlus.class, "eS2");
    }
    // eS1 - hang1
    // es0 - hang2
    public static void initializeStorage(@NonNull HardwareMap hm){
        Storage.sensor = hm.get(FastColorRangeSensor.class, "Storage");
    }
    public static void initializeElevator(@NonNull HardwareMap hm){
        Elevator.motor = new CachedMotor (hm.get(DcMotor.class, "eM0"));
        Elevator.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void initializeOuttake(@NonNull HardwareMap hm){
        Claw.clawServo = hm.get(ServoPlus.class, "cS2");
        Arm.servo1 = hm.get(ServoPlus.class, "eS3");
        Arm.servo2 = hm.get(ServoPlus.class, "eS5");
        Claw.clawSensor = hm.get(FastColorRangeSensor.class, "Claw");
    }
    public static void initializeIntake(@NonNull HardwareMap hm){
        ActiveIntake.motor = new CachedMotor(hm.get(DcMotor.class, "eM3"));
        ActiveIntake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ServoPlus latch = hm.get(ServoPlus.class, "cS0");
        // PTO - cS4
    }
    public static void initializeRobot(@NonNull HardwareMap hm){
        initializeIntake(hm);
        initializeElevator(hm);
        initializeExtendo(hm);
        initializeOuttake(hm);
        initializeStorage(hm);
        initializeHubCacheing(hm);
        initializeChassis(hm);
//        initializeClimb(hm);

        Claw.clawSensor.setLowPassFilterCoefficient(0.9);
    }
    public static void initializeClimb(HardwareMap hm){
        Climb.PTO = hm.get(ServoPlus.class, "cS4");
        Climb.W1 = hm.get(ServoPlus.class, "eS1");
        Climb.W2 = hm.get(ServoPlus.class, "eS0");
        Climb.disengagePTO();
        Climb.PutDown();
    }

    public static void startDevices(){

        Arm.servo1.getController().pwmEnable();
        Arm.servo2.getController().pwmEnable();

        Extendo.dropDownIntakeRight.getController().pwmEnable();
        Extendo.dropDownIntakeLeft.getController().pwmEnable();

        Claw.clawServo.getController().pwmEnable();

    }

    public synchronized static void updateCacheing(){
        for(LynxModule h : hubs){
            h.clearBulkCache();
        }
    }

    public static void UninitializeRobot(){
        for(int i = 0; i < 4; i++){
            Extendo.motor.getController().setMotorPower(i, 0);
        }
        Arm.servo1.getController().pwmDisable();
        Arm.servo2.getController().pwmDisable();

        Extendo.dropDownIntakeRight.getController().pwmDisable();
        Extendo.dropDownIntakeLeft.getController().pwmDisable();

        Claw.clawServo.getController().pwmDisable();
    }
    public static void initializeChassis(HardwareMap hm){
        Chassis.BL = new CachedMotor(hm.get(DcMotorEx.class, "cM2"));
        Chassis.BR = new CachedMotor(hm.get(DcMotorEx.class, "eM2"));
        Chassis.FL = new CachedMotor(hm.get(DcMotorEx.class, "cM3"));
        Chassis.FR = new CachedMotor(hm.get(DcMotorEx.class, "eM1"));
    }

        /*
        * cM2 - stanga spate
        * cM3 - stanga fata
        * eM1 - drapta fata
        * eM2 - drapta spate
        *
        *
        * */
}
