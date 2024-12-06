package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.HelperClasses.LimeLightHelpers.LimeLightColorTracking;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
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
    public static HardwareMap hardwareMap;
    public static void initializeHubCacheing(@NonNull HardwareMap hm){
        hardwareMap = hm;

        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule l : hubs){
            l.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        try {
            Voltage = hubs.get(0).getInputVoltage(VoltageUnit.VOLTS);
        } catch (Exception e){
            Voltage = 12;
        }
    }
    public static void initializeExtendo(){
        try {
            Extendo.motor = new CachedMotor(hardwareMap.get(DcMotor.class, "cM1"));

            Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            DropDown.DropDownRight = hardwareMap.get(ServoPlus.class, "eS2");
            DropDown.DropDownLeft = hardwareMap.get(ServoPlus.class, "eS4");
        } catch (Exception e){
            RobotLog.e("Extendo motor and servos not found");
        }
    }
    // eS1 - hang1
    // es0 - hang2
    public static void initializeStorage(@NonNull HardwareMap hm){
        try {
            Storage.sensor = hm.get(FastColorRangeSensor.class, "Storage");
            Storage.sensor.enableLed(true);
        } catch (Exception e){
            RobotLog.e("Storage sensor not found");
        }
    }
    public static void initializeElevator(){
        try {
            Elevator.motor = new CachedMotor(hardwareMap.get(DcMotor.class, "eM0"));
//        Elevator.motor = new CachedMotor(eM, 0);
            Elevator.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e){
            RobotLog.e("Elevator motor not found");
        }
    }
    public static void initializeOuttake(@NonNull HardwareMap hm){
        try {
            Claw.clawServo = hm.get(ServoPlus.class, "cS2");
            Arm.servo1 = hm.get(ServoPlus.class, "eS3");
            Arm.servo2 = hm.get(ServoPlus.class, "eS5");
            Claw.clawSensor = hm.get(FastColorRangeSensor.class, "Claw");
        } catch (Exception e){
            RobotLog.e("Outtake servos not found");
        }
    }
    public static void initializeIntake(){
        try {
            ActiveIntake.motor = new CachedMotor(hardwareMap.get(DcMotor.class, "eM3"));
//        ActiveIntake.motor = new CachedMotor(eM, 3);
            ActiveIntake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ActiveIntake.Blocker = hardwareMap.get(ServoPlus.class, "cS0");
        } catch (Exception e){
            RobotLog.e("Intake servos and motors not found");
        }
//        ActiveIntake.Blocker = new ServoPlus(cS, 0);
        // PTO - cS4
    }
    public static void initializeRobot(@NonNull HardwareMap hm){
        initializeHubCacheing(hm);
        Initialization.hubs.get(0).disengage();
        Initialization.hubs.get(1).disengage();
        initializeIntake();
        initializeElevator();
        initializeExtendo();
        initializeOuttake(hm);
        initializeStorage(hm);
        initializeChassis();
        initializeClimb();

        Claw.clawSensor.setLowPassFilterCoefficient(0.9);
    }
    public static void initializeClimb(){
//        Climb.PTO = new ServoPlus(cS, 4);
//        Climb.W1 = new ServoPlus(eS, 1);
//        Climb.W2 = new ServoPlus(eS, 0);
        try {
            Climb.PTO = hardwareMap.get(ServoPlus.class, "cS4");
            Climb.W1 = hardwareMap.get(ServoPlus.class, "eS1");
            Climb.W2 = hardwareMap.get(ServoPlus.class, "eS0");
        } catch (Exception e){
            RobotLog.e("Climb servos not found");
        }
        Climb.disengagePTO();
        Climb.PutDown();
    }
    public static void initializeLimeLight(){
        LimeLightColorTracking.camera = hardwareMap.get(Limelight3A.class, "camera");
    }

    public static void startDevices(){

        Arm.servo1.getController().pwmEnable();
        Arm.servo2.getController().pwmEnable();

        Claw.clawServo.getController().pwmEnable();

    }

    public synchronized static void updateCacheing(){
        if(hubs == null){
            Initialization.initializeHubCacheing(hardwareMap);
        }
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

        Claw.clawServo.getController().pwmDisable();
    }
    public static void initializeChassis(){
        try {
            Chassis.BL = new CachedMotor(hardwareMap.get(DcMotor.class, "cM2"));
            Chassis.BR = new CachedMotor(hardwareMap.get(DcMotor.class, "eM2"));
            Chassis.FL = new CachedMotor(hardwareMap.get(DcMotor.class, "cM3"));
            Chassis.FR = new CachedMotor(hardwareMap.get(DcMotor.class, "eM1"));
        } catch (Exception e){
            RobotLog.e("Chassis motors not found");
        }

        Chassis.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

        /*
        * cM2 - stanga spate
        * cM3 - stanga fata
        * eM1 - drapta fata
        * eM2 - drapta spate
        * */
}
