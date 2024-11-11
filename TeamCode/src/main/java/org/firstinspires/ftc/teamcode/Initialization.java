package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

import java.nio.charset.CharacterCodingException;
import java.util.List;

public class Initialization {
    public static List<LynxModule> hubs;
    public enum AllianceColor{
        RED,
        BLUE
    }
    public static AllianceColor Team;
    public static void initializeHubCacheing(@NonNull HardwareMap hm){
        hubs = hm.getAll(LynxModule.class);
        hubs.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        hubs.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    public static void initializeExtendo(@NonNull HardwareMap hm){
        Extendo.motor = (CachedMotor) hm.get(DcMotor.class, "cM0");
        Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Extendo.dropDownIntakeLeft = (ServoPlus) hm.get(Servo.class, "cS0");
        Extendo.dropDownIntakeRight = (ServoPlus) hm.get(Servo.class, "cS4");
    }
    public static void initializeStorage(@NonNull HardwareMap hm){
        Storage.sensor = hm.get(FastColorRangeSensor.class, "Storage");
    }
    public static void initializeElevator(@NonNull HardwareMap hm){
        Elevator.motor = (CachedMotor) hm.get(DcMotor.class, "cM1");
        Elevator.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void initializeOuttake(@NonNull HardwareMap hm){
        Claw.clawServo = (ServoPlus) hm.get(Servo.class, "cS1");
        Arm.servo1 = (ServoPlus) hm.get(Servo.class, "cS2");
        Arm.servo2 = (ServoPlus) hm.get(Servo.class, "cS3");
        Claw.clawSensor = hm.get(FastColorRangeSensor.class, "Claw");
    }
    public static void initializeIntake(@NonNull HardwareMap hm){
        ActiveIntake.motor = (CachedMotor) hm.get(DcMotor.class, "cM2");
        ActiveIntake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public static void initializeRobot(@NonNull HardwareMap hm){
        initializeIntake(hm);
        initializeElevator(hm);
        initializeExtendo(hm);
        initializeOuttake(hm);
        initializeStorage(hm);
        initializeHubCacheing(hm);
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
}
