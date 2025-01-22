package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

import java.io.BufferedReader;
import java.util.List;

public class Robot {
    public static List<LynxModule> hubs;
    public static Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    public static IMU imu;
    public static DcMotorControllerEx ControlHubMotors, ExpansionHubMotors;
    public static ServoController ControlHubServos, ExpansionHubServos;
    public static double VOLTAGE = 12;
    public static void InitializeHubs(HardwareMap hm){
        hubs = hm.getAll(LynxModule.class);
        boolean s = hubs.get(0).getImuType() == LynxModuleImuType.BHI260;
        ControlHubMotors = hm.getAll(DcMotorControllerEx.class).get(0);
        ControlHubMotors = hm.getAll(DcMotorControllerEx.class).get(1);

        ControlHubServos = hm.getAll(ServoController.class).get(0);
        ControlHubServos = hm.getAll(ServoController.class).get(1);

        imu = hm.get(IMU.class, "imu");
        VOLTAGE = hm.getAll(VoltageSensor.class).get(0).getVoltage();

        for(LynxModule l : hubs){
            l.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public static void clearCache(){
        for(LynxModule l : hubs){
            l.clearBulkCache();
        }
        telemetry.update();
    }
    public static void InitializeFull(HardwareMap hm){
        InitializeHubs(hm);
    }

    public static void InitializeExtendo(){
        Extendo.motor = new CachedMotor(ControlHubMotors, 0, DcMotorSimple.Direction.FORWARD);
        Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void InitializeDropDown(){
        DropDown.servo = new ServoPlus(ExpansionHubServos, 5, Servo.Direction.FORWARD);
    }
    public static void InitializeElevator(){
        Elevator.motor = new CachedMotor(ExpansionHubMotors, 3, DcMotorSimple.Direction.FORWARD);
        Elevator.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void InitializeArm(){
        Arm.servo1 = new ServoPlus(ExpansionHubServos, 1, Servo.Direction.FORWARD);
        Arm.servo2 = new ServoPlus(ExpansionHubServos, 2, Servo.Direction.FORWARD);
    }
    public static void InitializeClaw(){
        Claw.clawServo = new ServoPlus(ControlHubServos, 1, Servo.Direction.FORWARD);
    }
    public static void InitializeClimb(){
        Climb.W1 = new ServoPlus(ExpansionHubServos, 0, Servo.Direction.FORWARD);
        Climb.W2 = new ServoPlus(ExpansionHubServos, 4, Servo.Direction.FORWARD);
        Climb.PTO1 = new ServoPlus(ExpansionHubServos, 3, Servo.Direction.FORWARD);
        Climb.PTO2 = new ServoPlus(ControlHubServos, 0, Servo.Direction.FORWARD);
        Climb.run = Climb.climb;
    }
}
