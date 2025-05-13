package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.IMUBNO085;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.Extension;

import java.util.List;

public class Robot {
    public static List<LynxModule> hubs;

    public static Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    public static IMU imu;
    public static DcMotorController ControlHubMotors, ExpansionHubMotors;
    public static ServoController ControlHubServos, ExpansionHubServos, ServoHub;
    public static double VOLTAGE = 12;
    public static boolean isDisabled(){
        return !hubs.get(0).isEngaged();
    }
    public static void disable(){
        for(LynxModule l : hubs){
            l.disengage();
        }
    }
    public static void enable(){
        for(LynxModule l : hubs){
            l.engage();
        }
    }

    public static void startLimeLightStream(Limelight3A camera){

    }
    public static void InitializeHubs(HardwareMap hm){
        InitializeHubs(hm, false);
    }

    public static void InitializeHubs(HardwareMap hm, boolean b){
        if(hubs != null) return;
        hubs = hm.getAll(LynxModule.class);
//        if(b)
//            disable();

        ControlHubMotors = hm.get(DcMotorController.class, "Control Hub");
        ExpansionHubMotors = hm.get(DcMotorController.class, "Expansion Hub 2");

        ControlHubServos = hm.get(ServoController.class, "Control Hub");
        ExpansionHubServos = hm.get(ServoController.class, "Expansion Hub 2");

        IMUBNO085.controller = hm.get(DigitalChannelController.class, "Expansion Hub 2");
        try {
            ServoHub = hm.get(ServoController.class, "Servo Hub 1");
        } catch (Exception ignored){
            ServoHub = ControlHubServos;
        }
        MotorConfigurationType mct;

        for(int i = 0; i < 4; i++){
            mct = ControlHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ControlHubMotors.setMotorType(i, mct);

            mct = ExpansionHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ExpansionHubMotors.setMotorType(i, mct);
        }


//        imu = hm.get(IMUBNO085.class, "extIMU");
        imu = hm.get(IMU.class, "imuProst");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
//        Orientation o = new Orientation(
//                AxesReference.EXTRINSIC,
//                AxesOrder.ZXY,
//                AngleUnit.DEGREES,
//                0, 0, 0, 0
//        );
//        Robot.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(o)));

        imu.resetYaw();

        VOLTAGE = hm.getAll(VoltageSensor.class).get(0).getVoltage();

        for(LynxModule l : hubs){
            l.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public static void clearCache(){
        clearCache(true);
    }
    public static long loopTime = 0;
    private static ElapsedTime logFreq = new ElapsedTime();
    public static void clearCache(boolean update){
        for(LynxModule l : hubs){
            l.clearBulkCache();
        }
        if(update)
            telemetry.update();
        if(logFreq.seconds() >= 1) {
            RobotLog.ii("frequency", String.valueOf(1000.f / (System.currentTimeMillis() - loopTime)));
            logFreq.reset();
        }
        loopTime = System.currentTimeMillis();
    }

    public static void InitializeFull(HardwareMap hm, boolean disable){
        InitializeHubs(hm);
        if(disable){
            disable();
        }
        InitializeFull(hm);
    }
    public static void InitializeFull(HardwareMap hm){
        InitializeHubs(hm, true);
        InitializeElevator();
        InitializeExtendo();
        InitializeDropDown();
        InitializeClimb();
        InitializeClaw();
        InitializeArm();
        InitializeChassis();
        InitializeActiveIntake();
        InitializeLocalizer(hm);
        InitializeStorage(hm);
        InitializeExtension();
    }
    public static void InitializeExtension(){
        Extension.servo = new ServoPlus(ServoHub, 3, Servo.Direction.FORWARD);
    }
    public static void InitializeChassis(){
        Chassis.FL = new CachedMotor(ExpansionHubMotors, 2, DcMotorSimple.Direction.FORWARD);
        Chassis.FR = new CachedMotor(ControlHubMotors, 0, DcMotorSimple.Direction.FORWARD);
        Chassis.BL = new CachedMotor(ExpansionHubMotors, 3, DcMotorSimple.Direction.FORWARD);
        Chassis.BR = new CachedMotor(ControlHubMotors, 1, DcMotorSimple.Direction.FORWARD);

        Chassis.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void InitializeLocalizer(HardwareMap hm){
        Localizer.Initialize(hm);
    }
    public static void InitializeStorage(HardwareMap hm){
        Storage.sensor = hm.get(FastColorRangeSensor.class, "Storage");
//        Storage.sensor = null;
    }
    public static void InitializeExtendo(){
        Extendo.motor = new CachedMotor(ControlHubMotors, 3, DcMotorSimple.Direction.FORWARD);
        Extendo.encoder = new CachedMotor(ControlHubMotors, 3, DcMotorSimple.Direction.FORWARD);
        Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorConfigurationType m = Extendo.motor.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        Extendo.motor.setMotorType(m);
    }
    public static void InitializeDropDown(){
        DropDown.servo = new ServoPlus(ExpansionHubServos, 3, Servo.Direction.FORWARD);
    }
   public static void InitializeActiveIntake(){
        ActiveIntake.motor = new CachedMotor(ExpansionHubMotors, 0, DcMotorSimple.Direction.FORWARD);
        ActiveIntake.blocker = new ServoPlus(ExpansionHubServos, 2, Servo.Direction.FORWARD); // TODO: portul bun
    }
    public static void InitializeElevator(){
        Elevator.motor = new CachedMotor(ExpansionHubMotors, 1, DcMotorSimple.Direction.REVERSE);
        Elevator.motor2 = new CachedMotor(ControlHubMotors, 2, DcMotorSimple.Direction.REVERSE);
        Elevator.encoder = new CachedMotor(ControlHubMotors, 2, DcMotorSimple.Direction.REVERSE);
        Elevator.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elevator.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void InitializeArm(){
        Arm.servo2 = new ServoPlus(ServoHub, 2, Servo.Direction.FORWARD);
        Arm.servo1 = new ServoPlus(ServoHub, 5, Servo.Direction.FORWARD);
    }
    public static void InitializeClaw(){
        Claw.clawServo = new ServoPlus(ExpansionHubServos, 0, Servo.Direction.FORWARD);
    }
    public static void InitializeClimb(){
        Climb.W1 = new ServoPlus(ServoHub, 1, Servo.Direction.FORWARD);
        Climb.W2 = new ServoPlus(ServoHub, 4, Servo.Direction.FORWARD);
        Climb.PTO1 = new ServoPlus(ServoHub, 0, Servo.Direction.FORWARD);
//        Climb.PTO2 = new ServoPlus(ControlHubServos, 0, Servo.Direction.FORWARD);
        Climb.run = Climb.climb;
    }
}
