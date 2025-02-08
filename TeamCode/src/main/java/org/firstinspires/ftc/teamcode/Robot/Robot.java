package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Bitmap;
import android.media.MediaCodecInfo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMUNew;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.FastColorRangeSensor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.PinPoint;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2Impl;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.BufferedReader;
import java.util.List;

public class Robot {
    public static List<LynxModule> hubs;
    public static Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    public static IMU imu;
    public static DcMotorController ControlHubMotors, ExpansionHubMotors;
    public static ServoController ControlHubServos, ExpansionHubServos;
    public static double VOLTAGE = 12;
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

    public static class LLStream implements CameraStreamSource {
        private VideoCapture c;
        public LLStream(){
            c = new VideoCapture();
            c.open("http://172.28.0.1:5800/stream.mjpg");
        }
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

        }
    }

    public static void startLimeLightStream(Limelight3A camera){

    }

    public static void InitializeHubs(HardwareMap hm){
        hubs = hm.getAll(LynxModule.class);
        boolean s = hubs.get(0).getImuType() == LynxModuleImuType.BHI260;

        ControlHubMotors = hm.get(DcMotorController.class, "Control Hub");
        ExpansionHubMotors = hm.get(DcMotorController.class, "Expansion Hub 2");

        ControlHubServos = hm.get(ServoController.class, "Control Hub");
        ExpansionHubServos = hm.get(ServoController.class, "Expansion Hub 2");
        MotorConfigurationType mct;

        for(int i = 0; i < 4; i++){
            mct = ControlHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ControlHubMotors.setMotorType(i, mct);

            mct = ExpansionHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ExpansionHubMotors.setMotorType(i, mct);
        }


        imu = hm.get(IMU.class, "imuProst");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
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
        disable();
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
    }
    public static void InitializeChassis(){
        Chassis.FL = new CachedMotor(ExpansionHubMotors, 2, DcMotorSimple.Direction.FORWARD);
        Chassis.FR = new CachedMotor(ControlHubMotors, 2, DcMotorSimple.Direction.FORWARD);
        Chassis.BL = new CachedMotor(ControlHubMotors, 1, DcMotorSimple.Direction.FORWARD);
        Chassis.BR = new CachedMotor(ExpansionHubMotors, 1, DcMotorSimple.Direction.FORWARD);

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
    }
    public static void InitializeExtendo(){
        Extendo.motor = new CachedMotor(ControlHubMotors, 0, DcMotorSimple.Direction.FORWARD);
        Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void InitializeDropDown(){
        DropDown.servo = new ServoPlus(ExpansionHubServos, 5, Servo.Direction.FORWARD);
    }
    public static void InitializeActiveIntake(){
        ActiveIntake.motor = new CachedMotor(ExpansionHubMotors, 0, DcMotorSimple.Direction.FORWARD);
        ActiveIntake.blocker = new ServoPlus(ControlHubServos, 0, Servo.Direction.FORWARD); // TODO: portul bun
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
//        Climb.PTO2 = new ServoPlus(ControlHubServos, 0, Servo.Direction.FORWARD);
        Climb.run = Climb.climb;
    }
}
