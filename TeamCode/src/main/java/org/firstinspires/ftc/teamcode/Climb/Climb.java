package org.firstinspires.ftc.teamcode.Climb;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Config
public class Climb {
    public static final Scheduler climb = new Scheduler();
    public static Scheduler run = new Scheduler();
    public static ServoPlus W1, W2, PTO1;
    public static double BAR1 = 190, BAR2 = 700;
    public static double pitch = 0;
    public static ElapsedTime time = new ElapsedTime();
    public static double EngagePTO1 = 180, EngagePTO2 = 200, DisengagePTO1 = 300, DisengagePTO2 = 170,
                         EngageWheelie1 = 260, DisengageWheelie1 = 173, EngageWheelie2 = 100, DisengageWheelie2 = 195, climbArmIntertia = 310;

    public static void EngagePTO(){
        PTO1.setAngle(EngagePTO1);
//        PTO2.setAngle(EngagePTO2);
    }
    public static void DisengagePTO(){
        PTO1.setAngle(DisengagePTO1);
//        PTO2.setAngle(DisengagePTO2);
    }
    public static void ActivateWheelie(){
        W1.setAngle(EngageWheelie1);
        W2.setAngle(EngageWheelie2);
    }
    public static void DeactivateWheelie(){
        W1.setAngle(DisengageWheelie1);
        W2.setAngle(DisengageWheelie2);
    }
    public static boolean PTOActivated(){
        if(PTO1 == null) return false;
        return PTO1.isEqualToAngle(EngagePTO1);
    }
    public static boolean IsWheelieActive(){
        if(W1 == null) return false;
        return W1.isEqualToAngle(EngageWheelie1);
    }

    static {
        climb.
                addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActivateWheelie();
                        DisengagePTO();
//                        Extendo.motor.setPower(-0.4);
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(BAR1 + 150);
                        return Elevator.getCurrentPosition() >= BAR1 + 50;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(BAR1 + 80);
//                        return Elevator.getCurrentPosition() <= BAR1 + 100;
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.Disable = true;
                        EngagePTO();
                        Chassis.drive(-0.2, 0, 0);
//                        Elevator.setTargetPosition(BAR1 + 60);
                        Elevator.motor.setPower(-0.5);
                        Elevator.motor2.setPower(-0.5);
                        return true;
                    }
                })

                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.Disable = false;
                        MotorConfigurationType mct = Elevator.motor.getMotorType();
//                        mct.setAchieveableMaxRPMFraction(0.7);
//                        Elevator.motor.setMotorType(mct);
//                        Elevator.motor2.setMotorType(mct);
                        Elevator.setTargetPosition(-20);
                        return Elevator.getCurrentPosition() < 5;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        DisengagePTO();
                        Elevator.Disable = true;
                        DeactivateWheelie();
                        time.reset();
                        return true; // MAKE IT TRUE
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Elevator.getCurrentPosition() >= 30 || time.seconds() >= 0.6;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        DisengagePTO();
                        Elevator.Disable = false;
                        Elevator.setTargetPosition(BAR2);
                        Elevator.Disable = pitch < 3.5 && Elevator.getCurrentPosition() > BAR2 - 400;
                        return Elevator.getCurrentPosition() >= BAR2 - 10;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(climbArmIntertia);
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(pitch < 4){
                            Elevator.setTargetPosition(BAR2 - 250);
                            return true;
                        }
                        return false;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Elevator.getCurrentPosition() < BAR2 - 100;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        EngagePTO();
                        Chassis.drive(-0.4, 0, 0);
                        Elevator.Disable = true;
                        Elevator.motor.setPower(0.4);
                        Elevator.motor2.setPower(0.4);
//                        Chassis.BL.setPower(0);
//                        Chassis.BR.setPower(0);
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.Disable = false;
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(-100);
                        MotorConfigurationType mct = Elevator.motor.getMotorType();
                        mct.setAchieveableMaxRPMFraction(1);
                        Elevator.motor.setMotorType(mct);
                        Elevator.motor2.setMotorType(mct);
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(Elevator.getCurrentPosition() <= 0) {
                            Elevator.setTargetPosition(0);
                            return true;
                        }
                        return false;
                    }
                })
        ;
    }
    public static void Update(){
        pitch = Robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        Extendo.motor.setPower(-0.6);
        run.update();

        Elevator.update();
        Arm.update();

        Robot.telemetry.addData("tasks", Integer.valueOf(climb.tasks.size() - run.tasks.size()).toString() + "/" + climb);
        Robot.telemetry.addData("pitch", pitch);
    }
}
