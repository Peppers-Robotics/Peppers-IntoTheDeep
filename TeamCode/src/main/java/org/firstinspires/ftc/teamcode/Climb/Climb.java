package org.firstinspires.ftc.teamcode.Climb;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

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
    public static double BAR1 = 400, BAR2 = 950;
    public static double pitch = 0;
    public static double EngagePTO1 = 130, EngagePTO2 = 200, DisengagePTO1 = 180, DisengagePTO2 = 170,
                         EngageWheelie1 = 295, DisengageWheelie1 = 195, EngageWheelie2 = 125, DisengageWheelie2 = 245, climbArmIntertia = 310;

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
                        Extendo.motor.setPower(-0.4);
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(BAR1 + 50);
                        return Elevator.getCurrentPosition() > BAR1 - 50;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(200);
                        return Elevator.getCurrentPosition() < 300;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        EngagePTO();
                        Chassis.FL.setPower(0);
                        Chassis.FR.setPower(0);
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(-20);
                        return Elevator.getCurrentPosition() < 8;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.Disable = true;
                        return true;
                    }
                })
                .waitSeconds(0.2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(0);
                        DeactivateWheelie();
                        DisengagePTO();
                        Chassis.BL.setPower(0.07);
                        Chassis.BR.setPower(0.07);
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.Disable = false;
                        Elevator.setTargetPosition(BAR2 + 50);
                        return Elevator.getCurrentPosition() >= BAR2;
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
                        if(pitch > -2){
                            Elevator.setTargetPosition(BAR2 - 300);
                            return true;
                        }
                        return false;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        EngagePTO();
                        Chassis.BL.setPower(0);
                        Chassis.BR.setPower(0);
                        return true;
                    }
                })
                .waitSeconds(0.08)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(-100);
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(Elevator.getCurrentPosition() < 10) {
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
        run.update();

        Elevator.update();
        Arm.update();

        Robot.telemetry.addData("tasks", run + "/14");
        Robot.telemetry.addData("pitch", pitch);
    }
}
