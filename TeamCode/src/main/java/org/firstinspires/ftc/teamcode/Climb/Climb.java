package org.firstinspires.ftc.teamcode.Climb;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

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
    public static ServoPlus W1, W2, PTO1, PTO2;
    public static double BAR1 = 350, BAR2 = 850;
    public static double EngagePTO1 = 10, EngagePTO2 = 20, DisengagePTO1 = 20, DisengagePTO2 = 100,
                         EngageWheelie1 = 20, DisengageWheelie1 = 100, EngageWheelie2 = 10, DisengageWheelie2 = 0, climbArmIntertia = 310;

    public static void EngagePTO(){
        PTO1.setAngle(EngagePTO1);
        PTO2.setAngle(EngagePTO2);
    }
    public static void DisengagePTO(){
        PTO1.setAngle(DisengagePTO1);
        PTO2.setAngle(DisengagePTO2);
    }
    public static void ActivateWheelie(){
        W1.setAngle(EngageWheelie1);
        W1.setAngle(EngageWheelie2);
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
                        Extendo.motor.setPower(-0.4);
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(BAR1);
                        return Elevator.ReachedTargetPosition();
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
                .waitSeconds(0.2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(-20);
                        return Elevator.getCurrentPosition() < 4 || (Elevator.getCurrentPosition() < 100 && Math.abs(Elevator.motor.getVelocity()) < 1);
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(0);
                        DeactivateWheelie();
                        DisengagePTO();
                        Chassis.BL.setPower(0);
                        Chassis.BR.setPower(0);
                        return true;
                    }
                })
                .waitSeconds(0.2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(BAR2);
                        return Elevator.ReachedTargetPosition();
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        EngagePTO();
                        Chassis.BL.setPower(0);
                        Chassis.BR.setPower(0);
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(climbArmIntertia);
                        return Arm.motionCompleted();
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(Robot.imu.getRobotYawPitchRollAngles().getPitch() < -83){
                            Elevator.setTargetPosition(-50);
                            return true;
                        }
                        return false;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(Elevator.getCurrentPosition() < 20) {
                            Elevator.setTargetPosition(0);
                            return true;
                        }
                        return false;
                    }
                })

        ;
    }
    public static void Update(){
        run.update();
        Elevator.update();
        Arm.update();
        Robot.telemetry.addData("tasks", run + "/14");
    }
}
