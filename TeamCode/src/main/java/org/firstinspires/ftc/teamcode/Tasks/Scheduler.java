package org.firstinspires.ftc.teamcode.Tasks;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.Mutex;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

public class Scheduler implements Cloneable {

//    public LinkedList<Task> tasks;
    public LinkedList<Task> tasks;
    private boolean mutex = false;
    private int thread = 0;

    public Scheduler(){
        tasks = new LinkedList<>();
    }

    public Scheduler addTask(Task t){
        tasks.addFirst(t);
        return this;
    }
    public Scheduler addTaskAsync(Task t){
        addTask(new Task() {
            @Override
            public boolean Run() {
                new Thread(() -> {
                    while(t.Run());
                }).start();
                return true;
            }
        }) ;
        return this;
    }
    public boolean done(){
        return tasks.isEmpty();
    }
    public void skip(){
        tasks.poll();
    }
    public Scheduler waitSeconds(double seconds){
        tasks.addFirst(new Wait(seconds));
        return this;
    }
    public void removeAllTasks(){
        tasks = new LinkedList<>();
    }
    @NonNull
    public String toString(){
        return Integer.toString(tasks.size());
    }
    public boolean DEBUG = false;
    public boolean Next = false;

    public void update(){
        if(mutex) return;
        mutex = true;
        if(done()) return;
        Task t = tasks.getLast();
        boolean result = t.Run();
        if(DEBUG){
            if(Next){
                tasks.removeLast();
                Next = false;
            }
            mutex = false;
            return;
        }
        if(result)
            tasks.removeLast();
        mutex = false;
    }

    public Scheduler lineTo(SparkFunOTOS.Pose2D pose){
        addTask(new LineTo(pose));
        return this;
    }
    public Scheduler lineToAsync(SparkFunOTOS.Pose2D pose){
        addTask(new Task() {
            @Override
            public boolean Run() {
//                Chassis.profiledFollow(pose);
                Chassis.profiledCurve(Collections.singletonList(pose));
                return true;
            }
        });
        return this;
    }

    public Scheduler lineToAsyncIfGoodStorageColor(SparkFunOTOS.Pose2D pose){
        addTask(new Task() {
            @Override
            public boolean Run() {
//                Chassis.profiledFollow(pose);
                if(!Storage.isStorageEmpty())
                    Chassis.profiledCurve(Collections.singletonList(pose));
                return true;
            }
        });
        return this;
    }

    public Scheduler splineToAsync(List<SparkFunOTOS.Pose2D> points){
        addTask(new Task() {
            @Override
            public boolean Run() {
                Chassis.profiledCurve(points);
                return true;
            }
        });
        return this;
    }
    public Scheduler lineToLinearHeadingAsync(SparkFunOTOS.Pose2D pose, boolean profile){
        addTask(new Task() {
            @Override
            public boolean Run() {
                Chassis.unprofiledLinearHeading(pose);
                return true;
            }
        });
        return this;
    }
    public Scheduler lineToLinearHeadingAsync(SparkFunOTOS.Pose2D pose){
        addTask(new Task() {
            @Override
            public boolean Run() {
                Chassis.profiledLinearHeading(pose);
                return true;
            }
        });
        return this;
    }
    public Scheduler waitForSync(){
        addTask(new Task() {
            @Override
            public boolean Run() {
                if(!Chassis.asyncFollow){
                    RobotLog.dd("headingError", Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) + "");
                    RobotLog.dd("translationalError", Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), Chassis.getTargetPosition()) + "");
                }
                return !Chassis.asyncFollow;
            }
        });
        return this;
    }
    public Scheduler waitForStill(){
        addTask(new Task() {
            @Override
            public boolean Run() {
                return Localizer.getVelocity().x < 10 && Localizer.getVelocity().y < 10 && Localizer.getVelocity().h < Math.toRadians(5);
            }
        });
        return this;
    }
    public Scheduler waitForTrajDone(double precent){
        addTask(new Task() {
            @Override
            public boolean Run() {
                if(Chassis.getPrecentageOfMotionDone() >= precent){
                    RobotLog.d("headingError", Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h));
                    RobotLog.d("translationalError", Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), Chassis.getTargetPosition()));
                }
                return Chassis.getPrecentageOfMotionDone() >= precent;
            }
        });
        return this;
    }

    @NonNull
    @Override
    public Scheduler clone() {
        try {
            Scheduler clone = (Scheduler) super.clone();
            // TODO: copy mutable state here, so the clone can't change the internals of the original
            clone.tasks = new LinkedList<>();
            clone.tasks.addAll(tasks);
            return clone;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }

    public void clear() {
        if(mutex) return;
        mutex = true;
        tasks.clear();
        mutex = false;
    }
}
