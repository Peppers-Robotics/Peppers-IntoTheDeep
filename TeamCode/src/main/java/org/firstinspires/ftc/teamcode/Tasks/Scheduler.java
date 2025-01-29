package org.firstinspires.ftc.teamcode.Tasks;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;

import java.util.LinkedList;
import java.util.Objects;

public class Scheduler implements Cloneable {

    public LinkedList<Task> tasks;
    public Scheduler(){
        tasks = new LinkedList<>();
    }

    public Scheduler addTask(Task t){
        tasks.addFirst(t);
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
        if(done()) return;
        Task t = tasks.getLast();
        boolean result = t.Run();
        if(DEBUG){
            if(Next){
                tasks.removeLast();
                Next = false;
            }
            return;
        }
        if(result)
            tasks.removeLast();
    }

    public Scheduler lineTo(SparkFunOTOS.Pose2D pose){
        addTask(new LineTo(pose));
        return this;
    }
    public Scheduler lineToAsync(SparkFunOTOS.Pose2D pose){
        addTask(new Task() {
            @Override
            public boolean Run() {
                Chassis.profiledFollow(pose);
                return true;
            }
        });
        return this;
    }
    public Scheduler waitForSync(){
        addTask(new Task() {
            @Override
            public boolean Run() {
                return Chassis.asyncFollow;
            }
        });
        return this;
    }
    public Scheduler waitForStill(){
        addTask(new Task() {
            @Override
            public boolean Run() {
                return Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), Chassis.getTargetPosition()) < 0.5;
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
}
