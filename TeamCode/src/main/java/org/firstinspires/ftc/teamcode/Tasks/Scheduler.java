package org.firstinspires.ftc.teamcode.Tasks;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

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
        else
            if(t.needsReset) t.reset();
    }

    @Override
    public Scheduler clone() {
        try {
            Scheduler clone = (Scheduler) super.clone();
            // TODO: copy mutable state here, so the clone can't change the internals of the original
            clone.tasks = (LinkedList<Task>) tasks.clone();
            return clone;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
