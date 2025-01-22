package org.firstinspires.ftc.teamcode.Tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

public class Scheduler {
    public static class Wait extends Task {
        private ElapsedTime time;
        private double s;
        public Wait(double sec){
            s = sec;
            time = new ElapsedTime();
        }
        @Override
        public boolean Run() {
            return time.seconds() >= s;
        }
    }
    public static LinkedList<Task> tasks;
    public Scheduler(){
        tasks = new LinkedList<>();
    }

    public Scheduler addTask(Task t){
        tasks.add(t);
        return this;
    }
    public boolean done(){
        return tasks.isEmpty();
    }
    public void skip(){
        tasks.poll();
    }
    public Scheduler waitSeconds(double seconds){
        tasks.addLast(new Wait(seconds));
        return this;
    }
    public void removeAllTasks(){
        tasks = new LinkedList<>();
    }
    public String toString(){
        return Integer.toString(tasks.size());
    }
    public boolean DEBUG = false;
    public boolean Next = false;
    public void update(){
        if(done()) return;
        boolean result = tasks.getFirst().Run();
        if(DEBUG){
            if(Next){
                tasks.removeFirst();
                Next = false;
            }
            return;
        }
        if(result)
            tasks.removeFirst();
    }

}
