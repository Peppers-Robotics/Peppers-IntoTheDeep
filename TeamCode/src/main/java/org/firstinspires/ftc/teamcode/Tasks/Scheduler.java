package org.firstinspires.ftc.teamcode.Tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;

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
    public static Queue<Task> tasks;
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
        tasks.add(new Wait(seconds));
        return this;
    }
    public void update(){
        if(tasks.peek() == null) return;
        boolean result = tasks.peek().Run();
        if(result)
            tasks.poll();
    }

}
