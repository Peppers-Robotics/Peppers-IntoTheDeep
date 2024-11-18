package org.firstinspires.ftc.teamcode.HelperClasses;

import java.util.Comparator;




public abstract class Action implements Comparator<Action> {
    private static long timestamp = 0;
    private long thisTimestamp = 0;
    @Override
    public int compare(Action a, Action b){
        if(a.PriorityLevel - b.PriorityLevel == 0) return (int) (a.thisTimestamp - b.thisTimestamp);
        return a.PriorityLevel - b.PriorityLevel;
    }
    protected InternalClock currentTime = new InternalClock();
    public Action(){
        thisTimestamp = timestamp;
        timestamp ++;
    }
    public class InternalClock{
        private long ms = 0;
        public InternalClock() {
            ms = System.currentTimeMillis();
        }
        public double seconds(){
            return (System.currentTimeMillis() - ms) / 1000.f;
        }
        public double reset(){
            double r = seconds();
            ms = System.currentTimeMillis();
            return r;
        }
    }

    protected int PriorityLevel = 5;
    protected long msPasses = 0;

    public abstract void update();

    public void setPriorityLevel(int level){ PriorityLevel = level; }
    public int getPriorityLevel(){ return PriorityLevel; }

    public boolean Do(){
        if(passedCheckCondition()) return true;

        update();

        return false;
    }
    public abstract boolean passedCheckCondition();
}
