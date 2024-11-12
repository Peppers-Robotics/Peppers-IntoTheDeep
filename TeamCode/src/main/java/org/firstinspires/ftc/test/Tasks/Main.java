package org.firstinspires.ftc.test.Tasks;

import org.firstinspires.ftc.teamcode.HelperClasses.Action;
import org.firstinspires.ftc.teamcode.HelperClasses.Actioner;


public class Main {
    public static void main(String[] args){
        Actioner actioner = new Actioner();

        actioner.addAction(new RaiseElevator());
        actioner.addAction(new WaitTime(10));
        actioner.addAction(new ExtendArm());
        while (!actioner.isInIdle()){
            actioner.update();
        }
    }

    static class RaiseElevator extends Action {
        public double CurrentValue = 0;
        boolean o = false;
        @Override
        public void update() {
            setPriorityLevel(1);
            CurrentValue += 0.02;
            if(!o){
                System.out.println("elevator profile: " + CurrentValue);
                o = true;
            }
        }

        private boolean pass1 = false;
        @Override
        public boolean passedCheckCondition() {
            if(CurrentValue > 100 && !pass1) {currentTime.reset(); pass1 = true;}
            return pass1 && currentTime.seconds() >= 0.2; // deprecated, use WaitTime action
        }
    }
    static class ExtendArm extends Action{
        public double CurrentValue = 0;
        private boolean o = false;
        @Override
        public void update() {
            setPriorityLevel(1);
            CurrentValue += 0.01;
            if(!o) {
                System.out.println("arm profile: " + CurrentValue);
                o = true;
            }
        }

        @Override
        public boolean passedCheckCondition() {
            return CurrentValue >= 100;
        }
    }
    static class WaitTime extends Action{
        private boolean firstCall = false;
        @Override
        public void update() {
            setPriorityLevel(1);
            if(!firstCall) {
                firstCall = true;
                currentTime.reset();
                System.out.println("Waiting . . .");
            }
        }
        private double s = 0;
        public WaitTime(double seconds){
            s = seconds;
        }

        @Override
        public boolean passedCheckCondition() {
            if(currentTime.seconds() >= s) System.out.println("Done wating!");
            return currentTime.seconds() >= s;
        }
    }
}


