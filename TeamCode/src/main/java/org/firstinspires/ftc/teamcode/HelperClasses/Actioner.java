package org.firstinspires.ftc.teamcode.HelperClasses;

import java.util.Comparator;
import java.util.PriorityQueue;

class Nothing extends Action{

    @Override
    public void update() {

    }

    @Override
    public boolean passedCheckCondition() {
        return true;
    }
}

public class Actioner {
    private PriorityQueue<Action> queue = new PriorityQueue<>(new Nothing());
    public Actioner(){
    }

    public void addAction(Action a){
        queue.add(a);
    }
    public boolean isInIdle(){
        return queue.isEmpty();
    }

    public void update(){
        Action lastAction;
        if(queue.isEmpty()){
            queue.add(new Nothing());
        }
        lastAction = queue.peek();

        lastAction.update();
        if(lastAction.passedCheckCondition()) queue.remove();
    }
}
