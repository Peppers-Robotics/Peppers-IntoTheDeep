package org.firstinspires.ftc.teamcode.HelperClasses.Devices;

import com.qualcomm.robotcore.hardware.Gamepad;

public class AutoGamepad {

    public Gamepad wasPressed, wasReleased;
    public float left_stick_x, left_stick_y,
                 right_stick_x, right_stick_y,
                 right_trigger, left_trigger;
    public Gamepad lastState, currentState;
    public Gamepad gamepad; // used for applying rumbleEffects or ledEffects

    private void updatePressed(){
        wasPressed.a = !lastState.a && currentState.a;
        wasPressed.b = !lastState.b && currentState.b;
        wasPressed.x = !lastState.x && currentState.x;
        wasPressed.y = !lastState.y && currentState.y;
        wasPressed.square = !lastState.square && currentState.square;
        wasPressed.circle = !lastState.circle && currentState.circle;
        wasPressed.triangle = !lastState.triangle && currentState.triangle;
        wasPressed.cross = !lastState.cross && currentState.cross;

        wasPressed.dpad_up = !lastState.dpad_up && currentState.dpad_up;
        wasPressed.dpad_down = !lastState.dpad_down && currentState.dpad_down;
        wasPressed.dpad_left = !lastState.dpad_left && currentState.dpad_left;
        wasPressed.dpad_right = !lastState.dpad_right && currentState.dpad_right;

        wasPressed.left_bumper = !lastState.left_bumper && currentState.left_bumper;
        wasPressed.right_bumper = !lastState.right_bumper && currentState.right_bumper;

        wasPressed.left_stick_button = !lastState.left_stick_button && currentState.left_stick_button;
        wasPressed.right_stick_button = !lastState.right_stick_button && currentState.right_stick_button;

        wasPressed.options = !lastState.options && currentState.options;
        wasPressed.start = !lastState.start && currentState.start;
    }

    private void updateReleased(){
        wasReleased.a = lastState.a && !currentState.a;
        wasReleased.b = lastState.b && !currentState.b;
        wasReleased.x = lastState.x && !currentState.x;
        wasReleased.y = lastState.y && !currentState.y;
        wasReleased.square = lastState.square && !currentState.square;
        wasReleased.circle = lastState.circle && !currentState.circle;
        wasReleased.triangle = lastState.triangle && !currentState.triangle;

        wasReleased.dpad_up = lastState.dpad_up && !currentState.dpad_up;
        wasReleased.dpad_down = lastState.dpad_down && !currentState.dpad_down;
        wasReleased.dpad_left = lastState.dpad_left && !currentState.dpad_left;
        wasReleased.dpad_right = lastState.dpad_right && !currentState.dpad_right;

        wasReleased.left_bumper = lastState.left_bumper && !currentState.left_bumper;
        wasReleased.right_bumper = lastState.right_bumper && !currentState.right_bumper;

        wasReleased.left_stick_button = lastState.left_stick_button && !currentState.left_stick_button;
        wasReleased.right_stick_button = lastState.right_stick_button && !currentState.right_stick_button;

        wasReleased.options = lastState.options && !currentState.options;
        wasReleased.start = lastState.start && !currentState.start;
    }

    public AutoGamepad(Gamepad gp){
        currentState = gp;
        gamepad = gp;
        lastState = new Gamepad();
        wasReleased = new Gamepad();
        wasPressed = new Gamepad();
    }

    public void update(){
        updatePressed();
        updateReleased();

        left_stick_x = currentState.left_stick_x;
        left_stick_y = currentState.left_stick_y;

        right_stick_x = currentState.right_stick_x;
        right_stick_y = currentState.right_stick_y;

        left_trigger = currentState.left_trigger;
        right_trigger = currentState.right_trigger;

        lastState.copy(currentState);
    }

}