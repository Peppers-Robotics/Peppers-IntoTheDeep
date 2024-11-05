package tests;

import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.OutTake.Arm;

import java.util.Scanner;

import tests.HardwareEmulators.ServoE;

public class ArmProfileTest {
    public static void main(String[] args){
        Scanner inp = new Scanner(System.in);

        Arm.servo1 = new ServoPlus(new ServoE());
        Arm.servo2 = new ServoPlus(new ServoE());
        while(true) {
            int arm, pivot;
            arm = inp.nextInt();
            pivot = inp.nextInt();
            Arm.setArmAngle(arm);
            Arm.setPivotAngle(pivot);

            while(!Arm.motionCompleted()){
                System.out.print(Arm.servo1.getAngle());
                System.out.print(" ");
                System.out.println(Arm.servo2.getAngle());
            }

        }


    }
}