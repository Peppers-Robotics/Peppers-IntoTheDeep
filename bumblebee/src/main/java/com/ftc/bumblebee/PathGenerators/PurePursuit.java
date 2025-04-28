package com.ftc.bumblebee.PathGenerators;

import com.ftc.bumblebee.Localizers.Localizer;
import com.ftc.bumblebee.Localizers.Pose2d;

import java.util.Vector;

@SuppressWarnings({"unused"})
public class PurePursuit {
    private double Radius;
    private Vector<Pose2d> checkPoints;
    private Pose2d lastHeadingPose;
    private boolean isOnLastSegment = false;
    public PurePursuit(Vector<Pose2d> checkPoints, double radius){
        this.Radius = radius;
        this.checkPoints = checkPoints;
        lastHeadingPose = checkPoints.get(0);
    }
    private Pose2d getFirstDegreeCoeficients(Pose2d p1, Pose2d p2){
        if(p1.y == p2.y) return new Pose2d(0, 0);
        if(p1.x == p2.x) return new Pose2d(0, 0);
        double a = (p1.y - p2.y) / (p1.x - p2.x);
        double b = p1.y - a * p1.x;

        return new Pose2d(a, b);
    }
    private static double getAngleFromTwoPoints(Pose2d a, Pose2d b){
        return Math.atan2(a.x - b.x, a.y - b.y);
    }

    public static double Distance(Pose2d A, Pose2d B){
        return Math.sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
    }
    public Pose2d getNextHeadingPoint(Pose2d currentPose){

        if(Distance(lastHeadingPose, checkPoints.lastElement()) <= Radius && isOnLastSegment) return checkPoints.lastElement();

        for(int i = 0; i < checkPoints.size() - 1; i++){

            double a = getFirstDegreeCoeficients(checkPoints.elementAt(i), checkPoints.elementAt(i + 1)).x;
            double b = getFirstDegreeCoeficients(checkPoints.elementAt(i), checkPoints.elementAt(i + 1)).y;
            Pose2d thisPos = new Pose2d(currentPose.x, currentPose.y, currentPose.h);
            Pose2d n1 = new Pose2d(0, 0);
            Pose2d n2 = new Pose2d(0, 0);

            double alpha, beta, gamma, delta;

            if(checkPoints.elementAt(i).x == checkPoints.elementAt(i+1).x){
                double x = checkPoints.elementAt(i).x;

                delta = Radius * Radius - (x - currentPose.x) * (x - currentPose.x);
                if(delta < 0) continue;

                n1 = new Pose2d(x, Math.sqrt(delta) + currentPose.y);
                n2 = new Pose2d(x, - Math.sqrt(delta) + currentPose.y);

            } else if(checkPoints.elementAt(i).y == checkPoints.elementAt(i+1).y){
                double y = checkPoints.elementAt(i).y;

                delta = Radius * Radius - (y - currentPose.y) * (y - currentPose.y);
                if(delta < 0) continue;

                n1 = new Pose2d(Math.sqrt(delta) + currentPose.x, y);
                n2 = new Pose2d(-Math.sqrt(delta) + currentPose.x, y);

            } else {
                alpha = 1 + a * a;
                beta = -2 * thisPos.x + 2 * a * (b - thisPos.y);
                gamma = thisPos.x * thisPos.x + b * b - 2 * b * thisPos.y + thisPos.y * thisPos.y - Radius * Radius;

                delta = (beta * beta - 4 * alpha * gamma);

                if(delta < 0) continue;

                double x = (-beta + Math.sqrt(delta)) / (2 * alpha);
                double y = a * x + b;
                n1 = new Pose2d(x, y);
                x = (-beta - Math.sqrt(delta)) / (2 * alpha);
                y = a * x + b;
                n2 = new Pose2d(x, y);
            }

            if(i == checkPoints.size() - 2) {
                isOnLastSegment = true;
            }

            if(Distance(n1, checkPoints.elementAt(i + 1)) < Distance(n2, checkPoints.elementAt(i + 1))){
                lastHeadingPose = new Pose2d(n1.x, n1.y, -getAngleFromTwoPoints(currentPose, n1));
            } else {
                lastHeadingPose = new Pose2d(n2.x, n2.y, -getAngleFromTwoPoints(currentPose, n2));
            }

        }

        return lastHeadingPose;
    }
}
