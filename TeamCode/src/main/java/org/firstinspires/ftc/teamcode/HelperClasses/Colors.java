package org.firstinspires.ftc.teamcode.HelperClasses;


import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;

@Config
public class Colors {
    public static double ColorMaxDifference = 120;
    public static class Color{
        public double r, g, b, d;
        public Color(double r, double g, double b, double d){
            this.r = r;
            this.g = g;
            this.b = b;
            this.d = d;
        }

    }
    public enum ColorType {
        BLUE(new Color(0.4, 0.7, 1.3, 2)),
        BLUE1(new Color(0.2, 0.4, 0.4, 3.4)),

        RED(new Color(1.1, 0.8, 0.3, 2)),
        RED1(new Color(0.5, 0.4, 0.2, 3)),
        RED2(new Color(0.8, 0.6, 0.3, 2.6)),

        YELLOW(new Color(2.3, 3.2, 0.7, 1.5)),
        YELLOW1(new Color(0.6, 0.8, 0.2, 3.1)),
        YELLOW2(new Color(1, 1.41, 0.47, 3)),

        NONE(new Color(0.2, 0.3, 0.2, 4));

        private final Color color;

        ColorType(Color c) {
            this.color = c;
        }

        public Color getColor(){ return color; }

    }
    public static double getColorDistance(Color c1, Color c2) {
        double rDiff = c1.r - c2.r;
        double gDiff = c1.g - c2.g;
        double bDiff = c1.b - c2.b;
        return Math.sqrt(rDiff * rDiff + gDiff * gDiff + bDiff * bDiff);
    }

    public static ColorType getColorFromRGB(Color input) {
        double redDist1 = Math.min(getColorDistance(input, ColorType.RED.getColor()), getColorDistance(input, ColorType.RED1.getColor()));
        double redDist = Math.min(getColorDistance(input, ColorType.RED2.getColor()), redDist1);
        double blueDist = Math.min(getColorDistance(input, ColorType.BLUE.getColor()), getColorDistance(input, ColorType.BLUE1.getColor()));
        double yellowDist1 = Math.min(getColorDistance(input, ColorType.YELLOW.getColor()), getColorDistance(input, ColorType.YELLOW1.getColor()));
        double yellowDist = Math.min(getColorDistance(input, ColorType.YELLOW2.getColor()), yellowDist1);
        if(redDist < blueDist && redDist < yellowDist) return ColorType.RED;
        if(blueDist < redDist && blueDist < yellowDist) return ColorType.BLUE;
        if(yellowDist < redDist && yellowDist < blueDist) return ColorType.YELLOW;
        return ColorType.NONE;
    }

}
