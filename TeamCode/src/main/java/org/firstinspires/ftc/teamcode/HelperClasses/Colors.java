package org.firstinspires.ftc.teamcode.HelperClasses;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Colors {
    public static double ColorMaxDifference = 120;
    public static class Color{
        public double r, g, b;
        public Color(double r, double g, double b){
            this.r = r;
            this.g = g;
            this.b = b;
        }

    }
    public enum ColorType {
        BLUE(new Color(50, 110, 255)),

        RED(new Color(255, 116, 50)),

        YELLOW(new Color(200, 255, 50)),

        NONE(new Color(160, 255, 180));

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


    public static ColorType getColorFromRGB(Color input){

//        if(input.r > input.g && input.r > input.b) return ColorType.RED;
       /* if(Math.max(input.r, Math.max(input.g, input.b)) == input.b) return ColorType.BLUE;
        if(Math.max(input.r, Math.max(input.g, input.b)) == input.b) return ColorType.RED;
        if(Math.max(input.r, Math.max(input.g, input.b)) == input.g && Math.abs(input.r - input.g) >= 30) return ColorType.RED;
        return ColorType.NONE;*/

        ColorType detected = ColorType.NONE;
        double mini = 1e9;
        for(ColorType ct : ColorType.values()){
            if(ct == ColorType.NONE) continue;
            double dist = getColorDistance(input, ct.getColor());
            if(mini > dist){
                detected = ct;
                mini = dist;
            }
        }
        return detected;
    }

}
