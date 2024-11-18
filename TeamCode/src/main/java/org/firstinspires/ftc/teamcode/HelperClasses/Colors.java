package org.firstinspires.ftc.teamcode.HelperClasses;



public class Colors {
    public static class Color{
        public int r, g, b;
        public Color(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
    public enum ColorType {
        BLUE(new Color(0, 0, 255)),
//        GREEN(new Color(0, 255, 0)),
        RED(new Color(255, 0, 0)),
        YELLOW(new Color(255, 255, 0)),
        BLACK(new Color(0, 0, 0)),
        WHITE(new Color(255, 255, 255)),
//        CYAN(new Color(0, 255, 255)),
//        PURPLE(new Color(255, 0, 255)),
//        ORANGE(new Color(255, 128, 0)),
        NONE(new Color(0, 0, 0));
        private final Color color;

        ColorType(Color c) {
            this.color = c;
        }

        public Color getColor(){ return color; }

    }
    private static double getColorDistance(Color c1, Color c2) {
        int rDiff = c1.r - c2.r;
        int gDiff = c1.g - c2.g;
        int bDiff = c1.b - c2.b;
        return Math.sqrt(rDiff * rDiff + gDiff * gDiff + bDiff * bDiff);
    }


    public static ColorType getColorFromRGB(Color input){
        ColorType detected = ColorType.NONE;
        double minDist = Double.MAX_VALUE;
        for(ColorType ct : ColorType.values()){
            if(ct == ColorType.NONE) continue;
            double dist = getColorDistance(input, ct.getColor());
            if(dist < minDist){
                detected = ct;
                minDist = dist;
            }
        }
        return detected;
    }

}
