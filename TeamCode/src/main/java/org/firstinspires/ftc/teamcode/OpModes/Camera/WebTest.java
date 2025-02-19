package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Scanner;

@TeleOp
public class WebTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.enable();
        waitForStart();

        while (opModeIsActive()){
            try {
                String response = getString();

                System.out.println("Response: " + response);
            } catch (Exception e) {
                System.out.println("❌ Error: " + e.getMessage());
            }
        }
    }

    private static String getString() throws IOException {
        URL url = new URL("http://172.29.0.1:5800/stream");
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("GET");
        connection.setRequestProperty("User-Agent", "Mozilla/5.0");
        connection.connect();

        InputStream inputStream = connection.getInputStream();
        Scanner scanner = new Scanner(inputStream).useDelimiter("\\A");
        String response = scanner.hasNext() ? scanner.next() : "";
        return response;
    }
}