package org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class LimeLightStream implements CameraStreamSource {
    private VideoCapture camera;
    private Mat frame;
    private Lock frameLock;
    private volatile boolean running = true; // Ensures thread safety
    private Bitmap latestBitmap;

    public LimeLightStream(String ipAddr, int Width, int Height) {
        this.camera = new VideoCapture(0);
        camera.open("http://172.28.0.1:5800/stream.mjpg");
        this.frame = new Mat();
        this.frameLock = new ReentrantLock();
        this.latestBitmap = Bitmap.createBitmap(Width, Height, Bitmap.Config.ARGB_8888);

        if (!camera.isOpened()) {
            throw new RuntimeException("Error: Could not open camera " + ipAddr);
        }

        startCaptureThread();
    }

    private void startCaptureThread() {
        new Thread(() -> {
            while (running && camera.isOpened()) {
                Mat tempFrame = new Mat();
                if (camera.read(tempFrame)) {
                    frameLock.lock();
                    try {
                        Imgproc.cvtColor(tempFrame, frame, Imgproc.COLOR_BGR2RGB);
                        Utils.matToBitmap(frame, latestBitmap); // Convert to Bitmap
                    } finally {
                        frameLock.unlock();
                    }
                }
            }
        }).start();
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        if (continuation != null) {
            continuation.dispatch(bitmapConsumer -> {
                frameLock.lock();
                try {
                    if (!frame.empty() && latestBitmap != null) {
                        bitmapConsumer.accept(latestBitmap);
                    }
                } finally {
                    frameLock.unlock();
                }
            });
        }
    }

    public void close() {
        running = false;
        camera.release();
    }
}
