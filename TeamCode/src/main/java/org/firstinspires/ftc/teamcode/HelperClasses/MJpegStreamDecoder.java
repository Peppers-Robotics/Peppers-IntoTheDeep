package org.firstinspires.ftc.teamcode.HelperClasses;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.StrictMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Scanner;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class MJpegStreamDecoder implements CameraStreamSource {
    private final String streamUrl;
    private volatile boolean running = true;
    private final Lock frameLock = new ReentrantLock();
    private Bitmap latestBitmap;

    public MJpegStreamDecoder(String streamUrl) {
        this.streamUrl = streamUrl;
        startStreamThread();
    }

    private void startStreamThread() {
        new Thread(() -> {
            while (running) {
                try {
                    Bitmap frame = null;
                        while (running) {
                            Scanner scanner = getScanner();
                            byte data[] = (scanner.hasNext() ? scanner.next() : "").getBytes();
                            frame = BitmapFactory.decodeByteArray(data, 0, data.length);
                            if (frame != null) {
                                frameLock.lock();
                                try {
                                    latestBitmap = frame;
                                    frame.recycle();
                                } finally {
                                    frameLock.unlock();
                                }
                            }
                            scanner.remove();
                        }
                } catch (Exception e) {
                    System.err.println("Error reading MJPEG stream: " + e.getMessage());
                    try { Thread.sleep(2000); } catch (InterruptedException ignored) {} // Retry after 2 sec
                }
            }
        }).start();
    }

    private static Scanner getScanner() throws IOException {
        URL url = new URL("http://172.29.0.1:5800/stream.mjpg");
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("GET");
        connection.setRequestProperty("User-Agent", "Mozilla/5.0");
        connection.connect();
        InputStream inputStream = new BufferedInputStream(connection.getInputStream());

        Scanner scanner = new Scanner(inputStream).useDelimiter("\\A");
        return scanner;
    }

    private Bitmap decodeMJpegFrame(InputStream inputStream) {
        try {
            return BitmapFactory.decodeStream(inputStream);
        } catch (Exception e) {
            return null;
        }
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        if (continuation != null) {
            continuation.dispatch(bitmapConsumer -> {
                frameLock.lock();
                try {
                    if (latestBitmap != null) {
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
    }
}
