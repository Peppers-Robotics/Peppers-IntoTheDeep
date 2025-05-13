package org.firstinspires.ftc.teamcode.OpModes;

import android.media.MediaCodec;
import android.media.MediaExtractor;
import android.media.MediaFormat;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.nio.ByteBuffer;
import java.security.cert.CertPathValidatorException;
import java.util.ArrayList;

@TeleOp
@Config
public class Sound extends LinearOpMode {
    public static double sampleRate = 44100, outputRate = 44100, volume = 0.7;
    public static byte[] decodeMp3ToPcmBytes(String filePath) {
        ArrayList<Byte> pcmBytes = new ArrayList<>();

        try {
            File mp3File = new File(filePath);
            if (!mp3File.exists()) {
                throw new IllegalArgumentException("File does not exist: " + filePath);
            }

            MediaExtractor extractor = new MediaExtractor();
            extractor.setDataSource(filePath);

            int audioTrackIndex = -1;
            for (int i = 0; i < extractor.getTrackCount(); i++) {
                MediaFormat format = extractor.getTrackFormat(i);
                String mime = format.getString(MediaFormat.KEY_MIME);
                if (mime != null && mime.startsWith("audio/")) {
                    audioTrackIndex = i;
                    break;
                }
            }

            if (audioTrackIndex == -1) {
                throw new RuntimeException("No audio track found in " + filePath);
            }

            extractor.selectTrack(audioTrackIndex);
            MediaFormat format = extractor.getTrackFormat(audioTrackIndex);
            String mime = format.getString(MediaFormat.KEY_MIME);

            MediaCodec codec = MediaCodec.createDecoderByType(mime);
            codec.configure(format, null, null, 0);
            codec.start();

            boolean isEOS = false;
            MediaCodec.BufferInfo info = new MediaCodec.BufferInfo();

            while (!isEOS) {
                int inputIndex = codec.dequeueInputBuffer(10000);
                if (inputIndex >= 0) {
                    ByteBuffer inputBuffer = codec.getInputBuffer(inputIndex);
                    int sampleSize = extractor.readSampleData(inputBuffer, 0);
                    if (sampleSize < 0) {
                        codec.queueInputBuffer(inputIndex, 0, 0, 0L, MediaCodec.BUFFER_FLAG_END_OF_STREAM);
                        isEOS = true;
                    } else {
                        long presentationTimeUs = extractor.getSampleTime();
                        codec.queueInputBuffer(inputIndex, 0, sampleSize, presentationTimeUs, 0);
                        extractor.advance();
                    }
                }

                int outputIndex = codec.dequeueOutputBuffer(info, 10000);
                while (outputIndex >= 0) {
                    ByteBuffer outputBuffer = codec.getOutputBuffer(outputIndex);
                    byte[] buffer = new byte[info.size];
                    outputBuffer.get(buffer);
                    for (byte b : buffer) {
                        pcmBytes.add(b);
                    }
                    codec.releaseOutputBuffer(outputIndex, false);
                    if ((info.flags & MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) {
                        isEOS = true;
                        break;
                    }
                    outputIndex = codec.dequeueOutputBuffer(info, 0);
                }
            }

            codec.stop();
            codec.release();
            extractor.release();

        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }

        // Convert List<Byte> to byte[]
        byte[] result = new byte[pcmBytes.size()];
        for (int i = 0; i < pcmBytes.size(); i++) {
            result[i] = pcmBytes.get(i);
        }

        return result;
    }

    public static double block = 140, unblock = 280;

    @Override
    public void runOpMode() throws InterruptedException {
//        DcMotor speaker = new DcMotorImpl(hardwareMap.get(DcMotorController.class, "Control Hub"), 0);
        DcMotorController c = hardwareMap.get(DcMotorController.class, "Control Hub"),
                          e = hardwareMap.get(DcMotorController.class, "Expansion Hub 2");
        ElapsedTime t = new ElapsedTime();

        FtcDashboard.getInstance().getTelemetry().addLine("Parsing song . . .");
        FtcDashboard.getInstance().getTelemetry().update();
        byte[] sample = decodeMp3ToPcmBytes("/sdcard/FIRST/song.mp3");
        FtcDashboard.getInstance().getTelemetry().addLine("Song ready!");
        FtcDashboard.getInstance().getTelemetry().update();
        int step = (int) (sampleRate / outputRate);

        waitForStart();
        double norm = 0;
        t.reset();
        for(int i = 0; i < sample.length - 1 && opModeIsActive(); i += step * 2){
            int low = sample[i] & 0xff;
            int high = sample[i + 1];
            short s = (short) ((high << 8) | low);

            norm = s / 32768.f * volume;
            c.setMotorPower(0, norm);
            c.setMotorPower(1, norm);
            e.setMotorPower(2, norm);
            e.setMotorPower(3, norm);
        }
        norm = 0;
        c.setMotorPower(0, norm);
        c.setMotorPower(1, norm);
        e.setMotorPower(2, norm);
        e.setMotorPower(3, norm);
    }
}
