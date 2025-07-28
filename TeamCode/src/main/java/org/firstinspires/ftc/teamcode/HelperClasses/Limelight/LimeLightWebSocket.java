package org.firstinspires.ftc.teamcode.HelperClasses.Limelight;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.java_websocket.WebSocket;
import org.java_websocket.WebSocketFactory;
import org.java_websocket.WebSocketImpl;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import java.io.IOException;
import java.net.URI;

import javax.websocket.ClientEndpoint;
import javax.websocket.ContainerProvider;
import javax.websocket.DeploymentException;
import javax.websocket.OnMessage;
import javax.websocket.WebSocketContainer;

@ClientEndpoint
public class LimeLightWebSocket {
    private WebSocketContainer socket;

    @OnMessage
    public void onMessage(String s){
        Robot.telemetry.addLine(s);
    }

    public LimeLightWebSocket(String limelightIp) throws DeploymentException, IOException {
        limelightIp = "ws://" + limelightIp + ":5806";
        socket = ContainerProvider.getWebSocketContainer();
        try{
            socket.connectToServer(LimeLightWebSocket.class, URI.create(limelightIp));
        } catch (Exception e){
            throw e;
        }
    }
}
