package org.usfirst.frc.team1806.robot.Vision;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Vision.Message.HeartbeatMessage;
import org.usfirst.frc.team1806.robot.Vision.Message.TargetsMessage;
import org.usfirst.frc.team1806.robot.Vision.Message.UnknownTypeMessage;
import org.usfirst.frc.team1806.robot.Vision.Message.VisionMessage;
import org.usfirst.frc.team1806.robot.util.CrashTrackingRunnable;
import org.usfirst.frc.team1806.robot.util.Target;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collections;

public class VisionServer extends CrashTrackingRunnable {

    private static VisionServer s_instance = null;
    private ServerSocket m_server_socket;
    private boolean m_running = true;
    private int m_port;
    double lastMessageReceivedTime = 0;
    private boolean m_use_java_time = false;
    private Double targetsTimestamp;
    private ArrayList<Target> targets;

    private ArrayList<ServerThread> serverThreads = new ArrayList<>();
    private volatile boolean mWantsAppRestart = false;

    public static VisionServer getInstance() {
        if (s_instance == null) {
            s_instance = new VisionServer(Constants.kCoprocessorPort);
        }
        return s_instance;
    }

    private boolean mIsConnect = false;

    public boolean isConnected() {
        return mIsConnect;
    }

    public void requestAppRestart() {
        mWantsAppRestart = true;
    }

    protected class ServerThread extends CrashTrackingRunnable {
        private Socket m_socket;

        public ServerThread(Socket socket) {
            m_socket = socket;
        }

        public void send(VisionMessage message) {
            String toSend = message.toJson() + "\n";
            if (m_socket != null && m_socket.isConnected()) {
                try {
                    OutputStream os = m_socket.getOutputStream();
                    os.write(toSend.getBytes());
                } catch (IOException e) {
                    System.err.println("VisionServer: Could not send data to socket");
                }
            }
        }

        public void handleMessage(VisionMessage message, double timestamp) {
            if ("targets".equals(message.getType())) {
                try {
                    JsonParser jsonParser = new JsonParser();
                    JsonElement messgageElement = jsonParser.parse(message.toJson());
                    JsonArray targetsArray = messgageElement.getAsJsonObject().getAsJsonObject("message").getAsJsonArray("targets");
                    ArrayList<Target> newTargetsArray = new ArrayList<>();
                    for (JsonElement targetElement : targetsArray) {
                        newTargetsArray.add(new Target(targetElement.getAsJsonObject()));
                    }
                    double newTimestamp = messgageElement.getAsJsonObject().getAsJsonObject("message").getAsJsonPrimitive("timestamp").getAsDouble();
                    synchronized (targets){
                        synchronized (targetsTimestamp){
                            targets = newTargetsArray;
                            targetsTimestamp = newTimestamp;
                        }
                    }
                }
                catch(Exception e){
                    System.out.println("Could not parse targets message." + e.getMessage());
                    e.printStackTrace();
                }
            }
            if ("heartbeat".equals(message.getType())) {
                HeartbeatMessage hb = new HeartbeatMessage(getTimestamp());
                send(hb);
            }
        }

        public boolean isAlive() {
            return m_socket != null && m_socket.isConnected() && !m_socket.isClosed();
        }

        @Override
        public void runCrashTracked() {
            if (m_socket == null) {
                return;
            }
            try {
                InputStream is = m_socket.getInputStream();
                byte[] buffer = new byte[2048];
                int read;
                while (m_socket.isConnected() && (read = is.read(buffer)) != -1) {
                    double timestamp = getTimestamp();
                    lastMessageReceivedTime = timestamp;
                    String messageRaw = new String(buffer, 0, read);
                    String[] messages = messageRaw.split("\n");
                    for (String message : messages) {
                        UnknownTypeMessage parsedMessage = new UnknownTypeMessage(message);
                        if (parsedMessage.isValid()) {
                            handleMessage(parsedMessage, timestamp);
                        }
                    }
                }
                System.out.println("Socket disconnected");
            } catch (IOException e) {
                System.err.println("Could not talk to socket");
            }
            if (m_socket != null) {
                try {
                    m_socket.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    /**
     * Instantializes the VisionServer and connects to ADB via the specified
     * port.
     *
     * @param port network port
     */
    private VisionServer(int port) {
        targets = new ArrayList<>();
        targetsTimestamp = 0.0;
        try {
            m_port = port;

            System.out.println("Starting vision server socket");
            m_server_socket = new ServerSocket(port);
            try {
                String useJavaTime = System.getenv("USE_JAVA_TIME");
                m_use_java_time = "true".equals(useJavaTime);
            } catch (NullPointerException e) {
                m_use_java_time = false;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        new Thread(this).start();
        new Thread(new AppMaintainanceThread()).start();
    }


    @Override
    public void runCrashTracked() {
        while (m_running) {
            try {
                Socket p = m_server_socket.accept();
                System.out.println("Starting server socket");
                ServerThread s = new ServerThread(p);
                new Thread(s).start();
                serverThreads.add(s);
            } catch (IOException e) {
                System.err.println("Issue accepting socket connection!");
            } finally {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private class AppMaintainanceThread extends CrashTrackingRunnable {
        @Override
        public void runCrashTracked() {
            while (true) {
                if (getTimestamp() - lastMessageReceivedTime > .1) {
                    // camera disconnected
                    mIsConnect = false;
                } else {
                    mIsConnect = true;
                }
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private double getTimestamp() {
            return Timer.getFPGATimestamp();
    }

    public ArrayList<Target> getTargets(){
        synchronized (targets){
            return targets;
        }
    }

    public double getTargetsTimestamp(){
        synchronized (targetsTimestamp){
            return targetsTimestamp;
        }
    }
}
