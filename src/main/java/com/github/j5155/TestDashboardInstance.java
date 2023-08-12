package com.github.j5155;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.RobotStatus;
import com.acmerobotics.dashboard.SendFun;
import com.acmerobotics.dashboard.SocketHandler;
import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;
import com.acmerobotics.dashboard.message.redux.ReceiveOpModeList;
import com.acmerobotics.dashboard.message.redux.ReceiveRobotStatus;

import java.io.IOException;
import java.util.List;
import java.util.Objects;

import fi.iki.elonen.NanoHTTPD;
import fi.iki.elonen.NanoWSD;

public class TestDashboardInstance {
    private static final TestDashboardInstance instance = new TestDashboardInstance();
    public DashboardCore core = new DashboardCore();
    private final NanoWSD server = new NanoWSD(8000) {
        @Override
        protected NanoWSD.WebSocket openWebSocket(NanoHTTPD.IHTTPSession handshake) {
            return new DashWebSocket(handshake);
        }
    };
    private class DashWebSocket extends NanoWSD.WebSocket implements SendFun {
        final SocketHandler sh = core.newSocket(this);
        public DashWebSocket(NanoHTTPD.IHTTPSession handshakeRequest) {
            super(handshakeRequest);
        }
        @Override
        public void send(Message message) {
            try {
                String messageStr = DashboardCore.GSON.toJson(message);
                send(messageStr);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
        @Override
        protected void onOpen() {
            sh.onOpen();
            send(new ReceiveOpModeList(List.of("RR-PREVIEW")));
        }
        @Override
        protected void onClose(NanoWSD.WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) {
            sh.onClose();
        }
        @Override
        protected void onMessage(NanoWSD.WebSocketFrame message) {
            String payload = message.getTextPayload();
            Message msg = DashboardCore.GSON.fromJson(payload, Message.class);
            if (sh.onMessage(msg)) {
                return;
            }
            if (Objects.requireNonNull(msg.getType()) == MessageType.GET_ROBOT_STATUS) {
                String opModeName;
                RobotStatus.OpModeStatus opModeStatus;
                opModeStatus = RobotStatus.OpModeStatus.STOPPED;
                opModeName = "RR-PREVIEW";

                send(new ReceiveRobotStatus(
                        new RobotStatus(core.enabled, true, opModeName, opModeStatus, "rr-preview-via-dash based on Roadrunner and FTC Dashboard by acmerobotics and rbrott. Modified by j5155 and team #12087 Capital City Dynamics.", "")
                ));
            } else {
                System.out.println(msg.getType());
            }
        }
        @Override
        protected void onPong(NanoWSD.WebSocketFrame pong) {}
        @Override
        protected void onException(IOException exception) {}
    }
    public static TestDashboardInstance getInstance() {
        return instance;
    }
    public void start() {
        System.out.println("Starting Dashboard instance");
        core.enabled = true;
        try {
            server.start();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
