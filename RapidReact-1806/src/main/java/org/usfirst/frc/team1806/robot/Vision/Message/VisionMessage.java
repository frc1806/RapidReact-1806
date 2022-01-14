package org.usfirst.frc.team1806.robot.Vision.Message;

//From https://github.com/Team254/FRC-2016-Public/blob/811d5e11867ef7c659f98e01f48e3720d8724df1/vision_app/app/src/main/java/com/team254/cheezdroid/comm/messages/VisionMessage.java
//Once again we copy the poofs.

import com.google.gson.JsonObject;
import com.google.gson.JsonSyntaxException;

public abstract class VisionMessage {

    public abstract String getType();

    public abstract JsonObject getMessage();

    public String toJson() {
        JsonObject j = new JsonObject();
        try {
            j.addProperty("type", getType());
            j.add("message", getMessage());
        } catch (JsonSyntaxException e) {
            System.out.println("VisionMessage.java:" + "Could not encode JSON");
        }
        return j.toString() + "\n";
    }
}

