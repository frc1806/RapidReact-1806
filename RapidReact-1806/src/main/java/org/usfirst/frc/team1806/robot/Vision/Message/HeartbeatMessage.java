package org.usfirst.frc.team1806.robot.Vision.Message;

import com.google.gson.*;

public class HeartbeatMessage extends VisionMessage {

    double timestamp;
    String source;
    boolean valid;

    public HeartbeatMessage(double timeStamp){
        this.timestamp = timeStamp;
        this.source = "roborio";
        valid = true;
    }

    public HeartbeatMessage(JsonObject message){
        try{

            JsonElement sourceJson = message.get("source");
            this.valid = true;
            if(sourceJson != null){
                this.source = sourceJson.getAsJsonPrimitive().getAsString();
            }
            else{
                this.source = "unknown";
                this.valid = false;
            }

            JsonElement timeJson = message.get("timestamp");
            if (timeJson != null) {
                this.timestamp = timeJson.getAsJsonPrimitive().getAsBigDecimal().doubleValue();
            }
            else{
                this.timestamp = 0;
                this.valid = false;
            }
        }
        catch(JsonParseException parseException){
            System.out.println("Received invalid heartbeat");
            this.timestamp = 0;
            this.source = "";
            this.valid = false;
        }

    }

    public String getType(){
        return "heartbeat";
    }

    public double getTimestamp() {
        return timestamp;
    }

    public String getSource() {
        return source;
    }

    public boolean isValid() {
        return valid;
    }

    public JsonObject getMessage(){
        JsonObject message = new JsonObject();
        message.add("source", new JsonPrimitive(source));
        message.add("timestamp", new JsonPrimitive(timestamp));
        return message;
    }
}