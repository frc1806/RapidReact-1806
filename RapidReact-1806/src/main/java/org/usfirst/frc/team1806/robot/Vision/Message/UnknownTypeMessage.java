package org.usfirst.frc.team1806.robot.Vision.Message;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import com.google.gson.JsonParser;

public class UnknownTypeMessage extends VisionMessage {

    JsonObject jsonMessage;
    boolean isValid;
    String type;

    public UnknownTypeMessage(String jsonMessage){
        isValid = true;
        JsonElement messageElement;
        JsonParser messageParser = new JsonParser();
        try {
            messageElement = messageParser.parse(jsonMessage);
            JsonObject messageObject = messageElement.getAsJsonObject();

            type = messageObject.get("type").getAsJsonPrimitive().getAsString();
            this.jsonMessage = messageObject.get("message").getAsJsonObject();
        }
        catch(JsonParseException | IllegalStateException parseException){
            System.out.println("received invalid message: " + jsonMessage);
            this.isValid = false;
        }
    }

    public String getType(){
        return type;
    }

    public JsonObject getMessage(){
        return jsonMessage;
    }

    public boolean isValid(){
        return isValid;
    }
}
