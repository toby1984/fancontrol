package de.codesourcery.fancontrol;

import com.fasterxml.jackson.annotation.*;

@JsonTypeInfo(use = JsonTypeInfo.Id.NAME,
    include = JsonTypeInfo.As.EXISTING_PROPERTY,
    property = "type")
@JsonSubTypes({
    @JsonSubTypes.Type(value = Sensor.MainboardSensor.class, name = "mainboard"),
    @JsonSubTypes.Type(value = Sensor.HddSensor.class, name = "hdd")
})
public abstract class Sensor
{
    enum Type
    {
        MAINBOARD("mainboard"),
        HDD("hdd");

        private final String jsonIdentifier;

        Type(String jsonIdentifier)
        {
            this.jsonIdentifier = jsonIdentifier;
        }

        @JsonValue
        public String getJsonIdentifier()
        {
            return jsonIdentifier;
        }
    }

    public static final class MainboardSensor extends Sensor
    {
        public String chip;
        public String feature;

        @JsonProperty("subfeature")
        public String subFeature;

        @Override
        public Type getType()
        {
            return Type.MAINBOARD;
        }
    }

    public static final class HddSensor extends Sensor
    {
        public String device;

        @Override
        public Type getType()
        {
            return Type.HDD;
        }
    }

    public abstract Type getType();
}