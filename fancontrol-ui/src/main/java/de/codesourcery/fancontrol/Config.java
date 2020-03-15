package de.codesourcery.fancontrol;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonValue;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

public class Config
{
    @JsonProperty("min_fan_speed")
    public int minFanSpeed;

    @JsonProperty("max_fan_speed")
    public int maxFanSpeed;

    @JsonProperty("smoothing_alpha")
    public float smoothingAlpha;

    @JsonProperty("loop_delay_millis")
    public int loopDelayMillis;

    @JsonProperty("serial_device")
    public String serialDevice;

    @JsonProperty("baud_rate")
    public int baud_rate;

    @JsonProperty("thermal_zones")
    public final List<ThermalZone> thermalZones = new ArrayList<>();

    public String toJSON() throws JsonProcessingException
    {
        final ObjectMapper mapper = new ObjectMapper();
        mapper.enable(SerializationFeature.INDENT_OUTPUT);
        return mapper.writeValueAsString(this);
    }

    public static Config fromJSON(InputStream in) throws IOException
    {
        return new ObjectMapper().readValue( in, Config.class );
    }
}
