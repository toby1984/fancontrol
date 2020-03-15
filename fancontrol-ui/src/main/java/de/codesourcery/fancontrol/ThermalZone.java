package de.codesourcery.fancontrol;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.*;

public class ThermalZone
{
    public String name;
    public List<Sensor> sensors = new ArrayList<>();

    @JsonProperty("fan_speed_mapping")
    public Map<String,Integer> fanSpeedMapping = new HashMap<>();

    @JsonIgnore
    public List<FanSpeedMapping> getFanSpeedMapping() {
        List<FanSpeedMapping> result = new ArrayList<>();
        for ( Map.Entry<String, Integer> entry : fanSpeedMapping.entrySet() ) {
            result.add( new FanSpeedMapping(Integer.parseInt(entry.getKey()), entry.getValue() ) );
        }
        return result;
    }

    @JsonIgnore
    public void setFanSpeedMapping(List<FanSpeedMapping> mapping) {
        this.fanSpeedMapping.clear();
        for ( FanSpeedMapping m : mapping ) {
            fanSpeedMapping.put( Integer.toString( m.temperature ) , m.fanSpeed );
        }
    }
}