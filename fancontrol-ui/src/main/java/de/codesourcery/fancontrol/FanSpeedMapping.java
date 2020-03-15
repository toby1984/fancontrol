package de.codesourcery.fancontrol;

import org.apache.commons.lang3.Validate;

import java.awt.Point;
import java.util.Objects;

public class FanSpeedMapping
{
    public int temperature;
    public int fanSpeed;

    public FanSpeedMapping() {
    }

    public FanSpeedMapping(int temperature, int fanSpeed)
    {
        setTemperature(temperature);
        setFanSpeed(fanSpeed);
    }

    @Override
    public boolean equals(Object o)
    {
        if (this == o)
        {
            return true;
        }
        if (o == null || getClass() != o.getClass())
        {
            return false;
        }
        FanSpeedMapping that = (FanSpeedMapping) o;
        return temperature == that.temperature &&
                   fanSpeed == that.fanSpeed;
    }

    @Override
    public int hashCode()
    {
        return Objects.hash(temperature, fanSpeed);
    }

    public Point toPoint() {
        return new Point(temperature,fanSpeed);
    }

    public void setFromPoint(Point p) {
        Validate.notNull(p, "p must not be null");
        setTemperature(p.x);
        setFanSpeed(p.y);
    }

    public int getTemperature()
    {
        return temperature;
    }

    public void setTemperature(int temperature)
    {
        if ( temperature < 0 || temperature > 100 ) {
            throw new IllegalArgumentException("Invalid temperature: "+temperature);
        }
        this.temperature = temperature;
    }

    public int getFanSpeed()
    {
        return fanSpeed;
    }

    public void setFanSpeed(int fanSpeed)
    {
        if ( fanSpeed < 0 || fanSpeed > 100 ) {
            throw new IllegalArgumentException("Invalid fan speed: "+fanSpeed);
        }
        this.fanSpeed = fanSpeed;
    }
}
