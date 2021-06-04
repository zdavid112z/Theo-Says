package com.pmproject.pmproject;

public class Vec2 {

    public double x, y;

    public Vec2()
    {
        x = 0;
        y = 0;
    }

    public Vec2(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public double length()
    {
        return Math.sqrt(x * x + y * y);
    }

    public Vec2 normalized()
    {
        double len = length();
        if (len < 0.001) {
            return new Vec2(0, 0);
        }
        return new Vec2(x / len, y / len);
    }

    public Vec2 plus(Vec2 v)
    {
        return new Vec2(x + v.x, y + v.y);
    }

    public Vec2 minus(Vec2 v)
    {
        return new Vec2(x - v.x, y - v.y);
    }

    public void copyFrom(Vec2 other)
    {
        x = other.x;
        y = other.y;
    }

    public Vec2 sqrt()
    {
        return new Vec2(Math.sqrt(x), Math.sqrt(y));
    }

}
