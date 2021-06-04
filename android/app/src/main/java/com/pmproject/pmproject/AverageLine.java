package com.pmproject.pmproject;

public class AverageLine {

    public double a = 0, b = 0, c = 0;
    public int n = 0;

    public AverageLine()
    {

    }

    public void addLine(Vec2 from, Vec2 to)
    {
        double x0 = from.x;
        double y0 = from.y;
        double x1 = to.x;
        double y1 = to.y;

        if (Math.abs(y0 - y1) < 0.001f)
            y1 += 0.1f;
        double a = y0 - y1;
        double b = x1 - x0;
        double c = x0 * y1 - x1 * y0;
        b /= a;
        c /= a;
        a /= a;
        if (a < 0) {
            a = -a;
            b = -b;
            c = -c;
        }
        this.a += a;
        this.b += b;
        this.c += c;
        this.n++;
    }

    public void submit()
    {
        a /= n;
        b /= n;
        c /= n;
    }

    public Vec2 intersect(AverageLine line)
    {
        double a0 = this.a;
        double b0 = this.b;
        double c0 = this.c;
        double a1 = line.a;
        double b1 = line.b;
        double c1 = line.c;
        double x = (b0 * c1 - b1 * c0) / (b1 * a0 - a1 * b0);
        double y = (a0 * c1 - a1 * c0) / (a1 * b0 - b1 * a0);
        return new Vec2(x, y);
    }

}
