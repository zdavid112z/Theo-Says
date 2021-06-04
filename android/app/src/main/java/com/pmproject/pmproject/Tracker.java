package com.pmproject.pmproject;

import android.util.Pair;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;

public class Tracker {

    public final int n;
    private final Deque<Vec2> q = new ArrayDeque<>();
    private final ArrayList<Vec2> points = new ArrayList<>();
    private Vec2 lastPoint;

    private Vec2 mean;
    private Vec2 sigma;
    private Vec2 sigma2;
    private Vec2 normMean;
    private Vec2 normSigma;
    private Vec2 normSigma2;
    private int elementsCount;

    public Tracker(int n)
    {
        this.n = n;
        clear();
    }

    public void clear()
    {
        points.clear();
        q.clear();
        for (int i = 0; i < n; i++)
            q.add(new Vec2());
        for (int i = 0; i <= n; i++)
            points.add(new Vec2());
        mean = new Vec2();
        sigma = new Vec2();
        sigma2 = new Vec2();
        normMean = new Vec2();
        normSigma = new Vec2();
        normSigma2 = new Vec2();
        lastPoint = new Vec2();
        elementsCount = 0;
    }

    private Pair<Double, Double> updateSingle(double oldValue, double newValue, double oldMean, double oldSigma2)
    {
        double newMean = oldMean + (newValue - oldValue) / n;
        double newSigma2 = oldSigma2 + (newValue - oldValue) * (newValue - newMean + oldValue - oldMean) / (n - 1);
        return new Pair<>(newMean, Math.max(newSigma2, 0));
    }

    public void addPoint(Vec2 point)
    {
        point = new Vec2(point.x, point.y);
        points.add(point);
        points.remove(0);

        Vec2 newValue = point.minus(lastPoint);
        q.addLast(newValue);
        lastPoint.copyFrom(point);
        Vec2 oldValue = q.pollFirst();
        assert oldValue != null;

        Vec2 newValueNorm = newValue.normalized();
        Vec2 oldValueNorm = oldValue.normalized();

        Pair<Double, Double> x = updateSingle(oldValue.x, newValue.x, mean.x, sigma2.x);
        Pair<Double, Double> y = updateSingle(oldValue.y, newValue.y, mean.y, sigma2.y);
        Pair<Double, Double> nx = updateSingle(oldValueNorm.x, newValueNorm.x, normMean.x, normSigma2.x);
        Pair<Double, Double> ny = updateSingle(oldValueNorm.y, newValueNorm.y, normMean.y, normSigma2.y);

        mean.x = x.first;
        mean.y = y.first;
        sigma2.x = x.second;
        sigma2.y = y.second;
        normMean.x = nx.first;
        normMean.y = ny.first;
        normSigma2.x = nx.second;
        normSigma2.y = ny.second;

        sigma = sigma2.sqrt();
        normSigma = normSigma2.sqrt();
        elementsCount = Math.min(elementsCount + 1, n);
    }

    public boolean isReady()
    {
        return elementsCount == n;
    }

    public Vec2 getSigma() {
        return sigma;
    }

    public Vec2 getNormSigma() {
        return normSigma;
    }

    public Vec2 getMean() {
        return mean;
    }

    public Vec2 getNormMean() {
        return normMean;
    }

    public ArrayList<Vec2> getPoints() {
        return points;
    }
}
