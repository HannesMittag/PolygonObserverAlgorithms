package com.hmittag.polygonalgorithms.Model.Vector;

public class Vector {
    //region fields
    private double x;
    private double y;

    //endregion

    //region Constructor
    public Vector ()	{
        this.x = 0;
        this.y = 0;
    }
    public Vector (double xPos, double yPos)	{
        this.x = xPos;
        this.y = yPos;
    }
    //endregion

    //region getter setter
    public double getX() {
        return this.x;
    }
    public double getY()	{
        return this.y;
    }

    public void setX(double X)	{
        this.x = X;
    }
    public void setY(double Y) {
        this.y = Y;
    }
    //endregion

    //region Vector operations
    //add
    public static Vector add(Vector v1, Vector v2)	{
        return new Vector((v1.x+v2.x), (v1.y+v2.y));
    }
    public void add(int value)	{
        this.x += 1;
        this.y += 1;
    }
    //sub
    public static Vector sup(Vector v1, Vector v2)	{
        return new Vector((v1.x-v2.x), (v1.y-v2.y));
    }
    public void sub(int value)	{
        this.x -= 1;
        this.y -= 1;
    }
    //mult
    public void mult(double n)	{
        this.x = this.x*n;
        this.y = this.y*n;
    }
    public static Vector mult(Vector v1, double n)	{
        return new Vector((v1.x*n), (v1.y*n));
    }
    public static double mult(Vector v1, Vector v2)	{
        return (v1.x*v2.x) + (v1.y*v2.y);
    }
    //div
    public static Vector div(Vector v1, double n)	{
        return new Vector((v1.x/n), (v1.y/n));
    }
    public void div(double n)	{
        this.x = this.x/n;
        this.y = this.y/n;
    }
    //magnitude
    public static double mag(Vector v)	{
        return Math.sqrt((Math.pow(v.x, 2)) + (Math.pow(v.y, 2)));
    }
    //normalize
    public void normalize()	{
        this.div(mag(this));
    }
    //limit
    public void limit(double max) {
        if (mag(this) > max) {
            this.normalize();
            this.mult(max);
        }
    }

    //endregion

    //region override
    public String toString()	{
        return "[" + this.x + "," + this.y + "]";
    }

    public boolean equals(Object obj) {
        Vector v = (Vector) obj;
        return this.x == v.getX() && this.y == v.getY();
    }

    //endregion
}
