package com.hmittag.polygonalgorithms.DYN4J;

import com.hmittag.polygonalgorithms.Model.Polygon.Polygon;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import org.dyn4j.geometry.Vector2;

import java.util.ArrayList;
import java.util.List;

public class Dyn4JHelper {

    public static Vector vector2ToVector(Vector2 vector2)   {
        if (vector2 != null)    {
            return new Vector(vector2.x, vector2.y);
        }
        return null;
    }

    public static List<Vector> vector2ListToVectorList(List<Vector2> vector2List)   {
        List<Vector> ret = new ArrayList<>();
        if ( vector2List != null)   {
            for (Vector2 v : vector2List)   {
                ret.add(new Vector(v.x, v.y));
            }
        }
        return ret;
    }

    public static List<Vector2> VectorListToVector2List(List<Vector> vectorList)   {
        List<Vector2> ret = new ArrayList<>();
        if ( vectorList != null)   {
            for (Vector v : vectorList)   {
                ret.add(new Vector2(v.getX(), v.getY()));
            }
        }
        return ret;
    }

    public static Polygon dyn4JPolygonToPolygon(org.dyn4j.geometry.Polygon polygon) {
        List<Vector> ret = new ArrayList<>();

        if (polygon != null)    {
            for (Vector2 v : polygon.getVertices()) {
                ret.add(vector2ToVector(v));
            }
        }

        return new Polygon(ret);
    }

    public static org.dyn4j.geometry.Polygon polygonToDyn4JPolygon(Polygon polygon) {
        if (polygon != null)    {
            List<Vector2> vert = new ArrayList<>();

            for (Vector v : polygon.getVertices())    {
                vert.add(new Vector2(v.getX(), v.getY()));
            }

            return new org.dyn4j.geometry.Polygon(vert.toArray(new Vector2[0]));
        }
        return null;
    }

}
