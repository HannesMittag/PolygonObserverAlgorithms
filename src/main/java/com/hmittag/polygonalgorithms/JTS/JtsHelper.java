package com.hmittag.polygonalgorithms.JTS;

import com.hmittag.polygonalgorithms.Model.Polygon.Polygon;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineSegment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class JtsHelper {

    public static Coordinate vector2JTSCoordinate(final Vector v) {
        Coordinate c = null;
        if (v != null)  {
            c = new Coordinate(v.getX(), v.getY());
        }
        return c;
    }
    public static Vector JTSCoordinate2Vector(final Coordinate coordinate)  {
        Vector v = new Vector();
        if (coordinate != null) {
            v.setX(coordinate.getX());
            v.setY(coordinate.getY());
        }

        return v;
    }

    public static org.locationtech.jts.geom.Polygon polygon2JTSPolygon(final Polygon p)    {
        org.locationtech.jts.geom.Polygon polygon = null;
        if (p != null)  {
            Coordinate [] cs = new Coordinate[p.getVertices().size() + 1];
            for (int i = 0; i < p.getVertices().size(); i++)  {
                cs[i] = vector2JTSCoordinate(p.getVertices().get(i));
            }
            cs[cs.length-1] = vector2JTSCoordinate(p.getVertices().get(0));

            polygon = new GeometryFactory().createPolygon(cs);
        }
        return polygon;
    }

    public static Polygon JTSPolygon2Polygon(final Geometry polygon)   {
        try {
            org.locationtech.jts.geom.Polygon jtsPolygon = (org.locationtech.jts.geom.Polygon) polygon;
            List<Vector> polygonPoints = new ArrayList<>();
            if (polygon != null) {
                Coordinate[] coordinates = jtsPolygon.getCoordinates();
                for (Coordinate c : coordinates) {
                    polygonPoints.add(new Vector(c.getX(), c.getY()));
                }
            }

            return new Polygon(polygonPoints);
        }
        catch (Exception exception) {
            //pass
            return null;
        }
    }

    public static Coordinate[] geometryListToCoordinates(List<Geometry> geometryList)   {
        if (geometryList != null)   {
            List<Coordinate> coordinates = new ArrayList<>();
            for (Geometry geometry : geometryList) {
                Coordinate[] geom = geometry.getCoordinates();
                coordinates.addAll(Arrays.asList(geom));
            }

            return coordinates.toArray(new Coordinate[0]);
        }
        return null;
    }

    public static LineSegment jtsLineSegmentFromTwoPoints(Vector p, Vector p1)    {
        return new LineSegment(new Coordinate(p.getX(), p.getY()), new Coordinate(p1.getX(), p1.getY()));
    }

    public static List<LineSegment> polygon2JtsLineSegments(Polygon p)  {
        List<LineSegment> lineSegments = new ArrayList<>();
        for (int i = 0; i < p.getVertices().size()-1 ; i++)    {
            lineSegments.add(jtsLineSegmentFromTwoPoints(p.getVertices().get(i), p.getVertices().get(i+1)));
        }

        lineSegments.add(jtsLineSegmentFromTwoPoints(p.getVertices().get(p.getVertices().size()-1), p.getVertices().get(0)));

        return lineSegments;
    }

}
