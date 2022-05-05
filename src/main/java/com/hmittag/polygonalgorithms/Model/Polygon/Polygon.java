package com.hmittag.polygonalgorithms.Model.Polygon;

import com.hmittag.polygonalgorithms.JTS.JtsHelper;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import org.locationtech.jts.algorithm.ConvexHull;
import org.locationtech.jts.algorithm.Orientation;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryCollection;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.operation.buffer.BufferOp;
import org.locationtech.jts.operation.buffer.BufferParameters;
import org.locationtech.jts.triangulate.DelaunayTriangulationBuilder;

import java.util.ArrayList;
import java.util.List;

public class Polygon {

    //region constants
    public static final Vector[] SAMPLE_POLYGON_0 = new Vector[]   {new Vector(269, 149), new Vector(417, 148)
    , new Vector(428, 306), new Vector(569, 316), new Vector(589, 140), new Vector(743, 141)
            , new Vector(735, 534), new Vector(285, 550)};

    public static final Vector[] SAMPLE_POLYGON_1 = new Vector[]    {new Vector(206.0,134.0), new Vector(649.0,135.0), new Vector(743.0,460.0)
            , new Vector(555.0,280.0), new Vector(583.0,486.0), new Vector(330.0,383.0), new Vector(380.0,242.0)
            , new Vector(239.0,220.0), new Vector(245.0,180.0)};

    public static final Vector[] SAMPLE_POLYGON_2 = new Vector[]    { new Vector(294.0, 170.0), new Vector(411.0, 86.0), new Vector(370.0
            , 247.0), new Vector(551.0, 105.0), new Vector(600.0, 209.0), new Vector(727.0, 281.0), new Vector(613.0, 313.0)
            , new Vector(548.0, 235.0), new Vector(618.0, 556.0), new Vector(455.0, 555.0), new Vector(483.0, 379.0)
            , new Vector(452.0, 275.0), new Vector(295.0, 451.0), new Vector(229.0, 358.0), new Vector(279.0, 305.0)};

    public static final Vector[] SAMPLE_POLYGON_3 = new Vector[]    { new Vector(237.0, 94.0)
            , new Vector(320.0, 76.0), new Vector(299.0, 152.0), new Vector(249.0, 186.0)
            , new Vector(263.0, 258.0), new Vector(344.0, 258.0), new Vector(425.0, 245.0)
            , new Vector(496.0, 247.0), new Vector(536.0, 291.0), new Vector(451.0, 353.0)
            , new Vector(311.0, 365.0), new Vector(209.0, 372.0), new Vector(254.0, 448.0)
            , new Vector(409.0, 446.0), new Vector(555.0, 449.0), new Vector(633.0, 443.0)
            , new Vector(725.0, 390.0), new Vector(800.0, 478.0), new Vector(642.0, 566.0)
            , new Vector(479.0, 563.0), new Vector(361.0, 567.0), new Vector(280.0, 631.0)
            , new Vector(164.0, 619.0), new Vector(95.0, 583.0), new Vector(78.0, 283.0)
            , new Vector(101.0, 162.0), new Vector(140.0, 102.0)};

    public static final Vector[] SAMPLE_POLYGON_4 = new Vector[] { new Vector(276.0, 148.0), new Vector(501.0
            , 121.0), new Vector(706.0, 303.0), new Vector(367.0, 421.0), new Vector(211.0
            , 333.0) };

    public static final Vector[] SAMPLE_POLYGON_5 = new Vector[] { new Vector(706.0, 307.0), new Vector(511.0
            , 453.0), new Vector(265.0, 365.0), new Vector(166.0, 169.0), new Vector(795.0
            , 118.0) };

    public static final Vector[] SAMPLE_POLYGON_6 = new Vector[] { new Vector(257.0, 251.0), new Vector(449.0
            , 109.0), new Vector(555.0, 107.0), new Vector(535.0, 432.0), new Vector(454.0
            , 431.0) };
    //endregion

    //region fields
    private List<Vector> vertices;
    //endregion

    //region constructor
    public Polygon() {
        this.vertices = new ArrayList<>();
    }

    public Polygon(final List<Vector> points) {
        this.vertices = points;
    }

    public Polygon(final Vector[] points) {
        if (points != null) {
            this.vertices = new ArrayList<>();
            for (Vector v : points) {
                this.vertices.add(v);
            }
        }
    }

    //endregion

    public void addPoint(final Vector vector) {
        if (this.vertices == null)    {
            this.vertices = new ArrayList<>();
        }

        this.vertices.add(vector);
    }

    public GeometryCollection triangulate()   {
        Geometry polygon = JtsHelper.polygon2JTSPolygon(this);
        org.locationtech.jts.geom.Polygon polygon1 = (org.locationtech.jts.geom.Polygon) polygon;
        DelaunayTriangulationBuilder builder = new DelaunayTriangulationBuilder();
        builder.setSites(polygon);

        GeometryCollection triangulation = (GeometryCollection) builder.getTriangles(new GeometryFactory());
        triangulation = (GeometryCollection) triangulation.intersection(polygon);

        return triangulation;
    }

    public Geometry bufferOp(double offset)  {
        Geometry polygon = JtsHelper.polygon2JTSPolygon(this);
        Geometry geom = BufferOp.bufferOp(polygon, offset, new BufferParameters(0, BufferParameters.CAP_FLAT, BufferParameters.JOIN_MITRE, 10));
        System.out.println(geom);
        return geom;
    }

    public boolean neighborsPolygon(Polygon p)  {
        if (p != null)  {
            List<Vector> vertices = p.getVertices();
            for (int i = 0; i < this.vertices.size(); i++)  {
                Vector v = this.vertices.get(i);
                for (int j = 0; j < vertices.size(); j++)   {
                    Vector v1 = vertices.get(j);
                    if (v.equals(v1))   {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    //region getter setter

    public List<Vector> getVertices() {
        return vertices;
    }
    public void setVertices(final List<Vector> vertices) {
        this.vertices = vertices;
    }

    //endregion

    public String toString() {
        String s = "{";
        if (vertices != null) {
            for (Vector v : this.vertices)    {
                s += " new Vector(" + v.getX() + "," + " " + v.getY() + "),";
            }
        }
        s += "}";
        return s;
    }

    //region statics
    public static Polygon makeClockwise(Polygon p)  {
        if (!isClockwise(p)) {
            List<Vector> newVertices = new ArrayList<>();
            for (int i = p.getVertices().size()-1; i >= 0; i--) {
                newVertices.add(p.getVertices().get(i));
            }
            p.setVertices(newVertices);
        }

        return p;
    }

    public static boolean isClockwise(Polygon p)    {
        Coordinate[] cs = new Coordinate[p.getVertices().size()];
        for (int i = 0; i < cs.length; i++)  {
            cs[i] = new Coordinate(p.getVertices().get(i).getX(), p.getVertices().get(i).getY());
        }
        return !Orientation.isCCW(cs);
    }
    //endregion
}
