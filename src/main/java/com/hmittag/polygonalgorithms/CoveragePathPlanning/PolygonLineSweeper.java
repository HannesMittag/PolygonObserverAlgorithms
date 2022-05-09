package com.hmittag.polygonalgorithms.CoveragePathPlanning;

import com.hmittag.polygonalgorithms.JTS.JtsHelper;
import com.hmittag.polygonalgorithms.Model.Polygon.Polygon;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.util.Pair;
import org.locationtech.jts.algorithm.Angle;
import org.locationtech.jts.algorithm.Orientation;
import org.locationtech.jts.geom.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class PolygonLineSweeper {
    //region constants
    private static final double POINT_ON_LINE_THRESHOLD = 0.1;
    //endregion

    //region fields
    private GeometryFactory geometryFactory;

    //test
    private GraphicsContext graphicsContext;
    //endregion

    //region consturctor
    public PolygonLineSweeper() {
        this.geometryFactory = new GeometryFactory();
    }
    //endregion

    //region cpp
    public boolean sweep(Polygon polygon, double workingWidth)  {
        Geometry hull = JtsHelper.polygon2JTSPolygon(polygon).convexHull();
        Polygon test = JtsHelper.JTSPolygon2Polygon(hull);
        test.getVertices().remove(test.getVertices().size()-1);

        //strokePolygonLine(test, true);

        /*double[] polygonBoundingBox = computePolygonBoundingBox(polygon.getVertices());
        PolygonLineSweeper.SweepLine sweepLine = findLineSweepDirection(polygon, null);*/
        double[] polygonBoundingBox = computePolygonBoundingBox(test.getVertices());
        PolygonLineSweeper.SweepLine sweepLine = findLineSweepDirection(test, null);

        List<Vector> cpp0 = sweep(polygon, polygonBoundingBox, sweepLine, workingWidth);
        if (cpp0 != null)   {
            polygon.setCpp0(cpp0);
            computeCoverageAlternatives(polygon);
            return true;
        }
        else {
            List<LineSegment> forbiddenLineSegments = new ArrayList<>();
            forbiddenLineSegments.add(sweepLine.lineSegment);
            int counter = 0;
            while (counter <= polygon.getVertices().size()-1) {
                PolygonLineSweeper.SweepLine sweepLineNew = findLineSweepDirection(test, forbiddenLineSegments);
                if (sweepLineNew == null)   {
                    return false;
                }
                cpp0 = sweep(polygon, polygonBoundingBox, sweepLineNew, workingWidth);
                if (cpp0 != null)   {
                    break;
                }
                forbiddenLineSegments.add(sweepLineNew.lineSegment);
                counter++;
            }

            if (cpp0 != null)   {
                polygon.setCpp0(cpp0);
                computeCoverageAlternatives(polygon);
                return true;
            }
        }

        return false;
    }

    private List<Vector> sweep(Polygon polygon, double[] polygonBoundingBox, PolygonLineSweeper.SweepLine sweepLine, double workingWidth)    {
        List<Vector> cppPoints = new ArrayList<>();

        if (sweepLine != null) {
            org.locationtech.jts.geom.Polygon jtsPolygon = JtsHelper.polygon2JTSPolygon(polygon);
            if (!jtsPolygon.isValid())  {
                strokePolygonLine(polygon, true);
                System.out.println(polygon);
                System.out.println("\n!!!!!!CANNOT SWEEP, POLYGON INVALID\n");
            }
            List<LineSegment> polygonEdges = JtsHelper.polygon2JtsLineSegments(polygon);
            LineString currentLineString = stretchSweepLine(sweepLine, polygonBoundingBox);
            //strokeLineString(currentLineString, true);
            System.out.println("\n\n");

            //add root points to cpp
            /*cppPoints.add(JtsHelper.JTSCoordinate2Vector(sweepLine.lineSegment.p1));
            cppPoints.add(JtsHelper.JTSCoordinate2Vector(sweepLine.lineSegment.p0));*/

            //init translate
            double hackeyWorkingWidth = 1.2;
            LineString testLS = translateParallel(currentLineString, workingWidth);
            //currentLineString = translateParallel(currentLineString, workingWidth);
            //strokeLineString(currentLineString, true);

            if (!jtsPolygon.intersects(testLS))  {
                workingWidth *= -1;
                hackeyWorkingWidth *= -1;
            }
            currentLineString = translateParallel(currentLineString, hackeyWorkingWidth);
            //strokeLineString(currentLineString, true);

            Geometry diff = currentLineString.difference(jtsPolygon);
            Coordinate[] testCoord = diff.getCoordinates();

            if (testCoord.length != 4)   {
                return null;
            }
            cppPoints.add(JtsHelper.JTSCoordinate2Vector(testCoord[1]));
            cppPoints.add(JtsHelper.JTSCoordinate2Vector(testCoord[2]));

            currentLineString = translateParallel(currentLineString, workingWidth);

            // loop
            int ticker = 1;
            int test = 0;
            while (jtsPolygon.intersects(currentLineString))  {
                //strokeLineString(currentLineString, true);
                List<Coordinate> coordinates = new ArrayList<>();
                Geometry diff1 = currentLineString.difference(jtsPolygon);
                Coordinate[] testCoord1 = diff1.getCoordinates();

                if (testCoord1.length != 4)   {
                    return null;
                }

                coordinates.add(testCoord1[1]);
                coordinates.add(testCoord1[2]);


                //if more than two intersections found, mission cannot be build
                if (coordinates.size() > 2) {
                    return null;
                }

                //add points to cpp
                switch (ticker) {
                    case 0:
                        cppPoints.add(JtsHelper.JTSCoordinate2Vector(coordinates.get(0)));
                        if (coordinates.size() > 1) {
                            cppPoints.add(JtsHelper.JTSCoordinate2Vector(coordinates.get(1)));
                        }
                        ticker = 1;
                        break;

                    case 1:
                        if (coordinates.size() > 1) {
                            cppPoints.add(JtsHelper.JTSCoordinate2Vector(coordinates.get(1)));
                        }
                        cppPoints.add(JtsHelper.JTSCoordinate2Vector(coordinates.get(0)));
                        ticker = 0;
                        break;
                }

                currentLineString = translateParallel(currentLineString, workingWidth);
                test++;
            }

            //strokeLine(cppPoints, false);

        }

        return cppPoints;
    }

    private PolygonLineSweeper.SweepLine findLineSweepDirection(Polygon p, List<LineSegment> forbiddenLineSegments)    {
        PolygonLineSweeper.SweepLine sweepLine = null;
        int sweepLineBeginIndex = 0;
        if (p != null)  {
            List<LineSegment> edges = JtsHelper.polygon2JtsLineSegments(p);
            List<Vector> vertices = p.getVertices();
            Vector opposedVertex = null;
            boolean isFirstEdge = true;
            double optimalDist = 0;
            for (LineSegment edge : edges)  {
                double maxDist = 0;
                for (Vector vert : vertices) {
                    double dist = edge.distancePerpendicular(JtsHelper.vector2JTSCoordinate(vert));
                    if (dist > maxDist) {
                        maxDist = dist;
                        opposedVertex = vert;
                    }
                }

                if (maxDist < optimalDist || isFirstEdge) {
                    if (!isForbiddenEdge(edge, forbiddenLineSegments)) {
                        optimalDist = maxDist;
                        isFirstEdge = false;
                        Vector sweepDirection = computeSweepLine(opposedVertex
                                , computePerpendicularIntersection(JtsHelper.JTSCoordinate2Vector(edge.p0), JtsHelper.JTSCoordinate2Vector(edge.p1), opposedVertex));
                        sweepLine = new PolygonLineSweeper.SweepLine(edge, sweepDirection);
                    }
                }
            }
        }
        else {
            System.out.println("polygon is null");
        }

        if (sweepLine == null)  {
            return null;
        }

        LineSegment ls = sweepLine.lineSegment;
        Coordinate p0 = ls.p0;
        for (Vector v : p.getVertices())    {
            if (v.getX() == p0.getX() && v.getY() == p0.getY()) {
                sweepLineBeginIndex = p.getVertices().indexOf(v);
            }
        }

        //reorg polygon
        /*List<Vector> correctedVert = new ArrayList<>();
        int counter = 0;
        int runnerIndex = sweepLineBeginIndex;
        while (counter <= p.getVertices().size())    {
            correctedVert.add(p.getVertices().get(runnerIndex));

            runnerIndex++;
            if (runnerIndex == p.getVertices().size())    {
                runnerIndex = 0;
            }
            counter++;
        }
        p.setVertices(correctedVert);*/

        return sweepLine;
    }
    private boolean isForbiddenEdge(LineSegment edge, List<LineSegment> forbiddenLineSegments)    {
        if (forbiddenLineSegments != null) {
            for (LineSegment ls : forbiddenLineSegments) {
                if (/*ls.equals(edge)*/(ls.p0.equals(edge.p0) || ls.p0.equals(edge.p1)) && (ls.p1.equals(edge.p0) || ls.p1.equals(edge.p1))) {
                    return true;
                }
            }
        }
        return false;
    }

    private Vector computeSweepLine(Vector p, Vector p1)  {
        return new Vector(p1.getX() - p.getX(), p1.getY() - p.getY());
    }

    private LineString stretchSweepLine(PolygonLineSweeper.SweepLine sweepLine, double[] boundingBox)   {
        Vector v0 = new Vector(boundingBox[0], boundingBox[2]);
        Vector v1 = new Vector(boundingBox[1], boundingBox[2]);
        Vector v2 = new Vector(boundingBox[2], boundingBox[3]);
        Vector v3 = new Vector(boundingBox[0], boundingBox[3]);

        LineSegment ls0 = new LineSegment(JtsHelper.vector2JTSCoordinate(v0), JtsHelper.vector2JTSCoordinate(v1));
        LineString lString0 = ls0.toGeometry(geometryFactory);
        LineSegment ls1 = new LineSegment(JtsHelper.vector2JTSCoordinate(v1), JtsHelper.vector2JTSCoordinate(v2));
        LineString lString1 = ls0.toGeometry(geometryFactory);
        LineSegment ls2 = new LineSegment(JtsHelper.vector2JTSCoordinate(v2), JtsHelper.vector2JTSCoordinate(v3));
        LineString lString2 = ls0.toGeometry(geometryFactory);
        LineSegment ls3 = new LineSegment(JtsHelper.vector2JTSCoordinate(v3), JtsHelper.vector2JTSCoordinate(v0));
        LineString lString3 = ls0.toGeometry(geometryFactory);

        List<Coordinate> runnerLineStringCoords = new ArrayList<>();

        Coordinate c0 = sweepLine.lineSegment.lineIntersection(ls0);
        Coordinate c1 = sweepLine.lineSegment.lineIntersection(ls1);
        Coordinate c2 = sweepLine.lineSegment.lineIntersection(ls2);
        Coordinate c3 = sweepLine.lineSegment.lineIntersection(ls3);


        HashMap<Double, Point> hashMap = new HashMap<>();
        Point p = null;
        Point p1 = null;
        Point p2 = null;
        Point p3 = null;

        List<Double> doubles = new ArrayList<>();
        if (c0 != null) {
            p = geometryFactory.createPoint(c0);
            double dist = p.distance(lString0);
            doubles.add(dist);

            runnerLineStringCoords.add(c0);
        }
        if (c1 != null) {
            p1 = geometryFactory.createPoint(c1);
            double dist = p1.distance(lString1);
            doubles.add(dist);

            runnerLineStringCoords.add(c1);
        }
        if (c2 != null) {
            p2 = geometryFactory.createPoint(c2);
            double dist = p2.distance(lString2);
            doubles.add(dist);

            runnerLineStringCoords.add(c2);
        }
        if (c3 != null) {
            p3 = geometryFactory.createPoint(c3);
            double dist = p3.distance(lString3);
            doubles.add(dist);

            runnerLineStringCoords.add(c3);
        }


        Coordinate[] startAndEnd = new Coordinate[2];
        Pair<Coordinate, Coordinate> coordinatePair = null;

        double maxDist = 0;
        for (int i = 0; i < runnerLineStringCoords.size()-1; i++) {
            Coordinate c = runnerLineStringCoords.get(i);
            for (int j = i; j < runnerLineStringCoords.size(); j++) {
                Coordinate cRunner = runnerLineStringCoords.get(j);
                double dist = distanceBetweenTwoPoints(new double[] {c.x, c.y} , new double[] {cRunner.x, cRunner.y});

                if (dist > maxDist) {
                    coordinatePair = new Pair<>(c, cRunner);
                    maxDist = dist;
                }
            }
        }

        /*for (int i = 0; i < 2; i++) {
            int removal = 0;
            for (int j = 0; j < runnerLineStringCoords.size(); j++) {
                Coordinate c = runnerLineStringCoords.get(j);
                double distToCentroid = distanceBetweenTwoPoints(new double[] {c.x, c.y} , new double[] {polygonCentroid.getCoordinate().x, polygonCentroid.getCoordinate().y});
                if (distToCentroid > maxDist)   {
                    startAndEnd[i] = c;
                    maxDist = distToCentroid;
                }
            }
            runnerLineStringCoords.remove(removal);
            maxDist = 0;
        }*/

        LineString finishedLS = geometryFactory.createLineString(runnerLineStringCoords.toArray(new Coordinate[0]));

        LineString lsTest = geometryFactory.createLineString(new Coordinate[]   {coordinatePair.getKey(), coordinatePair.getValue()});
        List<Vector> vectors = new ArrayList<>();
        for (Coordinate c : finishedLS.getCoordinates())    {
            vectors.add(new Vector(c.getX(), c.getY()));
        }


        //strokeLine(vectors, false);

        return lsTest;

        //return finishedLS;
    }

    //coverage alternatives
    private void computeCoverageAlternatives(Polygon p)  {
        List<Vector> rootCoveragePath = p.getCpp0();
        if (rootCoveragePath != null)   {
            List<Vector> cpp1 = new ArrayList<>();
            List<Vector> cpp2 = new ArrayList<>();
            List<Vector> cpp3 = new ArrayList<>();

            //opposite line sweep
            for (int i = rootCoveragePath.size()-1; i >= 0; i--)    {
                cpp2.add(rootCoveragePath.get(i));
            }

            //opposite clockwise
            cpp1 = computeOppositeClockwiseCoverage(rootCoveragePath);
            cpp3 = computeOppositeClockwiseCoverage(cpp2);

            p.setCpp1(cpp1);
            p.setCpp2(cpp2);
            p.setCpp3(cpp3);
            p.createCoverageList();
        }
    }

    private List<Vector> computeOppositeClockwiseCoverage(List<Vector> rootCP)  {
        List<Vector> oppC = new ArrayList<>();
        for (int i = 1; i < rootCP.size(); i += 2) {
            oppC.add(rootCP.get(i));
            oppC.add(rootCP.get(i -1));
        }
        return oppC;
    }

    //sweep line record
    private record SweepLine (LineSegment lineSegment, Vector sweepDirection)   {
    }
    //endregion

    //region edge shifting
    public boolean shiftEdge(Polygon polygon, LineSegment edge, double offset)    {
        System.out.println("polygon: " + polygon);
        org.locationtech.jts.geom.Polygon jtsPolygon = JtsHelper.polygon2JTSPolygon(polygon);
        if (!jtsPolygon.isValid())   {
            System.out.println("POLYGON INVALID!");
        }
        //strokePolygonLine(polygon, false);
        //polygon.getVertices().remove(polygon.getVertices().get(0));
        //find edge start index
        Vector v0 = JtsHelper.JTSCoordinate2Vector(edge.p0);
        Vector v1 = JtsHelper.JTSCoordinate2Vector(edge.p1);
        int v0Index = polygon.getVertices().indexOf(v0);
        System.out.println("index v0: " + v0Index);
        int v1Index = polygon.getVertices().indexOf(v1);
        if (v1Index != v0Index+1 && v1Index != 0)   {
            int swap = v0Index;
            v0Index = v1Index;
            v1Index = swap;
        }

        /*org.locationtech.jts.geom.Polygon jtsPolygon = JtsHelper.polygon2JTSPolygon(polygon);
        if (!jtsPolygon.isValid())   {
            System.out.println("POLYGON INVALID!");
        }*/
        double[] polygonBoundingBox = computePolygonBoundingBox(polygon.getVertices());

        Vector startP = polygon.getVertices().get(v0Index);
        int endPIndex = v0Index+1;
        if (endPIndex >= polygon.getVertices().size())  {
            endPIndex = 0;
        }
        Vector endP = polygon.getVertices().get(endPIndex);
        System.out.println("index v1: " + endPIndex);
        //fillPoint(startP);

        if (hasShiftableEdge(polygon, v0Index, endPIndex)) {
            LineSegment ls = new LineSegment(JtsHelper.vector2JTSCoordinate(startP), JtsHelper.vector2JTSCoordinate(endP));
            LineString sweepLine = geometryFactory.createLineString(new Coordinate[] {ls.p0, ls.p1});
            sweepLine = translateParallel(sweepLine, offset);
            if (!sweepLine.intersects(jtsPolygon)) {
                offset *= -1;
            }
            sweepLine = translateParallel(sweepLine, offset*2);
            sweepLine = stretchSweepLine(new SweepLine(new LineSegment(sweepLine.getCoordinateN(0), sweepLine.getCoordinateN(1)), new Vector()), polygonBoundingBox);
            //strokeLineString(sweepLine, true);

            Geometry difference = sweepLine.difference(jtsPolygon);
            System.out.println("difference: " + difference);
            Coordinate[] coordinates = difference.getCoordinates();
            if (coordinates.length != 4) {
                System.out.println("length not 4");
                return false;
            }

            Vector p0New = JtsHelper.JTSCoordinate2Vector(coordinates[1]);
            Vector p1New = JtsHelper.JTSCoordinate2Vector(coordinates[2]);
            List<Vector> vertNew = polygon.getVertices();

            double dist0 = distanceBetweenTwoPoints(new double[]{p0New.getX(), p0New.getY()}, new double[]{vertNew.get(v0Index).getX(), vertNew.get(v0Index).getY()});
            double dist1 = distanceBetweenTwoPoints(new double[]{p1New.getX(), p1New.getY()}, new double[]{vertNew.get(v0Index).getX(), vertNew.get(v0Index).getY()});
            if (dist0 > dist1)  {
                vertNew.set(v0Index, p1New);
                vertNew.set(endPIndex, p0New);
            }
            else {
                vertNew.set(v0Index, p0New);
                vertNew.set(endPIndex, p1New);
            }

            //strokePolygonLine(polygon, true);
            return true;
        }
        return false;
    }

    private boolean hasShiftableEdge(Polygon polygon, int p0Index, int p1Index)  {
        Vector tip0;
        if (p0Index == 0)   {
            tip0 = polygon.getVertices().get(polygon.getVertices().size()-1);
        }
        else {
            tip0 = polygon.getVertices().get(p0Index-1);
        }

        Vector tip1;
        if (p1Index+1 > polygon.getVertices().size()-1) {
            tip1 = polygon.getVertices().get(0);
        }
        else {
            tip1 = polygon.getVertices().get(p1Index+1);
        }

        double angle1 = Angle.angleBetweenOriented(JtsHelper.vector2JTSCoordinate(tip0), JtsHelper.vector2JTSCoordinate(polygon.getVertices().get(p0Index))
                , JtsHelper.vector2JTSCoordinate(polygon.getVertices().get(p1Index)));


        double angle2 = Angle.angleBetweenOriented(JtsHelper.vector2JTSCoordinate(polygon.getVertices().get(p0Index)), JtsHelper.vector2JTSCoordinate(polygon.getVertices().get(p1Index))
                , JtsHelper.vector2JTSCoordinate(tip1));


        System.out.println("angle1: " + Math.toDegrees(angle1));
        System.out.println("angle2: " + Math.toDegrees(angle2));

        return (angle1 < 180 && angle1 > 0) && (angle2 < 180 && angle2 > 0);
    }
    //endregion

    //region helper computations
    private double[] computePolygonBoundingBox(List<Vector> polygonCoordinates) {
        double minY = polygonCoordinates.get(0).getY();
        double maxY = polygonCoordinates.get(0).getY();
        double minX = polygonCoordinates.get(0).getX();
        double maxX = polygonCoordinates.get(0).getX();

        for (Vector polygonCoordinate : polygonCoordinates) {
            if (polygonCoordinate.getX() < minX) {
                minX = polygonCoordinate.getX();
            } else if (polygonCoordinate.getX() > maxX) {
                maxX = polygonCoordinate.getX();
            }

            if (polygonCoordinate.getY() < minY) {
                minY = polygonCoordinate.getY();
            } else if (polygonCoordinate.getY() > maxY) {
                maxY = polygonCoordinate.getY();
            }
        }
        return new double[] {minX, maxX, minY, maxY};
    }

    private Vector computePerpendicularIntersection(Vector ls0, Vector ls1, Vector p)   {
        double k = ((ls1.getY() - ls0.getY()) * (p.getX() - ls0.getX()) - (ls1.getX() - ls0.getX()) * (p.getY() - ls0.getY()))
                / (Math.pow(ls1.getY() - ls0.getY(), 2) + Math.pow(ls1.getX() - ls0.getX(), 2));

        double x = p.getX() - k * (ls1.getY() - ls0.getY());
        double y = p.getY() + k * (ls1.getX() - ls0.getX());

        return new Vector(x, y);
    }

    private LineString translateParallel(LineString lineString, double dist)  {
        double r = Math.sqrt(Math.pow(lineString.getCoordinateN(1).x - lineString.getCoordinateN(0).x, 2) + Math.pow(lineString.getCoordinateN(1).y - lineString.getCoordinateN(0).y, 2));
        double deltaX = (dist/r) * (lineString.getCoordinateN(0).y - lineString.getCoordinateN(1).y);
        double deltaY = (dist/r) * (lineString.getCoordinateN(1).x - lineString.getCoordinateN(0).x);
        Coordinate transP0 = new Coordinate(lineString.getCoordinateN(0).x + deltaX, lineString.getCoordinateN(0).y + deltaY);
        Coordinate transP1 = new Coordinate(lineString.getCoordinateN(1).x + deltaX, lineString.getCoordinateN(1).y + deltaY);
        return geometryFactory.createLineString(new Coordinate[] {transP0, transP1});
    }

    private boolean pointLiesOnLine(Vector p, Vector v0, Vector v1) {
        double dxc = p.getX() - v0.getX();
        double dyc = p.getY() - v0.getY();

        double dxl = v1.getX() - v0.getX();
        double dyl = v1.getY() - v0.getY();

        double cross = dxc * dyl - dyc * dxl;

        return Math.abs(cross) <= POINT_ON_LINE_THRESHOLD;
    }

    private boolean pointLiesOnLineString(Vector p, Vector v0, Vector v1) {
        double dxc = p.getX() - v0.getX();
        double dyc = p.getY() - v0.getY();

        double dxl = v1.getX() - v0.getX();
        double dyl = v1.getY() - v0.getY();

        double cross = dxc * dyl - dyc * dxl;

        if (Math.abs(cross) > POINT_ON_LINE_THRESHOLD) {
            return false;
        }


        if (Math.abs(dxl) >= Math.abs(dyl))
            return dxl > 0 ?
                    v0.getX() <= p.getX() && p.getX() <= v1.getX() : v1.getX() <= p.getX() && p.getX() <= v0.getX();
        else
            return dyl > 0 ?
                    v0.getY() <= p.getY() && p.getY() <= v1.getY() : v1.getY() <= p.getY() && p.getY() <= v0.getY();

    }

    private Coordinate[] computeFurthestPairOfPoints(List<Coordinate> vectors)  {
        Coordinate[] coordinates = new Coordinate[2];
        double maxDist = 0;
        Coordinate finalCord = null;
        Coordinate finalCord1 = null;
        for (int i = 0; i < vectors.size(); i++)    {
            Coordinate cRoot = vectors.get(i);
            for (int j = i+1; j < vectors.size(); j++)    {
                Coordinate c = vectors.get(j);
                if (cRoot.distance(c) > maxDist)    {
                    maxDist = cRoot.distance(c);
                    finalCord = cRoot;
                    finalCord1 = c;

                }
            }
        }

        return new Coordinate[] {finalCord, finalCord1};
    }

    static LineString removeCollinearVertices(final LineString ls) {
        if (ls == null) {
            throw new NullPointerException("The provided linestring is null");
        }

        final int N = ls.getNumPoints();
        final boolean isLinearRing = ls instanceof LinearRing;

        List<Coordinate> retain = new ArrayList<>();
        retain.add(ls.getCoordinateN(0));

        int i0 = 0, i1 = 1, i2 = 2;
        Coordinate firstCoord = ls.getCoordinateN(i0);
        Coordinate midCoord;
        Coordinate lastCoord;
        while (i2 < N) {
            midCoord = ls.getCoordinateN(i1);
            lastCoord = ls.getCoordinateN(i2);

            final int orientation = Orientation.index(firstCoord, midCoord, lastCoord);
            // Colllinearity test
            if (orientation != Orientation.COLLINEAR) {
                // add midcoord and change head
                retain.add(midCoord);
                i0 = i1;
                firstCoord = ls.getCoordinateN(i0);
            }
            i1++;
            i2++;
        }
        retain.add(ls.getCoordinateN(N - 1));

        //
        // Return value
        //
        final int size = retain.size();
        // nothing changed?
        if (size == N) {
            // free everything and return original
            retain.clear();

            return ls;
        }

        return isLinearRing
                ? ls.getFactory().createLinearRing(retain.toArray(new Coordinate[size]))
                : ls.getFactory().createLineString(retain.toArray(new Coordinate[size]));
    }

    public static double distanceBetweenTwoPoints(double[] p0, double[] p1)  {
        return Math.sqrt(Math.pow(p1[0] - p0[0], 2) + Math.pow(p1[1] - p0[1], 2));
    }
    //endregion

    //region getter setter
    public void setGraphicsContext(GraphicsContext graphicsContext) {
        this.graphicsContext = graphicsContext;
    }

    //endregion

    //region test
    public void fillPoint(Vector v) {
        graphicsContext.setFill(Color.RED);
        graphicsContext.fillOval(v.getX(), v.getY(), 5, 5);
    }

    public void strokePolygonLine(Polygon polygon, boolean random) {
        if (polygon != null)    {
            if (random) {
                graphicsContext.setStroke(Color.color(Math.random(), Math.random(), Math.random()));
                graphicsContext.setLineWidth(1.0);
            }
            else {
                graphicsContext.setStroke(Color.YELLOW);
                graphicsContext.setLineWidth(2.0);
            }
            for (int i = 0; i < polygon.getVertices().size()-1; i++)    {
                Vector v0 = polygon.getVertices().get(i);
                Vector v1 = polygon.getVertices().get(i+1);
                graphicsContext.strokeLine(v0.getX(), v0.getY(), v1.getX(), v1.getY());
            }

            if (polygon.getVertices().size() > 2) {
                Vector v0 = polygon.getVertices().get(polygon.getVertices().size()-1);
                Vector v1 = polygon.getVertices().get(0);
                graphicsContext.strokeLine(v0.getX(), v0.getY(), v1.getX(), v1.getY());
            }

        }
    }

    public void strokeLine(List<Vector> vectors, boolean random)    {
        if (vectors != null)    {
            if (random) {
                graphicsContext.setStroke(Color.color(Math.random(), Math.random(), Math.random()));
            }
            else {
                graphicsContext.setStroke(Color.YELLOW);
            }
            graphicsContext.setLineWidth(1.0);

            for (int i = 0; i < vectors.size()-1; i++)    {
                Vector v0 = vectors.get(i);
                Vector v1 = vectors.get(i+1);
                graphicsContext.strokeLine(v0.getX(), v0.getY(), v1.getX(), v1.getY());
            }
        }
    }

    public void strokeLineString(LineString ls, boolean random) {
        if (ls != null) {
            List<Vector> vs = new ArrayList<>();
            for (Coordinate c : ls.getCoordinates())    {
                vs.add(new Vector(c.getX(), c.getY()));
            }

            strokeLine(vs, random);
        }
    }
    //endregion
}
