package com.hmittag.polygonalgorithms.CoveragePathPlanning;

import com.hmittag.polygonalgorithms.Decompose.Decomposer;
import com.hmittag.polygonalgorithms.JTS.JtsHelper;
import com.hmittag.polygonalgorithms.Model.Polygon.Polygon;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import com.hmittag.polygonalgorithms.Waypoints.ConvexWaypointsCalculator;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.util.Pair;
import org.locationtech.jts.algorithm.Centroid;
import org.locationtech.jts.algorithm.Distance;
import org.locationtech.jts.algorithm.Orientation;
import org.locationtech.jts.algorithm.RobustLineIntersector;
import org.locationtech.jts.geom.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class CoveragePathPlanner {
    //region constants
    private static final double POINT_ON_LINE_THRESHOLD = 0.1;
    //endregion

    //region fields
    private PolygonLineSweeper polygonLineSweeper;
    private GeometryFactory geometryFactory;
    private Decomposer decomposer;

    //test
    private GraphicsContext graphicsContext;
    //endregion

    //region constructor
    public CoveragePathPlanner()    {
        geometryFactory = new GeometryFactory();
        polygonLineSweeper = new PolygonLineSweeper();
        decomposer = new Decomposer();
    }
    //endregion

    //region path planning
    public List<Vector> planPath(Polygon p, List<Polygon> holes, double workingWidth, double polygonBufferSized) {
        List<Vector> finalPath = new ArrayList<>();

        strokePolygonLine(p, false);

        Polygon bufferedPolygon = JtsHelper.JTSPolygon2Polygon(p.bufferOp(polygonBufferSized));
        bufferedPolygon.getVertices().remove(bufferedPolygon.getVertices().size()-1);

        this.polygonLineSweeper.sweep(bufferedPolygon, workingWidth);

        //List<Polygon> polygonList = decomposer.decompose(bufferedPolygon);
        //create coverage for each polygon
        /*for (Polygon part : polygonList)   {
            this.polygonLineSweeper.sweep(part, workingWidth);
        }*/
        //groupPolygonsAndCover(polygonList, workingWidth);

        /*List<Vector> joinedPath = createJoinedPath(polygonList);
        strokeLine(joinedPath, true);*/

        return finalPath;
    }

    private List<Vector> createJoinedPath(List<Polygon> polygonList)    {
        List<List<Vector>> finalPaths = new ArrayList<>();
        boolean computationFinished = false;
        List<List<Vector>> currentPathPool = new ArrayList<>();

        double minLength = Double.MAX_VALUE;
        int polygonIndex = polygonList.size()-1;
        while (!computationFinished) {
            for (int i = 0; i < polygonList.size(); i++) {
                Polygon p = polygonList.get(i);
                currentPathPool.add(p.getCoverPathsList().get(p.getCurrentCoverPathIndex()));

                computationFinished = handleCurrentCoverageIndexIncrement(polygonList);
            }
            double dist = computeCoveragePathsDistance(currentPathPool);
            if (dist < minLength)   {
                finalPaths = currentPathPool;
            }
            currentPathPool = new ArrayList<>();
        }

        //join paths together
        List<Vector> ret = new ArrayList<>();
        for (List<Vector> path : finalPaths)    {
            ret.addAll(path);
        }

        return ret;
    }

    private boolean handleCurrentCoverageIndexIncrement(List<Polygon> polygonList)  {
        boolean computationFinished = false;
        int counter = polygonList.size()-1;
        while (counter >= 0)  {
            Polygon p = polygonList.get(counter);
            int newIndex = p.getCurrentCoverPathIndex() + 1;
            if (newIndex > 3)   {
                newIndex = 0;
                p.setCurrentCoverPathIndex(newIndex);

                //finished ?
                if (counter == 0)   {
                    return true;
                }
            }
            else {
                p.setCurrentCoverPathIndex(newIndex);
                break;
            }
            counter--;
        }
        return false;
    }

    private double computeCoveragePathsDistance(List<List<Vector>> paths)    {
        double sum = 0;
        for (int i = 0; i < paths.size()-1; i++)  {
            sum += computePathLength(paths.get(i));
            sum += distanceBetweenTwoPoints(paths.get(i).get(paths.get(i).size()-1), paths.get(i+1).get(paths.get(i+1).size()-1));
            sum += computePathLength(paths.get(i+1));
        }
        return sum;
    }
    //endregion

    //region coverage
    private void groupPolygonsAndCover(List<Polygon> polygons, double workingWidth)  {
        for (int i = 0; i < polygons.size()-1; i++)   {
            Polygon p = polygons.get(i);
            for (int j = i+1; j < polygons.size(); j++)   {
                Polygon pRunner = polygons.get(j);

                if (p.neighborsPolygon(pRunner))    {
                    System.out.println("neighbor found");
                    Polygon fusedPolygon = Polygon.fusePolygon(p, pRunner);
                    System.out.println("fused polygon: " + fusedPolygon);
                    //check if uninterrupted path can be build
                    if (this.polygonLineSweeper.sweep(fusedPolygon, workingWidth))  {
                        polygons.set(i, fusedPolygon);
                        polygons.remove(j);
                    }
                }
            }
        }

        //cover the rest
        for (Polygon p : polygons)  {
            if (p.getCpp0() == null)    {
                this.polygonLineSweeper.sweep(p, workingWidth);
            }
            //strokePolygonLine(p, true);
            strokeLine(p.getCpp0(), true);
        }
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

    public static double distanceBetweenTwoPoints(Vector p0, Vector p1)  {
        return Math.sqrt(Math.pow(p1.getX() - p0.getX(), 2) + Math.pow(p1.getY() - p0.getY(), 2));
    }

    public static double computePathLength(List<Vector> path)    {
        double sum = 0;
        for (int i = 0; i < path.size()-1; i++)   {
            Vector v0 = path.get(i);
            Vector v1 = path.get(i+1);
            sum += distanceBetweenTwoPoints(v0, v1);
        }

        return sum;
    }
    //endregion

    //region getter setter
    public void setGraphicsContext(GraphicsContext graphicsContext) {
        this.graphicsContext = graphicsContext;
        this.polygonLineSweeper.setGraphicsContext(graphicsContext);
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
