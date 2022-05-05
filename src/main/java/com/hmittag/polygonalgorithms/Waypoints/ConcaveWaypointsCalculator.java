package com.hmittag.polygonalgorithms.Waypoints;

import com.hmittag.polygonalgorithms.DYN4J.Dyn4JHelper;
import com.hmittag.polygonalgorithms.JTS.JtsHelper;
import com.hmittag.polygonalgorithms.Model.Polygon.Polygon;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.decompose.Bayazit;
import org.dyn4j.geometry.decompose.EarClipping;
import org.dyn4j.geometry.decompose.SweepLine;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;

import java.util.ArrayList;
import java.util.List;

public class ConcaveWaypointsCalculator {

    //region fields
    private ConvexWaypointsCalculator convexWaypointsCalculator;

    //test
    private GraphicsContext graphicsContext;
    private int test = 0;
    //endregion

    //region consturctor
    public ConcaveWaypointsCalculator() {
        this.convexWaypointsCalculator = new ConvexWaypointsCalculator();
    }
    //endregion

    //region compute
    public List<Vector> computeWaypoints(Polygon polygon, double distanceBetweenLines, double distanceToBoundaries, double linesOffset, double angle, ConvexWaypointsCalculator.StartingPosition startingPosition)  {
        List<Vector> finalMissionPointsList = new ArrayList<>();
        if (polygon == null) {

            return finalMissionPointsList;
        }

        strokePolygonLine(polygon, false);
        Geometry bufferedPolygon = polygon.bufferOp(-10);
        List<Polygon> polygonParts = convexPartition(JtsHelper.JTSPolygon2Polygon(bufferedPolygon));

        polygonParts.add(0, polygonParts.remove(findFirstPolygon(polygonParts, startingPosition)));
        //groupSubMissions(polygonParts, distanceBetweenLines, distanceToBoundaries, linesOffset, angle, startingPosition);



        for (int i = 0; i < polygonParts.size(); i++)   {
            Polygon p = polygonParts.get(i);
            p = JtsHelper.JTSPolygon2Polygon(p.bufferOp(-2));
            for (int j = 0; j < polygonParts.size(); j++)   {

            }
            List<Vector> rootPolygonVertices = p.getVertices();

            List<Vector> rootMissionCoordinates = convexWaypointsCalculator.calculateWaypointsForLongestSide(rootPolygonVertices, distanceBetweenLines, distanceToBoundaries, 0,/* 20,*/ startingPosition);
            fillPoint(rootMissionCoordinates.get(0));
            strokeLine(rootMissionCoordinates, true);
        }


        return null;
    }

    private List<List<Vector>> groupSubMissions(List<Polygon> polygonList, double distanceBetweenLines, double distanceToBoundaries, double linesOffset, double angle, ConvexWaypointsCalculator.StartingPosition startingPosition)   {
        List<List<Vector>> missions = new ArrayList<>();
        //List<Vector> firstMissionPoints = convexWaypointsCalculator.calculateWaypoints(rootPolygonVertices, distanceBetweenLines, distanceToBoundaries, linesOffset, angle, startingPosition);
        for (int i = 0; i < polygonList.size(); i++)    {
            Geometry rootPolygon = JtsHelper.polygon2JTSPolygon(polygonList.get(i));
            for (int j = 0; j < polygonList.size(); j++)    {
                Geometry runnerPolygon = JtsHelper.polygon2JTSPolygon(polygonList.get(j));

                //check difference
                if (runnerPolygon.intersects(rootPolygon))  {
                    List<Vector> rootMission = convexWaypointsCalculator.calculateWaypoints(polygonList.get(i).getVertices(), distanceBetweenLines, distanceToBoundaries, linesOffset, angle, startingPosition);
                    List<Vector> runnerMission = convexWaypointsCalculator.calculateWaypoints(polygonList.get(j).getVertices(), distanceBetweenLines, distanceToBoundaries, linesOffset, angle, startingPosition);
                    joinTwoMissions(rootMission, runnerMission);

                }
            }
            List<Vector> rootPolygonVertices = polygonList.get(i).getVertices();
            List<Vector> missionPoints = convexWaypointsCalculator.calculateWaypoints(rootPolygonVertices, distanceBetweenLines, distanceToBoundaries, linesOffset, angle, startingPosition);
        }

        return missions;
    }

    private void joinTwoMissions(List<Vector> m1, List<Vector> m2)   {
        GeometryFactory gf = new GeometryFactory();
        //find common vector

        List<Vector> rootTopPoints = new ArrayList<>();
        List<Vector> rootBotPoints = new ArrayList<>();
        Vector v0RootStart = m1.get(1);
        Vector v0RootEnd = m1.get(2);
        rootTopPoints.add(v0RootStart);

        Vector v1RootStart = m1.get(3);
        Vector v1RootEnd = m1.get(4);
        rootBotPoints.add(v1RootStart);


        List<Vector> runnerTopPoints = new ArrayList<>();
        List<Vector> runnerBotPoints = new ArrayList<>();
        Vector v0RunnerStart = m2.get(1);
        Vector v0RunnerEnd = m2.get(2);
        runnerTopPoints.add(v0RunnerStart);

        Vector v1RunnerStart = m2.get(3);
        Vector v1RunnerEnd = m2.get(4);
        runnerBotPoints.add(v1RunnerStart);


        //m1 line string
        int ticker = 1;
        try {
            for (int i = 4; i < m1.size() - 1; i += 2) {
                switch (ticker) {
                    case 0:
                        v0RootEnd = m1.get(i);
                        rootTopPoints.add(m1.get(i-1));
                        rootTopPoints.add(m1.get(i));
                        ticker = 1;
                        break;

                    case 1:
                        v1RootEnd = m1.get(i);
                        rootBotPoints.add(m1.get(i-1));
                        rootBotPoints.add(m1.get(i));
                        ticker = 0;
                        break;
                }
            }
        }
        catch (IndexOutOfBoundsException e) {
        }


        LineString lineString0Root = gf.createLineString(new Coordinate[]{new Coordinate(v0RootStart.getX(), v0RootStart.getY())
                , new Coordinate(v0RootEnd.getX(), v0RootEnd.getY())});
        LineString lineString1Root = gf.createLineString(new Coordinate[]{new Coordinate(v1RootStart.getX(), v1RootStart.getY())
                , new Coordinate(v1RootEnd.getX(), v1RootEnd.getY())});

        //m2 linestring
        ticker = 1;
        try {
            for (int i = 4; i < m2.size() - 1; i += 2) {
                switch (ticker) {
                    case 0:
                        v0RunnerEnd = m2.get(i);
                        runnerTopPoints.add(m2.get(i-1));
                        runnerTopPoints.add(m2.get(i));
                        ticker = 1;
                        break;

                    case 1:
                        v1RunnerEnd = m2.get(i);
                        runnerBotPoints.add(m2.get(i-1));
                        runnerBotPoints.add(m2.get(i));
                        ticker = 0;
                        break;
                }
            }
        }
        catch (IndexOutOfBoundsException e) {
        }

        LineString lineString0Runner = gf.createLineString(new Coordinate[]{new Coordinate(v0RunnerStart.getX(), v0RunnerStart.getY())
                , new Coordinate(v0RunnerEnd.getX(), v0RunnerEnd.getY())});
        LineString lineString1Runner = gf.createLineString(new Coordinate[]{new Coordinate(v1RunnerStart.getX(), v1RunnerStart.getY())
                , new Coordinate(v1RunnerEnd.getX(), v1RunnerEnd.getY())});


        //check intersection
        if (lineString0Root.intersects(lineString0Runner))  {
            Vector rootV = rootTopPoints.get(2);
            for (int i = 0; i < runnerTopPoints.size(); i++)    {
            }
        }
        else if (lineString0Root.intersects(lineString1Runner)) {

        }
        else if (lineString1Root.intersects(lineString0Runner)) {

        }
        else if (lineString1Root.intersects(lineString1Runner)) {

        }
        else {
            // lines are parallel

        }

    }

    private List<Polygon> convexPartition(final Polygon polygon) {
        List<Polygon> parts = new ArrayList<>();

        if (polygon != null)    {

            EarClipping earClipping = new EarClipping();
            SweepLine sweepLine = new SweepLine();

            Bayazit bayazit = new Bayazit();
            List<Vector> correctedPolygon = polygon.getVertices();
            correctedPolygon.remove(correctedPolygon.size()-1);
            //List<Convex> convexDyn4JPolygons = bayazit.decompose(Dyn4JHelper.VectorListToVector2List(correctedPolygon));
            List<Convex> convexDyn4JPolygons = bayazit.decompose(Dyn4JHelper.VectorListToVector2List(correctedPolygon));

            for (Convex c : convexDyn4JPolygons) {
                org.dyn4j.geometry.Polygon dyn4jPolygon = (org.dyn4j.geometry.Polygon) c;

                Polygon p = Dyn4JHelper.dyn4JPolygonToPolygon(dyn4jPolygon);
                strokePolygonLine(p, true);
                parts.add(p);

                List<Vector> points = p.getVertices();
                points.add(new Vector(p.getVertices().get(0).getX(), p.getVertices().get(0).getY()));
            }
        }

        return parts;
    }

    private int findFirstPolygon(List<Polygon> polygonList, ConvexWaypointsCalculator.StartingPosition startingPosition) {
        Polygon firstPolygon = null;
        switch (startingPosition)   {
            case BOTTOM_RIGHT:
                Vector bottomRight = polygonList.get(0).getVertices().get(0);
                Polygon bottomRightPolygon = polygonList.get(0);
                for (Polygon p : polygonList)    {
                    for (int i = 0; i < p.getVertices().size(); i++)   {
                        Vector v = p.getVertices().get(i);
                        if (v.getX() >= bottomRight.getX() && v.getY() >= bottomRight.getY()) {
                            bottomRight = v;
                            bottomRightPolygon = p;
                        }
                    }
                }

                return polygonList.indexOf(bottomRightPolygon);

            case BOTTOM_LEFT:
                Vector bottomLeft = polygonList.get(0).getVertices().get(0);
                Polygon bottomLeftPolygon = polygonList.get(0);
                for (Polygon p : polygonList)    {
                    for (int i = 0; i < p.getVertices().size(); i++)   {
                        Vector v = p.getVertices().get(i);
                        if (v.getX() <= bottomLeft.getX() && v.getY() >= bottomLeft.getY()) {
                            bottomLeft = v;
                            bottomLeftPolygon = p;
                        }
                    }
                }

                return polygonList.indexOf(bottomLeftPolygon);

            case TOP_RIGHT:
                Vector topRight = polygonList.get(0).getVertices().get(0);
                Polygon topRightPolygon = polygonList.get(0);
                for (Polygon p : polygonList)    {
                    for (int i = 0; i < p.getVertices().size(); i++)   {
                        Vector v = p.getVertices().get(i);
                        if (v.getX() >= topRight.getX() && v.getY() <= topRight.getY()) {
                            topRight = v;
                            topRightPolygon = p;
                        }
                    }
                }

                return polygonList.indexOf(topRightPolygon);

            case TOP_LEFT:
                Vector topLeft = polygonList.get(0).getVertices().get(0);
                Polygon topLeftPolygon = polygonList.get(0);
                for (Polygon p : polygonList)    {
                    for (int i = 0; i < p.getVertices().size(); i++)   {
                        Vector v = p.getVertices().get(i);
                        if (v.getX() <= topLeft.getX() && v.getY() <= topLeft.getY()) {
                            topLeft = v;
                            topLeftPolygon = p;
                        }
                    }
                }

                firstPolygon = topLeftPolygon;
                return polygonList.indexOf(topLeftPolygon);
        }
        return -1;
    }

    //endregion

    //region test
    public void prepareForTesting(GraphicsContext c)    {
        this.graphicsContext = c;
    }

    private void fillPoint(Vector v) {
        graphicsContext.setFill(Color.RED);
        graphicsContext.fillOval(v.getX(), v.getY(), 5, 5);
    }

    private void strokePolygonLine(Polygon polygon, boolean random) {
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

    private void strokeLine(List<Vector> vectors, boolean random)    {
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
    //endregion
}
