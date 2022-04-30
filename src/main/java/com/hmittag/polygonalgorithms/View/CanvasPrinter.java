package com.hmittag.polygonalgorithms.View;

import com.hmittag.polygonalgorithms.JTS.JtsHelper;
import com.hmittag.polygonalgorithms.Model.Polygon.Polygon;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import com.hmittag.polygonalgorithms.Waypoints.ConcaveWaypointsCalculator;
import com.hmittag.polygonalgorithms.Waypoints.LineStringDifference;
import com.hmittag.polygonalgorithms.Waypoints.ConvexWaypointsCalculator;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import org.locationtech.jts.geom.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class CanvasPrinter {

    //region fields
    private Canvas canvas;
    private GraphicsContext graphicsContext;

    private Polygon polygon;
    private ConcaveWaypointsCalculator concaveWaypointsCalculator;
    private ConvexWaypointsCalculator convexWaypointsCalculator;

    //endregion

    //region constructor
    public CanvasPrinter(Canvas canvas) {
        this.canvas = canvas;
        this.graphicsContext = canvas.getGraphicsContext2D();
        this.polygon = new Polygon(Polygon.SAMPLE_POLYGON_1);
        //this.polygon = new Polygon();
        this.concaveWaypointsCalculator = new ConcaveWaypointsCalculator();
        this.convexWaypointsCalculator = new ConvexWaypointsCalculator();
    }
    //endregion

    //region canvas
    public void setupCanvas()   {
        graphicsContext.setFill(Color.BLACK);
        graphicsContext.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());

        test();
        //calcTest();

        /*canvas.setOnMouseClicked(v -> {
            polygon.addPoint(new Vector(v.getX(), v.getY()));
            graphicsContext.setFill(Color.BLACK);
            graphicsContext.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());
            fillPolygonPoints();
            strokePolygonLine(polygon, false);

            System.out.println(polygon);
        });*/

        canvas.setOnMouseMoved(v -> {
            strokeMousePosText(new Vector(v.getX(), v.getY()));
        });

    }

    private void test() {
        this.concaveWaypointsCalculator.prepareForTesting(graphicsContext);

        this.concaveWaypointsCalculator.computeMission(this.polygon, 20, 0, 10, 60, ConvexWaypointsCalculator.StartingPosition.TOP_RIGHT);
    }

    /*private void calcTest() {
        //strokePolygonLine(this.polygon, false);

        //buffer op
        Geometry bufferedPolygon = this.polygon.bufferOp(-10);
        Polygon bufferedPolygon1 = JtsHelper.JTSPolygon2Polygon(bufferedPolygon);
        strokePolygonLine(bufferedPolygon1, true);
        Geometry bufferedPolygon2 = bufferedPolygon1.bufferOp(1);


        List<Vector> points = bufferedPolygon1.getVertices();
        //points.add(new Vector(bufferedPolygon.getVertices().get(0).getX(), bufferedPolygon.getVertices().get(0).getY()));
        List<Vector> rootMissionCoordinates = convexWaypointsCalculator.calculateWaypoints(points, 20, 0, 10, 60, ConvexWaypointsCalculator.StartingPosition.TOP_RIGHT);
        fillPoint(rootMissionCoordinates.get(0));
        strokeLine(rootMissionCoordinates, false);

        HashMap<Integer, LineStringDifference> intersections = new HashMap<>();
        GeometryFactory gf = new GeometryFactory();
        int biggesDifference = 0;
        int ticker = 0;
        int ticker1 = 0;
        for (int i = 0; i < rootMissionCoordinates.size()-1; i++) {
            Vector v = rootMissionCoordinates.get(i);
            Vector v1 = rootMissionCoordinates.get(i+1);
            //create linestring
            LineString lineString = gf.createLineString(new Coordinate[]  {new Coordinate(v.getX(), v.getY())
                    , new Coordinate(v1.getX(), v1.getY())});

            Geometry difference = lineString.difference(bufferedPolygon2);
            LineStringDifference lineStringDifference = new LineStringDifference(difference.getCoordinates(), false);

            switch (ticker) {
                case 0:
                    switch (ticker1)    {
                        case 0:
                            if (lineStringDifference.getCoordinates().length > 2)   {
                                Coordinate[] coords = new Coordinate[2];
                                coords[0] = lineStringDifference.getCoordinates()[0];
                                coords[1] = lineStringDifference.getCoordinates()[1];
                                lineStringDifference.setCoordinates(coords);
                            }
                            ticker1 = 1;
                            break;

                        case 1:
                            if (lineStringDifference.getCoordinates().length > 2)   {
                                Coordinate[] coords = new Coordinate[2];
                                coords[0] = lineStringDifference.getCoordinates()[lineStringDifference.getCoordinates().length-2];
                                coords[1] = lineStringDifference.getCoordinates()[lineStringDifference.getCoordinates().length-1];
                                lineStringDifference.setCoordinates(coords);
                            }
                            ticker1 = 0;
                            break;
                    }
                    ticker = 1;
                    break;

                case 1:
                    lineStringDifference.setSupportingLine(true);
                    ticker = 0;
                    break;
            }
            if (difference.getCoordinates().length > 0)    {
                intersections.put(i, lineStringDifference);
                if (difference.getCoordinates().length > biggesDifference)  {
                    biggesDifference = difference.getCoordinates().length;
                }
            }

        }
        List<List<Vector>> missions = new ArrayList<>();
        int cutIndex = 0;

        //first mission
        ticker = 0;
        boolean jumpOne = false;
        List<Vector> mission0 = new ArrayList<>();
        for (int i = 0; i < rootMissionCoordinates.size(); i++) {
            cutIndex = i;
            if (!intersections.containsKey(i))   {
                if (!jumpOne) {
                    mission0.add(rootMissionCoordinates.get(i));
                    ticker = 0;
                }
            }
            else {
                LineStringDifference lineStringDifference = intersections.get(i);
                Coordinate[] coords = lineStringDifference.getCoordinates();

                switch (ticker) {
                    case 0:
                        mission0.add(rootMissionCoordinates.get(i));
                        mission0.add(JtsHelper.JTSCoordinate2Vector(coords[0]));
                        jumpOne = true;
                        ticker = 1;
                        break;

                    case 1:
                        mission0.add(JtsHelper.JTSCoordinate2Vector(coords[coords.length-1]));
                        jumpOne = false;
                        ticker = 0;
                        break;
                }

                if (lineStringDifference.isSupportingLine())    {
                    biggesDifference++;
                    break;
                }
            }
        }
        strokeLine(mission0, true);

        for (int i = 1; i < biggesDifference; i++) {
            List<Vector> mission = new ArrayList<>();
            jumpOne = false;
            ticker = 0;
            if (!(cutIndex+1 < rootMissionCoordinates.size()))   {
                // mission got cut at the end

            }
            else {
                // mission got cut in the middle
                for (int j = cutIndex+1; j < rootMissionCoordinates.size(); j++)  {
                    cutIndex = j;
                    System.out.println("index " + j);
                    if (!intersections.containsKey(j))   {
                        if (!jumpOne) {
                            mission.add(rootMissionCoordinates.get(j));
                            ticker = 0;
                        }
                        System.out.println("not intersective");
                    }
                    else {
                        System.out.println("intersective");
                        LineStringDifference lineStringDifference = intersections.get(j);
                        Coordinate[] coords = lineStringDifference.getCoordinates();

                        switch (ticker) {
                            case 0:
                                mission.add(rootMissionCoordinates.get(j));
                                mission.add(JtsHelper.JTSCoordinate2Vector(coords[0]));
                                jumpOne = true;
                                ticker = 1;
                                break;

                            case 1:
                                mission.add(JtsHelper.JTSCoordinate2Vector(coords[coords.length-1]));
                                jumpOne = false;
                                ticker = 0;
                                break;
                        }

                        if (lineStringDifference.isSupportingLine())    {
                            System.out.println("lineString is supportive");

                            break;
                        }
                    }
                }
            }

            missions.add(mission);
            strokeLine(mission, true);

        }



        /*Bayazit bayazit = new Bayazit();
        List<Vector> correctedPolygon = bufferedPolygon.getVertices();
        correctedPolygon.remove(correctedPolygon.size()-1);
        List<Convex> convexDyn4JPolygons = bayazit.decompose(Dyn4JHelper.VectorListToVector2List(correctedPolygon));
        List<Polygon> convexPolygons = new ArrayList<>();*/

        /*for (Convex c : convexDyn4JPolygons) {
            org.dyn4j.geometry.Polygon polygon = (org.dyn4j.geometry.Polygon) c;

            Polygon p = Dyn4JHelper.dyn4JPolygonToPolygon(polygon);
            convexPolygons.add(p);
            strokePolygonLine(p, true);

            List<Vector> points = p.getVertices();
            points.add(new Vector(p.getVertices().get(0).getX(), p.getVertices().get(0).getY()));

            //List<Vector> vs = waypointsCalculator2.calculateWaypointsForLongestSide(points, 20, 0, 0, WaypointsCalculator.StartingPosition.BOTTOM_RIGHT);
            //List<Vector> vs = waypointsCalculator2.calculateWaypoints(points, 20, 0, 10, 70, WaypointsCalculator.StartingPosition.TOP_LEFT);

            //strokeLine(vs);
        }*/


        /*WaypointsCalculator.StartingPosition startingPosition = WaypointsCalculator.StartingPosition.TOP_RIGHT;

        //find first polygon according to starting points
        Polygon firstPolygon = null;
        switch (startingPosition)   {
            case BOTTOM_RIGHT:
                Vector bottomRight = convexPolygons.get(0).getVertices().get(0);
                Polygon bottomRightPolygon = convexPolygons.get(0);
                for (Polygon p : convexPolygons)    {
                    for (int i = 0; i < p.getVertices().size(); i++)   {
                        Vector v = p.getVertices().get(i);
                        if (v.getX() >= bottomRight.getX() && v.getY() >= bottomRight.getY()) {
                            bottomRight = v;
                            bottomRightPolygon = p;
                        }
                    }
                }

                firstPolygon = bottomRightPolygon;
                break;

            case BOTTOM_LEFT:
                Vector bottomLeft = convexPolygons.get(0).getVertices().get(0);
                Polygon bottomLeftPolygon = convexPolygons.get(0);
                for (Polygon p : convexPolygons)    {
                    for (int i = 0; i < p.getVertices().size(); i++)   {
                        Vector v = p.getVertices().get(i);
                        if (v.getX() <= bottomLeft.getX() && v.getY() >= bottomLeft.getY()) {
                            bottomLeft = v;
                            bottomLeftPolygon = p;
                        }
                    }
                }

                firstPolygon = bottomLeftPolygon;
                break;

            case TOP_RIGHT:
                Vector topRight = convexPolygons.get(0).getVertices().get(0);
                Polygon topRightPolygon = convexPolygons.get(0);
                for (Polygon p : convexPolygons)    {
                    for (int i = 0; i < p.getVertices().size(); i++)   {
                        Vector v = p.getVertices().get(i);
                        if (v.getX() >= topRight.getX() && v.getY() <= topRight.getY()) {
                            topRight = v;
                            topRightPolygon = p;
                        }
                    }
                }

                firstPolygon = topRightPolygon;
                break;

            case TOP_LEFT:
                Vector topLeft = convexPolygons.get(0).getVertices().get(0);
                Polygon topLeftPolygon = convexPolygons.get(0);
                for (Polygon p : convexPolygons)    {
                    for (int i = 0; i < p.getVertices().size(); i++)   {
                        Vector v = p.getVertices().get(i);
                        if (v.getX() <= topLeft.getX() && v.getY() <= topLeft.getY()) {
                            topLeft = v;
                            topLeftPolygon = p;
                        }
                    }
                }

                firstPolygon = topLeftPolygon;
                break;
        }

        List<Vector> points = firstPolygon.getVertices();

        //List<Vector> vs = waypointsCalculator2.calculateWaypointsForLongestSide(points, 20, 0, 0, WaypointsCalculator.StartingPosition.TOP_RIGHT);
        List<Vector> vs = waypointsCalculator2.calculateWaypoints(points, 20, 0, 10, 70, WaypointsCalculator.StartingPosition.TOP_LEFT);
        strokeLine(vs);*/




        /*Convex c = convexPolygons.get(0);
        System.out.println(c);
        org.dyn4j.geometry.Polygon polygon = (org.dyn4j.geometry.Polygon) c;

        Polygon p = Dyn4JHelper.dyn4JPolygonToPolygon(polygon);
        strokePolygonLine(p, true);

        strokeLine(waypointsCalculator.calculateWaypointsForLongestSide(p.getVertices(), 20, 0, 1, WaypointsCalculator.StartingPosition.BOTTOM_RIGHT));/

        //triangulate
        //GeometryCollection triangularCollection = bufferedPolygon.triangulate();




        //convex partitioning

    }*/

    public void fillPolygonPoints()   {
        graphicsContext.setFill(Color.RED);
        for (int i = 0; i < polygon.getVertices().size(); i++)    {
            Vector v = polygon.getVertices().get(i);
            graphicsContext.fillOval(v.getX(), v.getY(), 10, 10);
        }
    }

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

    private void strokeMousePosText(Vector v)  {
        if (v != null)  {
            graphicsContext.setFill(Color.BLACK);
            graphicsContext.fillRect(0, 0, 90, 40);
            graphicsContext.strokeText(v.toString(), 10, 10);
        }
    }
    //endregion
}
